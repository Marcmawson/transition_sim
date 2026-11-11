using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.InteropServices;
using PhysicsSim.ThreePhase3D.Core;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Flow
{
    public sealed class FlowGpuSolver : IDisposable
    {
        private const long BufferBudgetBytes = 512L * 1024 * 1024;
        private const float Guard = -123456.75f;
        private readonly MomentumPredictor3D predictor;
        private readonly VariableCoefficientProjection plan;
        private readonly ComputeShader shader;
        private readonly int predictKernel, pressureKernel, correctKernel;
        private readonly ComputeBuffer faces, stencil, viscousRates, rows, incidence, pressureRows;
        private readonly ComputeBuffer velocity, predicted, corrected, pressureA, pressureB;
        private readonly List<ComputeBuffer> owned = new List<ComputeBuffer>();
        private readonly float[] predictedReadback, correctedReadback;
        private readonly Vector2[] pressureReadback, pressureZero;
        private readonly Vector4[] pressureRowData;
        private bool disposed;

        public long AllocatedBytes { get; private set; }
        public double PredictionWallMs { get; private set; }
        public double IterationWallMs { get; private set; }
        public double CorrectionWallMs { get; private set; }

        public FlowGpuSolver(MomentumPredictor3D predictor, ComputeShader shader)
            : this(predictor?.Plan ?? throw new ArgumentNullException(nameof(predictor)), predictor, shader) { }

        public static FlowStep Project(VariableCoefficientProjection plan, ComputeShader shader,
            float[] tentative, double dt, int maxIterations)
        {
            if (plan == null) throw new ArgumentNullException(nameof(plan));
            using (var solver = new FlowGpuSolver(plan, null, shader))
                return solver.Project(tentative, dt, maxIterations);
        }

        private FlowGpuSolver(VariableCoefficientProjection plan, MomentumPredictor3D predictor, ComputeShader shader)
        {
            this.plan = plan;
            this.predictor = predictor;
            if (shader == null) throw new ArgumentNullException(nameof(shader));
            if (!SystemInfo.supportsComputeShaders) throw new NotSupportedException("S04 requires compute shaders.");
            this.shader = shader;

            try
            {
                predictKernel = predictor == null ? -1 : shader.FindKernel("PredictMomentum");
                pressureKernel = shader.FindKernel("IteratePressure");
                correctKernel = shader.FindKernel("CorrectVelocity");
                Groups(plan.FaceCount);
                Groups(plan.GroupCount);
                int faceStride = Marshal.SizeOf<VariableCoefficientProjection.Face>();
                if (faceStride != 32) throw new InvalidOperationException("Flow face layout must match the 32-byte shader struct.");
                long requiredBytes = (faceStride + 12L) * plan.FaceCount +
                    4L * ((predictor?.Stencil.Length ?? 0) + (long)(predictor?.Diffusion.Rates.Length ?? 0) + plan.Rows.Length + Math.Max(1, plan.Incidence.Length)) +
                    32L * plan.GroupCount + 24;
                if (requiredBytes > BufferBudgetBytes)
                    throw new NotSupportedException("S04 buffer allocation exceeds its 512 MiB payload budget.");
                faces = Upload(plan.Faces, faceStride);
                if (predictor != null)
                {
                    stencil = Upload(predictor.Stencil, 4);
                    viscousRates = Upload(predictor.Diffusion.Rates, 4);
                    velocity = Allocate(plan.FaceCount, 4);
                }
                rows = Upload(plan.Rows, 4);
                incidence = Upload(plan.Incidence, 4);
                pressureRows = Allocate(plan.GroupCount, 16);
                pressureRowData = new Vector4[plan.GroupCount];
                predicted = Allocate(plan.FaceCount + 1, 4);
                corrected = Allocate(plan.FaceCount + 1, 4);
                pressureA = Allocate(plan.GroupCount + 1, 8);
                pressureB = Allocate(plan.GroupCount + 1, 8);

                pressureReadback = new Vector2[pressureA.count];
                pressureZero = new Vector2[plan.GroupCount];
                predictedReadback = new float[predicted.count];
                correctedReadback = new float[corrected.count];
                var sentinel = new[] { Guard };
                foreach (ComputeBuffer buffer in new[] { predicted, corrected })
                    buffer.SetData(sentinel, 0, buffer.count - 1, 1);
                var pressureSentinel = new[] { new Vector2(Guard, Guard) };
                foreach (ComputeBuffer buffer in new[] { pressureA, pressureB })
                    buffer.SetData(pressureSentinel, 0, buffer.count - 1, 1);
            }
            catch
            {
                Dispose();
                throw;
            }
        }

        public FlowStep Advance(float[] input, D3 acceleration, double requestedSeconds, int maxIterations = 4096)
        {
            if (disposed) throw new ObjectDisposedException(nameof(FlowGpuSolver));
            if (maxIterations <= 0) throw new ArgumentOutOfRangeException(nameof(maxIterations));

            if (predictor == null) throw new InvalidOperationException("This solver was created for projection only.");
            double dt = predictor.SelectTimestep(input, acceleration, requestedSeconds);
            D3 h = plan.Grid.Spacing, a = predictor.LocalAcceleration(acceleration);
            shader.SetInt("FaceCount", plan.FaceCount);
            shader.SetInt("GroupCount", plan.GroupCount);
            shader.SetFloat("Dt", (float)dt);
            shader.SetVector("InverseSpacing", FloatVector(1 / h.X, 1 / h.Y, 1 / h.Z));
            shader.SetVector("Acceleration", FloatVector(a.X, a.Y, a.Z));
            velocity.SetData(input);

            var clock = Stopwatch.StartNew();
            shader.SetBuffer(predictKernel, "Faces", faces);
            shader.SetBuffer(predictKernel, "Stencil", stencil);
            shader.SetBuffer(predictKernel, "ViscousRates", viscousRates);
            shader.SetBuffer(predictKernel, "VelocityIn", velocity);
            shader.SetBuffer(predictKernel, "PredictedOut", predicted);
            Dispatch(predictKernel, plan.FaceCount);
            predicted.GetData(predictedReadback);
            PredictionWallMs += clock.Elapsed.TotalMilliseconds;
            CheckReadback(predictedReadback);

            return SolvePressure(dt, maxIterations);
        }

        private FlowStep Project(float[] tentative, double dt, int maxIterations)
        {
            if (disposed) throw new ObjectDisposedException(nameof(FlowGpuSolver));
            if (maxIterations <= 0 || tentative.Length != plan.FaceCount || !SiUnits.IsFinite(dt) || dt <= 0)
                throw new ArgumentException("Invalid projection input.");
            shader.SetInt("FaceCount", plan.FaceCount);
            shader.SetInt("GroupCount", plan.GroupCount);
            shader.SetFloat("Dt", (float)dt);
            if (!Array.TrueForAll(tentative, value => SiUnits.IsFinite(value)))
                throw new ArgumentException("Nonfinite tentative velocity.");
            Array.Copy(tentative, predictedReadback, tentative.Length);
            predicted.SetData(tentative, 0, 0, tentative.Length);
            return SolvePressure(dt, maxIterations);
        }

        private FlowStep SolvePressure(double dt, int maxIterations)
        {
            pressureA.SetData(pressureZero, 0, 0, pressureZero.Length);
            var clock = new Stopwatch();
            shader.SetBuffer(pressureKernel, "Faces", faces);
            shader.SetBuffer(pressureKernel, "Rows", rows);
            shader.SetBuffer(pressureKernel, "Incidence", incidence);
            for (int group = 0; group < plan.GroupCount; group++)
            {
                double flux = plan.OutwardFlux(group, predictedReadback) / dt;
                float high = (float)flux;
                pressureRowData[group] = new Vector4(plan.Diagonal[group], high, (float)(flux - high), 0);
            }
            pressureRows.SetData(pressureRowData);
            shader.SetBuffer(pressureKernel, "PressureRows", pressureRows);
            shader.SetBuffer(correctKernel, "Faces", faces);
            shader.SetBuffer(correctKernel, "PredictedIn", predicted);
            shader.SetBuffer(correctKernel, "CorrectedOut", corrected);

            var pressure = new double[plan.GroupCount];
            ComputeBuffer current = pressureA, next = pressureB;
            int iteration = 0;
            while (true)
            {
                clock.Restart();
                current.GetData(pressureReadback);
                IterationWallMs += clock.Elapsed.TotalMilliseconds;
                var tail = pressureReadback[pressureReadback.Length - 1];
                if (tail.x != Guard || tail.y != Guard)
                    throw new InvalidOperationException("Flow shader overwrote the pressure buffer tail guard.");
                for (int group = 0; group < pressure.Length; group++)
                {
                    var pair = pressureReadback[group];
                    pressure[group] = (double)pair.x + pair.y;
                    if (!SiUnits.IsFinite(pressure[group]))
                        throw new InvalidOperationException("Flow shader produced a nonfinite pressure.");
                }

                clock.Restart();
                shader.SetBuffer(correctKernel, "PressureIn", current);
                Dispatch(correctKernel, plan.FaceCount);
                corrected.GetData(correctedReadback);
                CorrectionWallMs += clock.Elapsed.TotalMilliseconds;
                CheckReadback(correctedReadback);
                var diagnostics = plan.Measure(predictedReadback, correctedReadback, pressure, dt, iteration);
                if (diagnostics.Converged)
                {
                    var result = new float[plan.FaceCount];
                    Array.Copy(correctedReadback, result, result.Length);
                    plan.SetClosedGauges(pressure);
                    return new FlowStep(result, pressure, diagnostics);
                }
                if (iteration == maxIterations) throw new FlowConvergenceException(diagnostics);

                clock.Restart();
                int block = Math.Min(31, maxIterations - iteration);
                for (int step = 0; step < block; step++)
                {
                    shader.SetBuffer(pressureKernel, "PressureIn", current);
                    shader.SetBuffer(pressureKernel, "PressureOut", next);
                    Dispatch(pressureKernel, plan.GroupCount);
                    var swap = current;
                    current = next;
                    next = swap;
                }
                IterationWallMs += clock.Elapsed.TotalMilliseconds;
                iteration += block;
            }
        }

        private static Vector4 FloatVector(double x, double y, double z)
        {
            var result = new Vector4((float)x, (float)y, (float)z, 0);
            if (float.IsInfinity(result.x) || float.IsInfinity(result.y) || float.IsInfinity(result.z))
                throw new ArgumentException("Flow vector exceeds FP32 range.");
            return result;
        }

        private static void CheckReadback(float[] data)
        {
            if (data[data.Length - 1] != Guard) throw new InvalidOperationException("Flow shader overwrote a buffer tail guard.");
            if (!Array.TrueForAll(data, value => SiUnits.IsFinite(value)))
                throw new InvalidOperationException("Flow shader produced a nonfinite result.");
        }

        private static int Groups(int count)
        {
            int groups = (int)((count + 63L) / 64);
            if (groups > 65535) throw new NotSupportedException("S04 grid exceeds the one-dimensional dispatch limit.");
            return groups;
        }

        private void Dispatch(int kernel, int count) => shader.Dispatch(kernel, Groups(count), 1, 1);

        private ComputeBuffer Upload<T>(T[] values, int stride)
        {
            var buffer = Allocate(Math.Max(1, values.Length), stride);
            if (values.Length != 0) buffer.SetData(values);
            return buffer;
        }

        private ComputeBuffer Allocate(int count, int stride)
        {
            var buffer = new ComputeBuffer(count, stride, ComputeBufferType.Structured);
            owned.Add(buffer);
            AllocatedBytes += (long)count * stride;
            return buffer;
        }

        public void Dispose()
        {
            if (disposed) return;
            disposed = true;
            foreach (var buffer in owned) buffer.Release();
            owned.Clear();
            AllocatedBytes = 0;
        }
    }
}
