using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    internal sealed class CutCellGpuContactTraction : IDisposable
    {
        private const float Guard = 9876.25f;
        private readonly VariableCoefficientProjection plan;
        private readonly ComputeShader shader;
        private readonly ComputeBuffer inputs, output;
        private readonly Vector4[] packed;
        private readonly float[] readback;
        private readonly int kernel, groups;
        internal int Dispatches { get; private set; }

        internal CutCellGpuContactTraction(VariableCoefficientProjection plan, ComputeShader shader)
        {
            this.plan = plan;
            this.shader = shader;
            kernel = shader.FindKernel("ApplyContactTraction");
            if (!shader.IsSupported(kernel))
                throw new NotSupportedException("Contact traction compute kernel is unsupported.");
            int count = plan.FaceCount;
            groups = (int)((count + 63L) / 64);
            if (groups > 65535 || 68L * count + 4 > 512L * 1024 * 1024)
                throw new NotSupportedException("Contact traction GPU dispatch or allocation exceeds its limit.");
            packed = new Vector4[4 * count];
            readback = new float[count + 1];
            try
            {
                inputs = new ComputeBuffer(packed.Length, 16, ComputeBufferType.Structured);
                output = new ComputeBuffer(readback.Length, 4, ComputeBufferType.Structured);
                output.SetData(new[] { Guard }, 0, count, 1);
            }
            catch { Dispose(); throw; }
        }

        internal double[] Apply(ContactTractionWork work)
        {
            if (work.Plan != plan) throw new ArgumentException("Contact traction topology mismatch.");
            for (int f = 0; f < plan.FaceCount; f++)
            {
                var face = plan.Faces[f];
                PackSide(f, face.Low, 4 * f);
                PackSide(f, face.High, 4 * f + 2);
            }
            inputs.SetData(packed);
            shader.SetInt("FaceCount", plan.FaceCount);
            shader.SetBuffer(kernel, "ContactData", inputs);
            shader.SetBuffer(kernel, "FaceImpulse", output);
            shader.Dispatch(kernel, groups, 1, 1);
            output.GetData(readback);
            if (readback[plan.FaceCount] != Guard)
                throw new InvalidOperationException("GPU contact traction buffer guard was overwritten.");
            var reference = work.ReferenceImpulses();
            var result = new double[plan.FaceCount];
            for (int f = 0; f < result.Length; f++)
            {
                result[f] = readback[f];
                double scale = Magnitude(4 * f) + Magnitude(4 * f + 2);
                if (!SiUnits.IsFinite(result[f]) || Math.Abs(result[f] - reference[f]) > 1e-15 + 1e-6 * scale)
                    throw new InvalidOperationException($"GPU contact traction differs from CPU on face {f}.");
            }
            Dispatches++;
            return result;

            void PackSide(int face, int group, int offset)
            {
                packed[offset] = Pack(work.WeightedRow(face, group));
                packed[offset + 1] = group < 0 ? default : Pack(work.Samples[group]);
            }

            double Magnitude(int offset)
            {
                Vector4 a = packed[offset], b = packed[offset + 1];
                return Math.Abs((double)a.x * b.x) + Math.Abs((double)a.y * b.y) + Math.Abs((double)a.z * b.z);
            }
        }

        private static Vector4 Pack(D3 value)
        {
            var packed = new Vector4((float)value.X, (float)value.Y, (float)value.Z, 0);
            if (!SiUnits.IsFinite(packed.x) || !SiUnits.IsFinite(packed.y) || !SiUnits.IsFinite(packed.z))
                throw new InvalidOperationException("Contact traction inputs exceed FP32 range.");
            return packed;
        }

        public void Dispose()
        {
            output?.Release();
            inputs?.Release();
        }
    }
}
