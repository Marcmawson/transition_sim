using System;
using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    public sealed class CutCellGpuMomentum : IDisposable
    {
        private readonly VariableCoefficientProjection plan;
        private readonly ComputeShader shader;
        private readonly int kernel, dispatch;
        private readonly ComputeBuffer faces, rows, incidence, groups, input, output;
        private readonly List<ComputeBuffer> owned = new List<ComputeBuffer>();
        private readonly VofFloatPair[] groupData, faceData, results;

        public CutCellGpuMomentum(VariableCoefficientProjection plan, ComputeShader shader)
        {
            this.plan = plan ?? throw new ArgumentNullException(nameof(plan));
            if (shader == null || !SystemInfo.supportsComputeShaders)
                throw new NotSupportedException("Cut-cell momentum compute is unavailable.");
            this.shader = shader;
            kernel = shader.FindKernel("RemapCutMomentum");
            if (!shader.IsSupported(kernel))
                throw new NotSupportedException("Cut-cell momentum compute is unsupported on " + SystemInfo.graphicsDeviceType);
            int ng = plan.GroupCount, nf = plan.FaceCount;
            dispatch = (int)((nf + 63L) / 64);
            long bytes = 8L * (5L * ng + 18L * nf + 1) + 4L * (plan.Rows.Length + (long)Math.Max(1, plan.Incidence.Length));
            if (dispatch > 65535 || bytes > 512L * 1024 * 1024)
                throw new NotSupportedException("Cut-cell momentum GPU dispatch or allocation exceeds its limit.");
            groupData = new VofFloatPair[5 * ng];
            faceData = new VofFloatPair[6 * nf];
            results = new VofFloatPair[10 * nf + 1];
            try
            {
                faces = Allocate(nf, 16);
                var faceRecords = new FaceRecord[nf];
                for (int f = 0; f < nf; f++)
                {
                    var face = plan.Faces[f];
                    faceRecords[f] = new FaceRecord { Low = face.Low, High = face.High, Axis = face.Axis, Active = face.Active };
                    faceData[f] = new VofFloatPair(face.Area);
                }
                faces.SetData(faceRecords);
                rows = Allocate(plan.Rows.Length, 4);
                rows.SetData(plan.Rows);
                incidence = Allocate(Math.Max(1, plan.Incidence.Length), 4);
                if (plan.Incidence.Length != 0) incidence.SetData(plan.Incidence);
                groups = Allocate(groupData.Length, 8);
                input = Allocate(faceData.Length, 8);
                output = Allocate(results.Length, 8);
                output.SetData(new[] { new VofFloatPair(VofGpuTransportLayout.Guard) }, 0, results.Length - 1, 1);
                for (int c = 0; c < ng; c++)
                {
                    var areas = new double[3];
                    for (int e = plan.Rows[c]; e < plan.Rows[c+1]; e++)
                    {
                        var face = plan.Faces[Math.Abs(plan.Incidence[e])-1];
                        areas[face.Axis] += face.Area;
                    }
                    for (int a = 0; a < 3; a++) groupData[(2+a)*ng+c] = new VofFloatPair(areas[a]);
                }
            }
            catch { Dispose(); throw; }
        }

        public CutCellMomentumResult Advance(CutCellInventories before, CutCellTransportResult after,
            double[] velocity, double liquidDensity, double gasDensity)
        {
            if (owned.Count == 0) throw new ObjectDisposedException(nameof(CutCellGpuMomentum));
            int ng = plan.GroupCount, nf = plan.FaceCount;
            if (before == null || after == null || after.Plan != plan || velocity == null || velocity.Length != nf ||
                before.LiquidM3.Length != ng || before.GasM3.Length != ng ||
                !SiUnits.IsFinite(liquidDensity) || !SiUnits.IsFinite(gasDensity) || liquidDensity <= 0 || gasDensity <= 0)
                throw new ArgumentException("Invalid cut-cell GPU momentum inputs.");
            for (int c = 0; c < ng; c++)
            {
                groupData[c] = new VofFloatPair(liquidDensity * before.LiquidM3[c] + gasDensity * before.GasM3[c]);
                groupData[ng+c] = new VofFloatPair(liquidDensity * after.Fields.LiquidM3[c] + gasDensity * after.Fields.GasM3[c]);
            }
            for (int f = 0; f < nf; f++)
            {
                if (plan.Faces[f].Active == 0 && velocity[f] != 0)
                    throw new ArgumentException("Inactive momentum face has nonzero velocity.");
                faceData[nf+f] = new VofFloatPair(velocity[f]);
                faceData[2*nf+f] = new VofFloatPair(liquidDensity * after.LiquidFaceM3[f] + gasDensity * after.GasFaceM3[f]);
                for (int a = 0; a < 3; a++)
                    faceData[(3+a)*nf+f] = new VofFloatPair(after.Reservoir[f].VelocityMPerS[a]);
            }
            groups.SetData(groupData);
            input.SetData(faceData);
            shader.SetInt("GroupCount", ng);
            shader.SetInt("FaceCount", nf);
            shader.SetBuffer(kernel, "Faces", faces);
            shader.SetBuffer(kernel, "Rows", rows);
            shader.SetBuffer(kernel, "Incidence", incidence);
            shader.SetBuffer(kernel, "GroupData", groups);
            shader.SetBuffer(kernel, "FaceData", input);
            shader.SetBuffer(kernel, "Output", output);
            shader.Dispatch(kernel, dispatch, 1, 1);
            output.GetData(results);
            if (results[results.Length-1].Value != VofGpuTransportLayout.Guard)
                throw new InvalidOperationException("Cut-cell momentum buffer guard was overwritten.");
            var u = new double[nf];
            var p = new double[nf];
            var boundary = new double[3];
            var wall = new double[3];
            double error = 0;
            for (int f = 0; f < nf; f++)
            {
                if (Read(3, f) != 0) throw new InvalidOperationException($"GPU staggered mass does not close on face {f}.");
                u[f] = Read(0, f);
                p[f] = Read(1, f);
                error = Math.Max(error, Read(2, f));
                for (int a = 0; a < 3; a++)
                {
                    boundary[a] += Read(4+a, f);
                    wall[a] += Read(7+a, f);
                }
            }
            return new CutCellMomentumResult(u, p, boundary, wall, error);

            double Read(int plane, int f)
            {
                double value = results[plane*nf+f].Value;
                if (!SiUnits.IsFinite(value)) throw new InvalidOperationException("Nonfinite GPU cut-cell momentum.");
                return value;
            }
        }

        [System.Runtime.InteropServices.StructLayout(System.Runtime.InteropServices.LayoutKind.Sequential)]
        private struct FaceRecord { public int Low, High, Axis, Active; }

        private ComputeBuffer Allocate(int count, int stride)
        {
            var buffer = new ComputeBuffer(count, stride, ComputeBufferType.Structured);
            owned.Add(buffer);
            return buffer;
        }

        public void Dispose()
        {
            foreach (var buffer in owned) buffer.Release();
            owned.Clear();
        }
    }
}
