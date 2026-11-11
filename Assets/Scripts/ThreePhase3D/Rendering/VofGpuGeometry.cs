using System;
using PhysicsSim.ThreePhase3D.Flow;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    public sealed class VofGpuGeometry : IDisposable
    {
        private readonly GeometricVof3D vof;
        private readonly ComputeShader shader;
        private readonly int sweep, groups;
        private readonly float[] alphaData;
        private readonly Vector4[] courantData, sweepData;
        private ComputeBuffer alpha, courant, swept;
        public double CandidateRoundoffM3 { get; private set; }

        public VofGpuGeometry(GeometricVof3D vof, ComputeShader shader)
        {
            this.vof = vof ?? throw new ArgumentNullException(nameof(vof));
            if (shader == null || !SystemInfo.supportsComputeShaders)
                throw new NotSupportedException("S05 GPU geometry requires an imported compute shader and compute support.");
            this.shader = shader;
            int count = vof.CellCount;
            groups = (int)((count + 63L) / 64);
            if (groups > 65535) throw new NotSupportedException("S05 GPU dispatch exceeds one-dimensional limits.");
            sweep = shader.FindKernel("SweepPlic");
            if (!shader.IsSupported(sweep))
                throw new NotSupportedException("VofGeometry.compute SweepPlic is unsupported on " +
                    SystemInfo.graphicsDeviceType + ". Check the Unity shader import messages.");
            alphaData = new float[count];
            courantData = new Vector4[count];
            sweepData = new Vector4[count + 1];
            try
            {
                alpha = new ComputeBuffer(count, 4);
                courant = new ComputeBuffer(count, 16);
                swept = new ComputeBuffer(count + 1, 16);
                swept.SetData(new[] { new Vector4(-1, -1, -1, -1) }, 0, count, 1);
            }
            catch { Dispose(); throw; }
        }

        public VofAdvance Advance(VofFields state, double[][] velocity, double dt)
        {
            if (swept == null) throw new ObjectDisposedException(nameof(VofGpuGeometry));
            return vof.Advance(state, velocity, dt, Candidates);
        }

        private double[][] Candidates(double[] fraction, double[][] total)
        {
            int count = vof.CellCount;
            for (int c = 0; c < count; c++)
            {
                alphaData[c] = (float)fraction[c];
                courantData[c] = new Vector4((float)total[0][c], (float)total[1][c], (float)total[2][c], 0);
            }
            alpha.SetData(alphaData);
            courant.SetData(courantData);
            shader.SetInt("Nx", vof.Grid.Cells.X);
            shader.SetInt("Ny", vof.Grid.Cells.Y);
            shader.SetInt("Nz", vof.Grid.Cells.Z);
            shader.SetInt("Count", count);
            shader.SetBuffer(sweep, "Alpha", alpha);
            shader.SetBuffer(sweep, "Courant", courant);
            shader.SetBuffer(sweep, "SweepOut", swept);
            shader.Dispatch(sweep, groups, 1, 1);
            swept.GetData(sweepData);
            if (sweepData[count] != new Vector4(-1, -1, -1, -1))
                throw new InvalidOperationException("S05 swept-volume buffer guard was overwritten.");
            var output = new[] { new double[count], new double[count], new double[count] };
            for (int a = 0; a < 3; a++)
            for (int c = 0; c < count; c++)
            {
                float ratio = sweepData[c][a];
                if (float.IsNaN(ratio) || ratio < -2e-6f || ratio > 1 + 2e-6f)
                    throw new InvalidOperationException("GPU PLIC sweep returned an invalid fraction.");
                double bounded = Math.Min(1, Math.Max(0, ratio));
                double width = Math.Abs(total[a][c]);
                CandidateRoundoffM3 += Math.Abs(bounded - ratio) * width * vof.Grid.CellVolume;
                output[a][c] = bounded * width;
            }
            return output;
        }

        public void Dispose()
        {
            alpha?.Release(); alpha = null;
            courant?.Release(); courant = null;
            swept?.Release(); swept = null;
        }
    }
}
