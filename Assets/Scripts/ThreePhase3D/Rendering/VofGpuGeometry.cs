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
        private readonly Vector4[] courantData;
        private readonly VofGpuUpdates updates;
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
            try
            {
                alpha = new ComputeBuffer(count, 4);
                courant = new ComputeBuffer(count, 16);
                swept = new ComputeBuffer(count, 16);
                updates = new VofGpuUpdates(vof, Resources.Load<ComputeShader>("ThreePhase3D/VofConservativeUpdates"));
            }
            catch { Dispose(); throw; }
        }

        public VofAdvance Advance(VofFields state, double[][] velocity, double dt)
        {
            if (swept == null) throw new ObjectDisposedException(nameof(VofGpuGeometry));
            var total = vof.PrepareCourant(state, velocity, dt, out var outgoing);
            DispatchCandidates(state.LiquidFraction, total);
            var result = updates.Advance(state, total, outgoing, swept, out double candidateRoundoff);
            CandidateRoundoffM3 += candidateRoundoff;
            return result;
        }

        private void DispatchCandidates(double[] fraction, double[][] total)
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
        }

        public void Dispose()
        {
            updates?.Dispose();
            alpha?.Release(); alpha = null;
            courant?.Release(); courant = null;
            swept?.Release(); swept = null;
        }
    }
}
