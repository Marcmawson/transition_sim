using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Materials;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    public sealed class LiquidEnthalpyGpuRecovery : IDisposable
    {
        private const int Threads = 64;
        private readonly ComputeShader shader;
        private readonly int kernel, dispatch, groups, tableCount;
        private readonly float minimumK, stepK, density;
        private ComputeBuffer enthalpy;
        public ComputeBuffer State { get; private set; }
        public ComputeBuffer Status { get; private set; }

        public LiquidEnthalpyGpuRecovery(ComputeShader shader, WaterPropertyTable table, int groupCount,
            double transportDensityKgM3)
        {
            if (shader == null || !SystemInfo.supportsComputeShaders)
                throw new NotSupportedException("Liquid enthalpy recovery requires compute shader support.");
            if (table == null || table.Branch != WaterBranch.Liquid)
                throw new ArgumentException("A liquid water table is required.", nameof(table));
            density = (float)transportDensityKgM3;
            if (!SiUnits.IsFinite(density) || density <= 0)
                throw new ArgumentOutOfRangeException(nameof(transportDensityKgM3));
            if (groupCount <= 0 || groupCount > 65535 * Threads)
                throw new ArgumentOutOfRangeException(nameof(groupCount));

            this.shader = shader;
            kernel = shader.FindKernel("RecoverLiquidTemperature");
            if (!shader.IsSupported(kernel)) throw new NotSupportedException("Liquid recovery kernel is unsupported.");
            groups = groupCount;
            dispatch = (groups + Threads - 1) / Threads;
            tableCount = table.Count;
            minimumK = (float)table.MinimumK;
            stepK = (float)table.StepK;
            var packed = table.CopyValues();
            var values = new float[tableCount];
            for (int i = 0; i < values.Length; i++)
            {
                values[i] = packed[i * WaterPropertyTable.Width + 1];
                if (i > 0 && values[i] <= values[i - 1])
                    throw new ArgumentException("Liquid enthalpy must increase with temperature.", nameof(table));
            }

            try
            {
                enthalpy = new ComputeBuffer(values.Length, sizeof(float));
                enthalpy.SetData(values);
                State = new ComputeBuffer(groups, 2 * sizeof(float));
                Status = new ComputeBuffer(groups, sizeof(uint));
            }
            catch
            {
                Dispose();
                throw;
            }
        }

        // State stores (kelvin, J/kg). Status: 0 liquid, 1 dry, 2 invalid.
        public void Recover(ComputeBuffer acceptedState)
        {
            if (enthalpy == null) throw new ObjectDisposedException(nameof(LiquidEnthalpyGpuRecovery));
            if (acceptedState == null || acceptedState.stride != 8 || acceptedState.count != 5 * groups + 1)
                throw new ArgumentException("Expected the accepted five-plane cut-cell state and guard.", nameof(acceptedState));

            shader.SetInt("GroupCount", groups);
            shader.SetInt("TableCount", tableCount);
            shader.SetFloat("TransportDensity", density);
            shader.SetFloat("MinimumK", minimumK);
            shader.SetFloat("StepK", stepK);
            shader.SetBuffer(kernel, "UpdatedState", acceptedState);
            shader.SetBuffer(kernel, "SpecificEnthalpy", enthalpy);
            shader.SetBuffer(kernel, "LiquidState", State);
            shader.SetBuffer(kernel, "RecoveryStatus", Status);
            shader.Dispatch(kernel, dispatch, 1, 1);
        }

        public void Dispose()
        {
            enthalpy?.Release();
            State?.Release();
            Status?.Release();
            enthalpy = State = Status = null;
        }
    }
}
