using System;
using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.PhaseChange;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    public sealed class CutCellGpuScalars : IDisposable
    {
        private const int Threads = 64;
        private readonly VariableCoefficientProjection plan;
        private readonly CutCellVofTransport transport;
        private readonly ComputeShader shader;
        private readonly CutCellGpuClipping clipping;
        public int ClippingDispatches => clipping?.Dispatches ?? 0;
        private readonly int budget, build, apply, groupDispatch, faceDispatch;
        private readonly List<ComputeBuffer> owned = new List<ComputeBuffer>();
        private readonly ComputeBuffer endpoints, rows, incidence, groups, candidates, transfers, updated, donorBudget;
        private readonly VofFloatPair[] groupData, candidateData, transferData, updatedData;

        public CutCellGpuScalars(VariableCoefficientProjection plan, ComputeShader shader, ComputeShader clippingShader = null, WettingProfile wetting = null)
        {
            this.plan = plan ?? throw new ArgumentNullException(nameof(plan));
            if (shader == null || !SystemInfo.supportsComputeShaders)
                throw new NotSupportedException("Cut-cell scalar compute shader is unavailable.");
            this.shader = shader;
            if (clippingShader != null) clipping = new CutCellGpuClipping(plan.Grid, clippingShader);
            budget = shader.FindKernel("BuildCutDonorBudget");
            build = shader.FindKernel("BuildCutFaceFlux");
            apply = shader.FindKernel("ApplyCutScalarFlux");
            if (!shader.IsSupported(budget) || !shader.IsSupported(build) || !shader.IsSupported(apply))
                throw new NotSupportedException("Cut-cell scalar kernels are unsupported on " + SystemInfo.graphicsDeviceType);
            int ng = plan.GroupCount, nf = plan.FaceCount;
            groupDispatch = DispatchCount(ng);
            faceDispatch = DispatchCount(nf);
            long bytes = 8L * ((CutCellScalarGpuLayout.GroupPlanes + CutCellScalarGpuLayout.StatePlanes + 2) * (long)ng +
                (CutCellScalarGpuLayout.CandidatePlanes + CutCellScalarGpuLayout.TransferPlanes + 1) * (long)nf + 2) +
                4L * (plan.Rows.Length + (long)Math.Max(1, plan.Incidence.Length));
            if (bytes > 512L * 1024 * 1024)
                throw new NotSupportedException("Cut-cell scalar GPU buffers exceed the 512 MiB payload budget.");
            transport = wetting == null ? new CutCellVofTransport(plan) : CutCellVofTransport.WithTrackedWetting(plan, wetting);
            groupData = new VofFloatPair[CutCellScalarGpuLayout.GroupPlanes * ng];
            candidateData = new VofFloatPair[CutCellScalarGpuLayout.CandidatePlanes * nf];
            transferData = new VofFloatPair[CutCellScalarGpuLayout.TransferPlanes * nf + 1];
            updatedData = new VofFloatPair[CutCellScalarGpuLayout.StatePlanes * ng + 1];
            try
            {
                endpoints = Buffer(nf, 8);
                var ends = new Vector2Int[nf];
                for (int f = 0; f < nf; f++) ends[f] = new Vector2Int(plan.Faces[f].Low, plan.Faces[f].High);
                endpoints.SetData(ends);
                rows = Buffer(plan.Rows.Length, 4);
                rows.SetData(plan.Rows);
                incidence = Buffer(Math.Max(1, plan.Incidence.Length), 4);
                if (plan.Incidence.Length > 0) incidence.SetData(plan.Incidence);
                groups = Buffer(groupData.Length, 8);
                candidates = Buffer(candidateData.Length, 8);
                donorBudget = Buffer(2 * ng, 8);
                transfers = Guarded(transferData.Length);
                updated = Guarded(updatedData.Length);
            }
            catch { Dispose(); throw; }
        }

        public CutCellTransportResult Advance(CutCellInventories state, double[] velocity, double dt,
            Func<int, CutCellInflow> inflow = null)
        {
            if (owned.Count == 0) throw new ObjectDisposedException(nameof(CutCellGpuScalars));
            CutCellGeometryBatch batch = clipping == null ? (CutCellGeometryBatch)null : clipping.Clip;
            return transport.Advance(state, velocity, dt, inflow, 0, Dispatch, batch);
        }

        public CutCellCoupledStep AdvanceCoupled(CutCellInventories state, FlowStep flow,
            double liquidDensity, double gasDensity, CutCellGpuMomentum momentum, ComputeShader pressureShader, int maxIterations)
        {
            if (owned.Count == 0) throw new ObjectDisposedException(nameof(CutCellGpuScalars));
            if (momentum == null || pressureShader == null || clipping == null)
                throw new ArgumentException("The coupled preview requires GPU momentum, pressure and clipping.");
            return transport.AdvanceCoupled(state, flow, liquidDensity, gasDensity, null, maxIterations,
                (next, velocity, dt, cap) => FlowGpuSolver.Project(next, pressureShader, velocity, dt, cap),
                Dispatch, clipping.Clip, momentum.Advance);
        }

        internal CutCellCoupledStep AdvanceCoupledWithWetting(CutCellInventories state, FlowStep flow,
            CutCellGpuMomentum momentum, ComputeShader pressureShader, CutCellGpuContactTraction traction)
        {
            return transport.AdvanceCoupledWithWetting(state, flow, BeakerVofTimeIntegration.LiquidDensity,
                BeakerVofTimeIntegration.GasDensity, BeakerVofTimeIntegration.LiquidViscosityPaS,
                BeakerVofTimeIntegration.GasViscosityPaS, null, BeakerVofTimeIntegration.IterationLimit,
                (next, velocity, dt, cap) => FlowGpuSolver.Project(next, pressureShader, velocity, dt, cap),
                Dispatch, clipping.Clip, momentum.Advance, traction.Apply);
        }

        private CutCellScalarOutput Dispatch(CutCellScalarWork work)
        {
            CutCellScalarGpuLayout.Pack(work, groupData, candidateData);
            groups.SetData(groupData);
            candidates.SetData(candidateData);
            shader.SetInt("GroupCount", plan.GroupCount);
            shader.SetInt("FaceCount", plan.FaceCount);
            shader.SetBuffer(budget, "Rows", rows);
            shader.SetBuffer(budget, "Incidence", incidence);
            shader.SetBuffer(budget, "Candidates", candidates);
            shader.SetBuffer(budget, "DonorBudget", donorBudget);
            shader.Dispatch(budget, groupDispatch, 1, 1);
            shader.SetBuffer(build, "DonorBudget", donorBudget);
            shader.SetBuffer(build, "Endpoints", endpoints);
            shader.SetBuffer(build, "Groups", groups);
            shader.SetBuffer(build, "Candidates", candidates);
            shader.SetBuffer(build, "Transfers", transfers);
            shader.Dispatch(build, faceDispatch, 1, 1);

            shader.SetBuffer(apply, "Groups", groups);
            shader.SetBuffer(apply, "Rows", rows);
            shader.SetBuffer(apply, "Incidence", incidence);
            shader.SetBuffer(apply, "Transfers", transfers);
            shader.SetBuffer(apply, "Updated", updated);
            shader.Dispatch(apply, groupDispatch, 1, 1);

            transfers.GetData(transferData);
            updated.GetData(updatedData);
            return CutCellScalarGpuLayout.Decode(work, transferData, updatedData);
        }

        private static int DispatchCount(int count)
        {
            int dispatch = (int)((count + (long)Threads - 1) / Threads);
            if (dispatch > 65535) throw new NotSupportedException("Cut-cell dispatch exceeds one-dimensional limits.");
            return dispatch;
        }

        private ComputeBuffer Buffer(int count, int stride)
        {
            var buffer = new ComputeBuffer(count, stride, ComputeBufferType.Structured);
            owned.Add(buffer);
            return buffer;
        }

        private ComputeBuffer Guarded(int count)
        {
            var buffer = Buffer(count, 8);
            buffer.SetData(new[] { new VofFloatPair(VofGpuTransportLayout.Guard) }, 0, count - 1, 1);
            return buffer;
        }

        public void Dispose()
        {
            foreach (var buffer in owned) buffer.Release();
            owned.Clear();
        }
    }
}
