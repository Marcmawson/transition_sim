using System;
using PhysicsSim.ThreePhase3D.Flow;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    public sealed class BeakerVofGpuSession : IDisposable
    {
        public readonly BeakerVofTimeIntegration Simulation;
        private readonly ComputeShader pressure;
        private readonly CutCellGpuScalars scalars;
        private readonly CutCellGpuMomentum momentum;
        private readonly CutCellGpuContactTraction traction;
        public int TractionDispatches => traction?.Dispatches ?? 0;
        public int ContactPatches { get; private set; }
        public int ClippingDispatches => scalars.ClippingDispatches;

        public BeakerVofGpuSession(bool wetting = false)
        {
            pressure = Load("SinglePhaseFlow");
            var scalarShader = Load("CutCellVofScalars");
            var momentumShader = Load("CutCellVofMomentum");
            var clippingShader = Load("CutCellVofClipping");
            Simulation = new BeakerVofTimeIntegration((plan, u, dt, cap) => FlowGpuSolver.Project(plan, pressure, u, dt, cap));
            try
            {
                scalars = new CutCellGpuScalars(Simulation.Plan, scalarShader, clippingShader,
                    wetting ? BeakerVofTimeIntegration.SyntheticWettingProfile() : null);
                if (wetting)
                    traction = new CutCellGpuContactTraction(Simulation.Plan, Load("CutCellContactTraction"));
                momentum = new CutCellGpuMomentum(Simulation.Plan, momentumShader);
            }
            catch { Dispose(); throw; }
        }

        public void Step()
        {
            int contacts = 0;
            Simulation.Step((state, flow) =>
            {
                var step = traction == null
                    ? scalars.AdvanceCoupled(state, flow, BeakerVofTimeIntegration.LiquidDensity,
                        BeakerVofTimeIntegration.GasDensity, momentum, pressure, BeakerVofTimeIntegration.IterationLimit)
                    : scalars.AdvanceCoupledWithWetting(state, flow, momentum, pressure, traction);
                contacts = step.Momentum.ContactTractionPatches;
                return step;
            });
            ContactPatches += contacts;
        }

        private static ComputeShader Load(string name)
        {
            var shader = Resources.Load<ComputeShader>("ThreePhase3D/" + name);
            if (shader == null) throw new InvalidOperationException(name + " compute shader did not import.");
            return shader;
        }

        public void Dispose()
        {
            traction?.Dispose();
            momentum?.Dispose();
            scalars?.Dispose();
        }
    }
}
