using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    public sealed class BeakerVofGpuSession : IDisposable
    {
        public readonly BeakerVofTimeIntegration Simulation;
        private readonly ComputeShader pressure;
        private readonly D3 acceleration;
        private readonly CutCellGpuScalars scalars;
        private readonly CutCellGpuMomentum momentum;
        private readonly CutCellGpuContactTraction traction;
        public int TractionDispatches => traction?.Dispatches ?? 0;
        public int ContactPatches { get; private set; }
        public int ClippingDispatches => scalars.ClippingDispatches;

        public BeakerVofGpuSession(bool wetting = false)
            : this(wetting, null, default) { }

        public BeakerVofGpuSession(BeakerVofTimeIntegration release)
            : this(false, release ?? throw new ArgumentNullException(nameof(release)), new D3(0, -9.81, 0)) { }

        private BeakerVofGpuSession(bool wetting, BeakerVofTimeIntegration release, D3 acceleration)
        {
            this.acceleration = acceleration;
            pressure = Load("SinglePhaseFlow");
            var scalarShader = Load("CutCellVofScalars");
            var momentumShader = Load("CutCellVofMomentum");
            var clippingShader = Load("CutCellVofClipping");
            Simulation = release ?? new BeakerVofTimeIntegration((plan, u, dt, cap) => FlowGpuSolver.Project(plan, pressure, u, dt, cap));
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

        public void Step() => StepCore(acceleration,
            acceleration.Length > 0 ? BeakerVofTimeIntegration.ReleaseMaxStepS : (double?)null,
            BeakerVofTimeIntegration.IterationLimit);

        internal void StepRelease(double gravityY, double maximumStepS, int pressureIterations, double burnerWatts = 0) =>
            StepCore(new D3(0, gravityY, 0), maximumStepS, pressureIterations, burnerWatts);

        private void StepCore(D3 gravity, double? maximumStepS, int pressureIterations, double burnerWatts = 0)
        {
            int contacts = 0;
            Simulation.Step((state, flow) =>
            {
                var step = traction == null
                    ? scalars.AdvanceCoupled(state, flow, Simulation.LiquidDensityKgM3,
                        BeakerVofTimeIntegration.GasDensity, momentum, pressure, pressureIterations, gravity, maximumStepS)
                    : scalars.AdvanceCoupledWithWetting(state, flow, momentum, pressure, traction);
                contacts = step.Momentum.ContactTractionPatches;
                return step;
            }, burnerWatts);
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
