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
        public int ClippingDispatches => scalars.ClippingDispatches;

        public BeakerVofGpuSession()
        {
            pressure = Load("SinglePhaseFlow");
            var scalarShader = Load("CutCellVofScalars");
            var momentumShader = Load("CutCellVofMomentum");
            var clippingShader = Load("CutCellVofClipping");
            Simulation = new BeakerVofTimeIntegration((plan, u, dt, cap) => FlowGpuSolver.Project(plan, pressure, u, dt, cap));
            try
            {
                scalars = new CutCellGpuScalars(Simulation.Plan, scalarShader, clippingShader);
                momentum = new CutCellGpuMomentum(Simulation.Plan, momentumShader);
            }
            catch { Dispose(); throw; }
        }

        public void Step() => Simulation.Step((state, flow) => scalars.AdvanceCoupled(state, flow,
            BeakerVofTimeIntegration.LiquidDensity, BeakerVofTimeIntegration.GasDensity, momentum, pressure,
            BeakerVofTimeIntegration.IterationLimit));

        private static ComputeShader Load(string name)
        {
            var shader = Resources.Load<ComputeShader>("ThreePhase3D/" + name);
            if (shader == null) throw new InvalidOperationException(name + " compute shader did not import.");
            return shader;
        }

        public void Dispose()
        {
            momentum?.Dispose();
            scalars?.Dispose();
        }
    }
}
