using System;
using System.Linq;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Geometry;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // Fixed geometry and a seeded circulation isolate the S05 advection/
    // pressure loop. Each accepted pressure solution transports the next step.
    public sealed class BeakerVofTimeIntegration
    {
        public const int Segments = 24, IterationLimit = 16384;
        public const double LiquidDensity = 997, GasDensity = 1.2, Dt = .025;
        public readonly CutCellGeometry Geometry;
        public readonly VariableCoefficientProjection Plan;
        public readonly int SliceJ;
        public readonly double InitialLiquidVolumeM3;
        public CutCellInventories State { get; private set; }
        public FlowStep Pressure { get; private set; }
        public int Steps { get; private set; }
        public double TimeS => Steps * Dt;
        public double LiquidDriftM3 => State.LiquidM3.Sum() - InitialLiquidVolumeM3;
        public double MaxDualMassErrorKg { get; private set; }
        public double MaxMomentumLedgerErrorKgMPerS { get; private set; }
        private readonly double[,] componentAreas;

        public static GeometryInputs GeometryInputsForPreview() => new GeometryInputs
        {
            innerRadiusM = .033, wallThicknessM = .031, baseThicknessM = .031, cavityHeightM = .06,
            externalBaseHeightM = .12, tripodRingRadiusM = .049
        };

        public BeakerVofTimeIntegration(PeriodicVofProjection.PressureSolve pressure = null)
        {
            var input = GeometryInputsForPreview();
            var grid = new Grid3D(16,13,16,new D3(.01,.01,.01),
                new RigidFrame(new D3(-.08,input.externalBaseHeightM-.025,-.08),
                    new D3(1,0,0),new D3(0,1,0),new D3(0,0,1)));
            Geometry = CutCellGeometry.Build(grid, ApparatusSurface.Beaker(input, Segments));
            var topology = new VariableCoefficientProjection(Geometry, BoundaryConditions3D.Closed, _ => GasDensity, _ => 0);
            var loop = FindCavityLoop(topology, input, out int slice);
            SliceJ = slice;

            var liquid = new double[topology.GroupCount];
            for (int k = 0; k < grid.Cells.Z; k++)
            for (int i = 0; i < grid.Cells.X; i++)
            {
                var position = grid.Centre(i, SliceJ, k);
                if (position.X >= 0 || position.X * position.X + position.Z * position.Z > input.innerRadiusM * input.innerRadiusM)
                    continue;
                int cell = grid.Cells.Index(i, SliceJ, k), group = topology.GroupByCell[cell];
                if (group >= 0) liquid[group] += .5 * Geometry.FluidVolumeM3[cell];
            }
            // Ensure a mixed donor on the seeded circulation, including cells
            // whose centres lie just outside the cylindrical cavity.
            var first = topology.Faces[loop[0]];
            liquid[first.Low] = .65 * topology.Volumes[first.Low];
            liquid[first.High] = .25 * topology.Volumes[first.High];
            var gas = topology.Volumes.Select((v,c) => v-liquid[c]).ToArray();
            State = new CutCellInventories(liquid,gas,new double[liquid.Length],new double[liquid.Length],new double[liquid.Length]);
            InitialLiquidVolumeM3 = liquid.Sum();

            Plan = new VariableCoefficientProjection(Geometry, BoundaryConditions3D.Closed, cell =>
            {
                int group = topology.GroupByCell[cell];
                return (LiquidDensity*liquid[group]+GasDensity*gas[group])/topology.Volumes[group];
            }, _ => 0);
            componentAreas = new double[3, Plan.GroupCount];
            foreach (var face in Plan.Faces)
            {
                if (face.Active == 0) continue;
                if (face.Low >= 0) componentAreas[face.Axis,face.Low] += face.Area;
                if (face.High >= 0) componentAreas[face.Axis,face.High] += face.Area;
            }
            var u = new float[Plan.FaceCount];
            for (int s = 0; s < loop.Length; s++)
                u[loop[s]] = (float)((s < 2 ? 1 : -1) * 4e-7 / Plan.Faces[loop[s]].Area);
            Pressure = (pressure ?? FlowCpuReference.Project)(Plan,u,Dt,IterationLimit);
        }

        public double LiquidFractionAt(int i, int j, int k)
        {
            int c = Plan.GroupByCell[Geometry.Grid.Cells.Index(i,j,k)];
            return c < 0 ? 0 : State.LiquidM3[c] / Plan.Volumes[c];
        }

        public void Step(Func<CutCellInventories, FlowStep, CutCellCoupledStep> advance)
        {
            if (advance == null) throw new ArgumentNullException(nameof(advance));
            var trial = advance(State,Pressure);
            if (trial == null || !trial.Pressure.Diagnostics.Converged)
                throw new InvalidOperationException("The beaker pressure solve did not converge.");
            if (Math.Abs(trial.Transport.Fields.LiquidM3.Sum()-InitialLiquidVolumeM3) > 1e-12 ||
                trial.Momentum.MaxDualMassErrorKg > 1e-10)
                throw new InvalidOperationException("The beaker transport ledger did not close.");

            var prior = new double[3];
            var transported = new double[3];
            for (int f = 0; f < Plan.FaceCount; f++)
            {
                var face = Plan.Faces[f];
                if (face.Active == 0) continue;
                prior[face.Axis] += (Part(face.Low, face) + Part(face.High, face)) * Pressure.VelocityMPerS[f];
                transported[face.Axis] += trial.Momentum.MomentumKgMPerS[f];
            }
            double error = 0;
            for (int a = 0; a < 3; a++)
                error = Math.Max(error, Math.Abs(transported[a] - prior[a] -
                    trial.Momentum.BoundaryMomentumKgMPerS[a] - trial.Momentum.WallImpulseKgMPerS[a]));
            if (error > 1e-10)
                throw new InvalidOperationException("The beaker momentum remap ledger did not close.");

            State = trial.Transport.Fields;
            Pressure = trial.Pressure;
            Steps++;
            MaxDualMassErrorKg = Math.Max(MaxDualMassErrorKg, trial.Momentum.MaxDualMassErrorKg);
            MaxMomentumLedgerErrorKgMPerS = Math.Max(MaxMomentumLedgerErrorKgMPerS, error);

            double Part(int group, VariableCoefficientProjection.Face face) => group < 0 ? 0 :
                (LiquidDensity * State.LiquidM3[group] + GasDensity * State.GasM3[group]) * face.Area /
                    componentAreas[face.Axis,group];
        }

        private static int[] FindCavityLoop(VariableCoefficientProjection plan, GeometryInputs input, out int slice)
        {
            var grid = plan.Grid;
            int firstJ = (int)Math.Ceiling((input.externalBaseHeightM+input.baseThicknessM-grid.Frame.Origin.Y)/grid.Spacing.Y);
            for (int j = firstJ; j < firstJ+2; j++)
            for (int k = 4; k < 11; k++)
            for (int i = 4; i < 11; i++)
            {
                var cells = new[] { grid.Cells.Index(i,j,k),grid.Cells.Index(i+1,j,k),
                    grid.Cells.Index(i+1,j,k+1),grid.Cells.Index(i,j,k+1) };
                var centre = grid.Centre(i,j,k);
                if (centre.X*centre.X+centre.Z*centre.Z > input.innerRadiusM*input.innerRadiusM ||
                    cells.Any(c => plan.Geometry.FluidVolumeM3[c]<.15*grid.CellVolume) ||
                    cells.All(c => plan.Geometry.FluidVolumeM3[c]>=.999*grid.CellVolume) ||
                    cells.Select(c => plan.GroupByCell[c]).Distinct().Count()!=4) continue;
                var faces = new[] { plan.FaceIndex(0,i+1,j,k),plan.FaceIndex(2,i+1,j,k+1),
                    plan.FaceIndex(0,i+1,j,k+1),plan.FaceIndex(2,i,j,k+1) };
                if (faces.All(f => plan.Faces[f].Active!=0)) { slice = j; return faces; }
            }
            throw new InvalidOperationException("No resolved circulation loop crosses the beaker cavity wall cells.");
        }
    }
}
