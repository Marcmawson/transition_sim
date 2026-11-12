using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.Solids;

namespace PhysicsSim.ThreePhase3D.Verification
{
    public static class IceTwoPhaseCheck
    {
        public static (double FirstSpeed, double SecondSpeed, double PressureChange) Run(
            Func<VariableCoefficientProjection, float[], double, double[], FlowStep> project)
        {
            if (project == null) throw new ArgumentNullException(nameof(project));
            const double rhoWater = 1000, rhoAir = 1.2, g = 9.81, dt = 0.002;
            var grid = new Grid3D(8, 8, 8, new D3(0.01, 0.01, 0.01));
            var centre = new D3(0.038, 0.041, 0.039);
            var waterline = new D3(0, centre.Y, 0);
            var size = new D3(0.018, 0.017, 0.016);
            var box = SurfacePrimitives.Box(size * -0.5, size * 0.5);
            var body = IceBody.Box(917, size, RigidFrame.Euler(centre, 0.14, 0, 0));
            CutCellGeometry Geometry(IceBody pose) => CutCellGeometry.Build(grid,
                iceSurface: SurfacePrimitives.Transform(box, pose.Frame));
            var geometry = Geometry(body);
            var boundaries = BoundaryConditions3D.Closed;
            var topology = new VariableCoefficientProjection(geometry, boundaries, _ => rhoWater, _ => 0);
            var inventory = PlanarLiquidInventory.Create(topology, waterline, new D3(0, 1, 0), 42, 12, 0.01);
            double water = System.Linq.Enumerable.Sum(inventory.LiquidM3);
            Near(Math.Abs(water - (0.08 * 0.08 * centre.Y - 0.5 * size.X * size.Y * size.Z)), 1e-15);
            VariableCoefficientProjection WithPhaseDensity(VariableCoefficientProjection groups, CutCellInventories fields) =>
                new VariableCoefficientProjection(groups.Geometry, boundaries, cell =>
            {
                int group = groups.GroupByCell[cell];
                return group < 0 ? rhoAir :
                    (rhoWater * fields.LiquidM3[group] + rhoAir * fields.GasM3[group]) / groups.Volumes[group];
            }, _ => 0);
            var plan = WithPhaseDensity(topology, inventory);
            var gravity = new D3(0, -g, 0);
            var initialColumn = new ColumnHydrostaticPressure(plan, inventory, gravity, rhoWater, rhoAir);
            int sampleCell = grid.Cells.Index(0, 1, 0);
            var sample = grid.Centre(0, 1, 0);
            double expectedPressure = g * (rhoWater * (waterline.Y - sample.Y) + rhoAir * (0.08 - waterline.Y));
            Near(Math.Abs(initialColumn.At(sampleCell, sample) - expectedPressure), 1e-8);
            var still = new float[plan.FaceCount];
            var moved = IcePressureCoupling.Advance(body, plan, dt, new D3(0, -g, 0),
                flux => project(plan, still, dt, flux),
                point => rhoWater * g * Math.Max(0, waterline.Y - point.Y));
            var updatedGeometry = Geometry(moved.Body);
            var updatedPlan = new VariableCoefficientProjection(updatedGeometry, boundaries, _ => rhoWater, _ => 0);
            var swept = MovingCutCellRemap.Reconcile(plan, updatedPlan, moved.Fluid, 0.005 * grid.CellVolume);
            var updated = MovingCutCellRemap.ApplyWithMomentum(plan, updatedPlan, inventory, swept,
                moved.Fluid.VelocityMPerS, rhoWater, rhoAir);
            for (int field = 0; field < inventory.Planes.Length; field++)
                Near(Math.Abs(System.Linq.Enumerable.Sum(updated.Fields.Planes[field]) -
                    System.Linq.Enumerable.Sum(inventory.Planes[field])), 1e-13);
            var staticSurface = PlanarLiquidInventory.Create(updatedPlan, waterline, new D3(0, 1, 0), 42, 12, 0.01);
            double redistribution = 0;
            for (int group = 0; group < updatedPlan.GroupCount; group++)
                redistribution += Math.Abs(updated.Fields.LiquidM3[group] - staticSurface.LiquidM3[group]);
            if (moved.Body.VelocityMPerS.Y >= 0 || redistribution <= 1e-12)
                throw new InvalidOperationException("Two-phase ice step failed to sink or redistribute the water inventory.");
            var secondPlan = WithPhaseDensity(updatedPlan, updated.Fields);
            var column = new ColumnHydrostaticPressure(secondPlan, updated.Fields, gravity, rhoWater, rhoAir);
            double pressureChange = 0;
            for (int z = 0; z < grid.Cells.Z; z++)
            for (int x = 0; x < grid.Cells.X; x++)
            {
                int cell = grid.Cells.Index(x, 0, z);
                var point = grid.Centre(x, 0, z);
                pressureChange = Math.Max(pressureChange, Math.Abs(column.At(cell, point) - initialColumn.At(cell, point)));
            }
            var tentative = column.Predict(Array.ConvertAll(updated.Momentum.VelocityMPerS, value => (float)value), dt);
            var second = IcePressureCoupling.Advance(moved.Body, secondPlan, dt, gravity,
                flux => project(secondPlan, tentative, dt, flux),
                columnPressure: column);
            if (pressureChange <= 1e-5 || second.Body.Frame.Origin.Y >= moved.Body.Frame.Origin.Y)
                throw new InvalidOperationException("Transported liquid did not update hydrostatic loading and ice motion.");
            var finalTopology = new VariableCoefficientProjection(Geometry(second.Body), boundaries, _ => rhoWater, _ => 0);
            var secondSweep = MovingCutCellRemap.Reconcile(secondPlan, finalTopology, second.Fluid, 0.005 * grid.CellVolume);
            var final = MovingCutCellRemap.ApplyWithMomentum(secondPlan, finalTopology, updated.Fields,
                secondSweep, second.Fluid.VelocityMPerS, rhoWater, rhoAir);
            for (int field = 0; field < inventory.Planes.Length; field++)
                Near(Math.Abs(System.Linq.Enumerable.Sum(final.Fields.Planes[field]) -
                    System.Linq.Enumerable.Sum(inventory.Planes[field])), 1e-13);
            return (moved.Body.VelocityMPerS.Y, second.Body.VelocityMPerS.Y, pressureChange);
        }

        private static void Near(double error, double tolerance)
        {
            if (!SiUnits.IsFinite(error) || error > tolerance)
                throw new InvalidOperationException($"Two-phase ice verification error {error:G8} exceeded {tolerance:G8}.");
        }
    }
}
