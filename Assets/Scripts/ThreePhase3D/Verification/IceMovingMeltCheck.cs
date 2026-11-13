using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Diagnostics;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.PhaseChange;
using PhysicsSim.ThreePhase3D.Solids;
namespace PhysicsSim.ThreePhase3D.Verification
{
    public static class IceMovingMeltCheck
    {
        public static (IceBody Body, double HeatJ, double MassLossKg, double DisplacementM) Run(
            int steps, double substep, Func<VariableCoefficientProjection, float[], double, double[], FlowStep> project,
            int cellsPerAxis = 8, bool transient = false)
        {
            if (project == null) throw new ArgumentNullException(nameof(project));
            if (steps <= 0) throw new ArgumentOutOfRangeException(nameof(steps));
            if (cellsPerAxis <= 0) throw new ArgumentOutOfRangeException(nameof(cellsPerAxis));
            if (!SiUnits.IsFinite(substep) || substep <= 0) throw new ArgumentOutOfRangeException(nameof(substep));
            var equilibrium = new SolidLiquidInterface(101325, 250, 290);
            double rho = equilibrium.LiquidMeltingDensityKgM3, gasDensity = 1.2;
            var gravity = new D3(0, -9.81, 0);
            double spacing = 0.08 / cellsPerAxis;
            var grid = new Grid3D(cellsPerAxis, cellsPerAxis, cellsPerAxis,
                new D3(spacing, spacing, spacing), RigidFrame.Identity);
            var dimensions = new D3(0.018, 0.017, 0.016);
            var body = IceBody.Box(917, dimensions, RigidFrame.Euler(new D3(0.038, 0.041, 0.039), 0.14, 0, 0));
            var initial = body;
            CutCellGeometry Geometry(IceBody pose, D3 size) => CutCellGeometry.Build(grid,
                iceSurface: SurfacePrimitives.Transform(SurfacePrimitives.Box(-size * 0.5, size * 0.5), pose.Frame));
            VariableCoefficientProjection Topology(CutCellGeometry geometry) =>
                new VariableCoefficientProjection(geometry, BoundaryConditions3D.Closed, _ => rho, _ => 0);
            VariableCoefficientProjection WithDensity(VariableCoefficientProjection topology, CutCellInventories inventory) =>
                new VariableCoefficientProjection(topology.Geometry, BoundaryConditions3D.Closed, cell =>
                {
                    int group = topology.GroupByCell[cell];
                    return group < 0 ? gasDensity :
                        (rho * inventory.LiquidM3[group] + gasDensity * inventory.GasM3[group]) / topology.Volumes[group];
                }, _ => 0);
            double Sum(double[] values) => System.Linq.Enumerable.Sum(values);

            var water = new PhysicsSim.ThreePhase3D.Materials.Water95();
            double warmK = equilibrium.MeltingTemperatureK + 10;
            var warm = water.Evaluate(warmK, 101325, PhysicsSim.ThreePhase3D.Materials.WaterBranch.Liquid);
            const double conductivity = 0.6;
            double diffusivity = conductivity / (rho * warm.CpJkgK);
            var topology = Topology(Geometry(body, dimensions));
            var fields = PlanarLiquidInventory.Create(topology, new D3(0, 0.055, 0), new D3(0, 1, 0), warm.EnthalpyJkg * rho, 12, 0);
            var velocity = new float[topology.FaceCount];
            double initialWater = Sum(fields.LiquidM3) * rho + body.MassKg;
            double initialGas = Sum(fields.GasM3) * gasDensity;
            double initialHeat = Sum(fields.LiquidEnergyJ) + body.MassKg * equilibrium.IceMeltingEnthalpyJkg;
            double initialEntropy = initialWater * warm.EntropyJkgK +
                body.MassKg * (equilibrium.IceMeltingEntropyJkgK - warm.EntropyJkgK);
            double totalHeat = 0;
            for (int step = 0; step < steps; step++)
            {
                var plan = WithDensity(topology, fields);
                var column = new ColumnHydrostaticPressure(plan, fields, gravity, rho, gasDensity);
                var tentative = column.Predict(velocity, substep);
                var moved = IcePressureCoupling.Advance(body, plan, substep, gravity,
                    flux => project(plan, tentative, substep, flux), columnPressure: column);
                topology = Topology(Geometry(moved.Body, dimensions));
                var swept = MovingCutCellRemap.Reconcile(plan, topology, moved.Fluid, grid.CellVolume * 0.005);
                var advected = MovingCutCellRemap.ApplyWithMomentum(plan, topology, fields, swept,
                    moved.Fluid.VelocityMPerS, rho, gasDensity);
                plan = WithDensity(topology, advected.Fields);
                column = new ColumnHydrostaticPressure(plan, advected.Fields, gravity, rho, gasDensity);
                var temperature = transient ? null : new double[plan.GroupCount];
                for (int group = 0; group < (temperature?.Length ?? 0); group++)
                {
                    double mass = advected.Fields.LiquidM3[group] * rho;
                    temperature[group] = mass > 0
                        ? equilibrium.Recover(mass, advected.Fields.LiquidEnergyJ[group]).TemperatureK
                        : equilibrium.MeltingTemperatureK;
                }
                var projected = project(plan,
                    Array.ConvertAll(advected.Momentum.VelocityMPerS, value => (float)value), substep, null);
                IceFaceConduction.TransientLayer? layer = transient
                    ? new IceFaceConduction.TransientLayer(step * substep, warmK, diffusivity) : null;
                var melted = IceFaceMeltingStep.Advance(plan, advected.Fields, projected, moved.Body, dimensions,
                    1, -1, equilibrium, gasDensity, conductivity, temperature, column.WettedAreaM2,
                    Topology, grid.CellVolume * 0.005, layer);
                if (!(melted.HeatJ > 0 && melted.Body.MassKg < body.MassKg))
                    throw new InvalidOperationException("The moving ice did not melt on the submerged face.");
                totalHeat += melted.HeatJ;
                (body, dimensions, topology, fields, gasDensity) =
                    (melted.Body, melted.DimensionsM, melted.Topology, melted.Fields, melted.GasDensityKgM3);
                velocity = Array.ConvertAll(melted.Momentum.VelocityMPerS, value => (float)value);
                Near(Math.Abs(Sum(fields.LiquidM3) * rho + body.MassKg - initialWater), 1e-13);
                Near(Math.Abs(Sum(fields.GasM3) * gasDensity - initialGas), 1e-13);
                Near(Math.Abs(Sum(fields.LiquidEnergyJ) + body.MassKg * equilibrium.IceMeltingEnthalpyJkg - initialHeat), 1e-8);
            }
            if ((body.Frame.Origin - initial.Frame.Origin).Length <= 1e-8)
                throw new InvalidOperationException("The melting body did not move.");
            var finalEntropy = new StableReduction.Accumulator();
            finalEntropy.Add(body.MassKg * equilibrium.IceMeltingEntropyJkgK);
            for (int group = 0; group < fields.LiquidM3.Length; group++)
            {
                double mass = fields.LiquidM3[group] * rho;
                if (mass > 0)
                    finalEntropy.Add(equilibrium.Recover(mass, fields.LiquidEnergyJ[group]).EntropyJPerK);
            }
            // In this closed thermal fixture only the ice and liquid exchange heat.
            if (!(finalEntropy.Value > initialEntropy))
                throw new InvalidOperationException("Moving melt did not produce positive water entropy.");
            if (transient)
            {
                double expected = 2 * conductivity * 10 * dimensions.X * dimensions.Z *
                    Math.Sqrt(steps * substep / (Math.PI * diffusivity));
                Near(Math.Abs(totalHeat / expected - 1), 0.01);
            }
            return (body, totalHeat, initial.MassKg - body.MassKg,
                (body.Frame.Origin - initial.Frame.Origin).Length);
        }

        private static void Near(double error, double tolerance)
        {
            if (!(error <= tolerance))
                throw new InvalidOperationException($"Moving melt error {error:G8} exceeds {tolerance:G8}.");
        }
    }
}
