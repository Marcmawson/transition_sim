using System;
using System.Linq;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.Materials;
using PhysicsSim.ThreePhase3D.PhaseChange;
using PhysicsSim.ThreePhase3D.Thermal;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // The circulation and water-release fixtures share a fixed beaker geometry.
    // Each accepted pressure solution supplies the next transport step.
    public sealed class BeakerVofTimeIntegration
    {
        public const int Segments = 24, IterationLimit = 16384;
        public const double LiquidDensity = 997, GasDensity = 1.2, Dt = .025;
        public const double ReleaseMaxStepS = .001;
        public const double LiquidViscosityPaS = .001, GasViscosityPaS = .000018;
        public static WettingProfile SyntheticWettingProfile() =>
            new WettingProfile("Synthetic beaker wall", "S06 numerical fixture", 65, .001, .01,
                new[] { new WettingMeasurement(0, 80), new WettingMeasurement(.2, 100) },
                new[] { new WettingMeasurement(0, 50), new WettingMeasurement(.2, 30) });

        public readonly CutCellGeometry Geometry;
        public readonly VariableCoefficientProjection Plan;
        public readonly int SliceJ;
        public readonly double InitialLiquidVolumeM3;
        public CutCellInventories State { get; private set; }
        public FlowStep Pressure { get; private set; }
        public int Steps { get; private set; }
        public readonly double LiquidDensityKgM3;
        public double InitialLiquidMassKg => InitialLiquidVolumeM3 * LiquidDensityKgM3;
        public double TimeS { get; private set; }
        public double InitialLiquidEnthalpyJ { get; }
        public double LiquidEnthalpyDriftJ => State.LiquidEnergyJ.Sum() - InitialLiquidEnthalpyJ - LiquidHeatJ;
        public double LiquidHeatJ { get; private set; }
        public double ThermalResidualJ { get; private set; }
        private BeakerSolidMesh thermalMesh;
        private SolidConduction glass;
        private double[] glassPower;
        private double thermalReferenceK, burnerEnergyJ, floorLiquidDistanceM, initialGlassTemperatureK;
        private (int Group, int Glass, double Area)[] floorContacts;
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
            : this(pressure, null, LiquidDensity, 0, 0) { }

        public static BeakerVofTimeIntegration CreateRelease(double amount, bool specifiedByMass,
            double temperatureK, double pressurePa, double surfaceJitterM = 0)
        {
            if (!SiUnits.IsFinite(amount) || amount <= 0)
                throw new ArgumentOutOfRangeException(nameof(amount));
            if (!SiUnits.IsFinite(surfaceJitterM) || surfaceJitterM < 0)
                throw new ArgumentOutOfRangeException(nameof(surfaceJitterM));
            var water = new Water95().Evaluate(temperatureK, pressurePa, WaterBranch.Liquid);
            double density = water.DensityKgM3;
            return new BeakerVofTimeIntegration(null, specifiedByMass ? amount / density : amount, density, surfaceJitterM, water.EnthalpyJkg);
        }

        private BeakerVofTimeIntegration(PeriodicVofProjection.PressureSolve pressure, double? releaseVolumeM3,
            double liquidDensityKgM3, double surfaceJitterM, double liquidEnthalpyJkg)
        {
            LiquidDensityKgM3 = liquidDensityKgM3;
            var input = GeometryInputsForPreview();
            var grid = new Grid3D(16,13,16,new D3(.01,.01,.01),
                new RigidFrame(new D3(-.08,input.externalBaseHeightM-.025,-.08),
                    new D3(1,0,0),new D3(0,1,0),new D3(0,0,1)));
            Geometry = CutCellGeometry.Build(grid, ApparatusSurface.Beaker(input, Segments));
            var topology = new VariableCoefficientProjection(Geometry, BoundaryConditions3D.Closed, _ => GasDensity, _ => 0);
            int[] loop = null;
            double[] liquid;
            if (releaseVolumeM3.HasValue)
            {
                SliceJ = 6;
                liquid = CreateReleaseVolumes(topology, input, releaseVolumeM3.Value, surfaceJitterM);
            }
            else
            {
                loop = FindCavityLoop(topology, input, out int slice);
                SliceJ = slice;
                liquid = CreateCirculationVolumes(topology, input, loop, slice);
            }
            var gas = topology.Volumes.Select((v,c) => v-liquid[c]).ToArray();
            // Advect the released liquid's thermal inventory with its volume.
            var liquidEnthalpy = liquid.Select(volume => volume * liquidDensityKgM3 * liquidEnthalpyJkg).ToArray();
            InitialLiquidEnthalpyJ = liquidEnthalpy.Sum();
            State = new CutCellInventories(liquid, gas, liquidEnthalpy, new double[liquid.Length], new double[liquid.Length]);
            InitialLiquidVolumeM3 = liquid.Sum();

            Plan = new VariableCoefficientProjection(Geometry, BoundaryConditions3D.Closed, cell =>
            {
                int group = topology.GroupByCell[cell];
                return (LiquidDensityKgM3*liquid[group]+GasDensity*gas[group])/topology.Volumes[group];
            }, _ => 0);
            componentAreas = new double[3, Plan.GroupCount];
            foreach (var face in Plan.Faces)
            {
                if (face.Active == 0) continue;
                if (face.Low >= 0) componentAreas[face.Axis,face.Low] += face.Area;
                if (face.High >= 0) componentAreas[face.Axis,face.High] += face.Area;
            }
            var u = new float[Plan.FaceCount];
            for (int s = 0; loop != null && s < loop.Length; s++)
                u[loop[s]] = (float)((s < 2 ? 1 : -1) * 4e-7 / Plan.Faces[loop[s]].Area);
            Pressure = (pressure ?? FlowCpuReference.Project)(Plan,u,releaseVolumeM3.HasValue ? ReleaseMaxStepS : Dt,IterationLimit);
        }

        public double LiquidFractionAt(int i, int j, int k)
        {
            int c = Plan.GroupByCell[Geometry.Grid.Cells.Index(i,j,k)];
            return c < 0 ? 0 : State.LiquidM3[c] / Plan.Volumes[c];
        }

        public void EnableFloorHeating(double initialTemperatureK, double? glassTemperatureK = null)
        {
            if (Steps != 0 || glass != null) throw new InvalidOperationException("Enable heating before the first step.");
            var input = GeometryInputsForPreview();
            thermalReferenceK = initialTemperatureK;
            initialGlassTemperatureK = glassTemperatureK ?? initialTemperatureK;
            thermalMesh = new BeakerSolidMesh(input);
            glass = new SolidConduction(thermalMesh.CapacityJPerK,
                Enumerable.Repeat(initialGlassTemperatureK, thermalMesh.CapacityJPerK.Length).ToArray(), thermalMesh.Faces);
            glassPower = new double[thermalMesh.CapacityJPerK.Length];
            var grid = Geometry.Grid;
            double floor = input.externalBaseHeightM + input.baseThicknessM;
            int j = (int)Math.Ceiling((floor - grid.Frame.Origin.Y) / grid.Spacing.Y);
            floorLiquidDistanceM = grid.Centre(0, j, 0).Y - floor;
            var contacts = new System.Collections.Generic.List<(int, int, double)>();
            for (int k = 0; k < grid.Cells.Z; k++)
            for (int i = 0; i < grid.Cells.X; i++)
            {
                var centre = grid.Centre(i, j, k);
                double radius = Math.Sqrt(centre.X * centre.X + centre.Z * centre.Z);
                int group = Plan.GroupByCell[grid.Cells.Index(i, j, k)];
                if (group < 0 || radius >= input.innerRadiusM) continue;
                int ring = (int)(radius / input.innerRadiusM * BeakerSolidMesh.InnerRadialCells);
                int surface = (ring + 1) * BeakerSolidMesh.BaseDepthCells - 1;
                contacts.Add((group, surface, grid.Spacing.X * grid.Spacing.Z));
            }
            floorContacts = contacts.ToArray();
        }

        private void ExchangeFloorHeat(CutCellInventories fields, double dt, double watts)
        {
            const double liquidCp = 4180, liquidConductivity = .6, glassConductivity = 1.2;
            var input = GeometryInputsForPreview();
            double referenceEnthalpy = InitialLiquidEnthalpyJ / InitialLiquidMassKg;
            for (int i = 0; i < glassPower.Length; i++) glassPower[i] = watts * thermalMesh.BurnerWeights[i];
            double heat = 0;
            // Cell-centre floor contact with a volume-fraction estimate of wet area.
            foreach (var contact in floorContacts)
            {
                int group = contact.Group;
                double volume = fields.LiquidM3[group];
                if (volume <= 0) continue;
                double capacity = volume * LiquidDensityKgM3 * liquidCp;
                double liquidK = thermalReferenceK +
                    (fields.LiquidEnergyJ[group] - volume * LiquidDensityKgM3 * referenceEnthalpy) / capacity;
                double glassK = glass.TemperatureK[contact.Glass];
                double rate = CoupledHeatFaces.HeatRateW(contact.Area * volume / Plan.Volumes[group],
                    glassK, liquidK, input.baseThicknessM / (2 * BeakerSolidMesh.BaseDepthCells),
                    glassConductivity, 0, floorLiquidDistanceM, liquidConductivity);
                double equilibriumHeat = (glassK - liquidK) /
                    (1 / capacity + 1 / thermalMesh.CapacityJPerK[contact.Glass]);
                double transfer = Math.Sign(rate) * Math.Min(Math.Abs(rate * dt), Math.Abs(equilibriumHeat));
                glassPower[contact.Glass] -= transfer / dt;
                fields.LiquidEnergyJ[group] += transfer;
                heat += transfer;
            }
            glass.Step(dt, glassPower);
            LiquidHeatJ += heat;
            burnerEnergyJ += watts * thermalMesh.BurnerWeights.Sum() * dt;
            double glassStored = glass.TemperatureK.Select((temperature, i) =>
                (temperature - initialGlassTemperatureK) * thermalMesh.CapacityJPerK[i]).Sum();
            ThermalResidualJ = fields.LiquidEnergyJ.Sum() - InitialLiquidEnthalpyJ + glassStored - burnerEnergyJ;
        }

        public void Step(Func<CutCellInventories, FlowStep, CutCellCoupledStep> advance, double burnerWatts = 0)
        {
            if (advance == null) throw new ArgumentNullException(nameof(advance));
            if (!SiUnits.IsFinite(burnerWatts) || burnerWatts < 0) throw new ArgumentOutOfRangeException(nameof(burnerWatts));
            if (burnerWatts > 0 && glass == null) throw new InvalidOperationException("Floor heating is disabled.");
            var trial = advance(State,Pressure);
            if (trial == null || !trial.Pressure.Diagnostics.Converged)
                throw new InvalidOperationException("The beaker pressure solve did not converge.");
            if (Math.Abs(trial.Transport.Fields.LiquidM3.Sum()-InitialLiquidVolumeM3) > 1e-12 ||
                trial.Momentum.MaxDualMassErrorKg > 1e-10)
                throw new InvalidOperationException("The beaker transport ledger did not close.");

            // Closed, source-free transport conserves the liquid enthalpy tracer.
            double enthalpyError = trial.Transport.Fields.LiquidEnergyJ.Sum() - InitialLiquidEnthalpyJ - LiquidHeatJ;
            double enthalpyTolerance = Math.Max(1e-10, Math.Abs(InitialLiquidEnthalpyJ) * 1e-6);
            if (!(Math.Abs(enthalpyError) <= enthalpyTolerance))
                throw new InvalidOperationException($"The beaker liquid enthalpy ledger did not close: {enthalpyError:E3} J.");

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

            if (glass != null) ExchangeFloorHeat(trial.Transport.Fields, trial.Pressure.Diagnostics.Dt, burnerWatts);
            TimeS += trial.Pressure.Diagnostics.Dt;
            State = trial.Transport.Fields;
            Pressure = trial.Pressure;
            Steps++;
            MaxDualMassErrorKg = Math.Max(MaxDualMassErrorKg, trial.Momentum.MaxDualMassErrorKg);
            MaxMomentumLedgerErrorKgMPerS = Math.Max(MaxMomentumLedgerErrorKgMPerS, error);

            double Part(int group, VariableCoefficientProjection.Face face) => group < 0 ? 0 :
                (LiquidDensityKgM3 * State.LiquidM3[group] + GasDensity * State.GasM3[group]) * face.Area /
                    componentAreas[face.Axis,group];
        }

        private double[] CreateCirculationVolumes(VariableCoefficientProjection topology, GeometryInputs input,
            int[] loop, int slice)
        {
            var grid = Geometry.Grid;
            var liquid = new double[topology.GroupCount];
            for (int k = 0; k < grid.Cells.Z; k++)
            for (int i = 0; i < grid.Cells.X; i++)
            {
                var position = grid.Centre(i, slice, k);
                if (position.X >= 0 || position.X * position.X + position.Z * position.Z > input.innerRadiusM * input.innerRadiusM)
                    continue;
                int cell = grid.Cells.Index(i, slice, k), group = topology.GroupByCell[cell];
                if (group >= 0) liquid[group] += .5 * Geometry.FluidVolumeM3[cell];
            }
            // Ensure a mixed donor on the seeded circulation, including cells
            // whose centres lie just outside the cylindrical cavity.
            var first = topology.Faces[loop[0]];
            liquid[first.Low] = .65 * topology.Volumes[first.Low];
            liquid[first.High] = .25 * topology.Volumes[first.High];
            return liquid;
        }

        private double[] CreateReleaseVolumes(VariableCoefficientProjection topology, GeometryInputs input, double volume, double surfaceJitterM)
        {
            // The box is inside the cavity and begins one cell above the base.
            // Only fully fluid cells are eligible for the exact box overlap.
            const double halfWidth = .01, clearance = .01;
            double bottom = input.externalBaseHeightM + input.baseThicknessM + clearance;
            double height = volume / (4 * halfWidth * halfWidth);
            if (height < Geometry.Grid.Spacing.Y || height >= input.cavityHeightM - clearance)
                throw new ArgumentOutOfRangeException(nameof(volume), "The release must span at least one cell and leave headspace.");
            var grid = Geometry.Grid;
            double amplitude = Math.Min(surfaceJitterM,
                Math.Min(height - grid.Spacing.Y, (input.cavityHeightM - clearance - height) / 2));
            // Equal-area quadrants with zero-sum offsets preserve the requested volume.
            double[] offsets = { -1, .5, -.25, .75 };
            var liquid = new double[topology.GroupCount];
            for (int k = 0; k < grid.Cells.Z; k++)
            for (int j = 0; j < grid.Cells.Y; j++)
            for (int i = 0; i < grid.Cells.X; i++)
            {
                var lower = grid.Frame.ToWorld(grid.Lower(i, j, k));
                double overlap = 0;
                for (int quadrant = 0; quadrant < offsets.Length; quadrant++)
                {
                    double left = (quadrant & 1) == 0 ? -halfWidth : 0;
                    double back = (quadrant & 2) == 0 ? -halfWidth : 0;
                    double top = bottom + height + amplitude * offsets[quadrant];
                    overlap += Overlap(lower.X, grid.Spacing.X, left, left + halfWidth) *
                        Overlap(lower.Y, grid.Spacing.Y, bottom, top) *
                        Overlap(lower.Z, grid.Spacing.Z, back, back + halfWidth);
                }
                if (overlap <= grid.CellVolume * 1e-12) continue;
                int cell = grid.Cells.Index(i, j, k), group = topology.GroupByCell[cell];
                if (group < 0 || Geometry.FluidVolumeM3[cell] < grid.CellVolume * (1 - 1e-10))
                    throw new InvalidOperationException("The release intersects the beaker solid.");
                liquid[group] += Math.Min(overlap, Geometry.FluidVolumeM3[cell]);
            }
            if (Math.Abs(liquid.Sum() - volume) > volume * 1e-10)
                throw new InvalidOperationException("The release does not fit the available fluid volume.");
            return liquid;

            double Overlap(double lower, double width, double start, double end) =>
                Math.Max(0, Math.Min(lower + width, end) - Math.Max(lower, start));
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
