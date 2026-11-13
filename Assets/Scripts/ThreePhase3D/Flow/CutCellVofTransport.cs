using System;
using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.Materials;
using PhysicsSim.ThreePhase3D.PhaseChange;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // Scalar reference path for the S04 pressure groups. Inventories are
    // extensive; open-boundary inputs use the same signed face ledger.
    public sealed class CutCellVofTransport
    {
        internal const double PressureCorrectionLimit = 1e-6;
        private const double Tolerance = 1e-10;
        private readonly VariableCoefficientProjection plan;
        private readonly CutCellVofGeometry geometry;
        private readonly CutCellMomentum momentum;
        private readonly int[] lowCell, highCell;
        private readonly int[] parent, parentFace;
        private readonly List<int> order = new List<int>();

        private readonly Func<int, WettingContact> contactAtCell;
        private readonly HashSet<InterfacePatch> wettablePatches;
        private readonly Dictionary<int, List<InterfacePatch>> trackedPatches;
        private readonly WettingProfile trackedWetting;
        private readonly PlanarWallCapillarity capillarity;
        private readonly bool planarBulkViscosity;

        public static CutCellVofTransport WithPlanarWallCapillarity(VariableCoefficientProjection plan,
            int firstFluidY, double surfaceTensionNPerM, double contactAngleDegrees = 90) =>
            new CutCellVofTransport(plan, null, null,
                new PlanarWallCapillarity(plan ?? throw new ArgumentNullException(nameof(plan)),
                    firstFluidY, surfaceTensionNPerM, contactAngleDegrees));

        public static CutCellVofTransport WithPlanarWallWetting(VariableCoefficientProjection plan,
            int firstFluidY, double surfaceTensionNPerM, WettingProfile profile, bool bulkViscosity = false)
        {
            if (plan == null) throw new ArgumentNullException(nameof(plan));
            if (profile == null) throw new ArgumentNullException(nameof(profile));
            return new CutCellVofTransport(plan, null, profile,
                new PlanarWallCapillarity(plan, firstFluidY, surfaceTensionNPerM, profile.EquilibriumAngleDegrees), bulkViscosity);
        }

        public static CutCellVofTransport WithTrackedWetting(VariableCoefficientProjection plan, WettingProfile profile) =>
            new CutCellVofTransport(plan, null, profile ?? throw new ArgumentNullException(nameof(profile)));

        public CutCellVofTransport(VariableCoefficientProjection plan) : this(plan, null) { }

        // The caller supplies the active contact cells and wall-relative speeds for the accepted step.
        public CutCellVofTransport(VariableCoefficientProjection plan, Func<int, WettingContact> contactAtCell)
            : this(plan, contactAtCell, null) { }

        private CutCellVofTransport(VariableCoefficientProjection plan, Func<int, WettingContact> contactAtCell,
            WettingProfile trackedWetting, PlanarWallCapillarity capillarity = null, bool planarBulkViscosity = false)
        {
            this.plan = plan ?? throw new ArgumentNullException(nameof(plan));
            this.contactAtCell = contactAtCell;
            if (contactAtCell != null)
            {
                wettablePatches = new HashSet<InterfacePatch>();
                foreach (var patch in plan.Geometry.Patches)
                    if (patch.Role == SolidRole.Fixed && patch.FluidExposedAreaM2 > 0)
                        wettablePatches.Add(patch);
            }
            this.trackedWetting = trackedWetting;
            this.capillarity = capillarity;
            this.planarBulkViscosity = planarBulkViscosity;
            if (trackedWetting != null)
            {
                trackedPatches = new Dictionary<int, List<InterfacePatch>>();
                foreach (var patch in plan.Geometry.Patches)
                {
                    // The exposed polygon is known exactly only for wholly exposed patches.
                    if (patch.Role != SolidRole.Fixed || patch.ContactAreaM2 != 0) continue;
                    if (!trackedPatches.TryGetValue(patch.Cell, out var patches))
                        trackedPatches.Add(patch.Cell, patches = new List<InterfacePatch>());
                    patches.Add(patch);
                }
            }
            geometry = new CutCellVofGeometry(plan.Geometry);
            momentum = new CutCellMomentum(plan);
            lowCell = new int[plan.FaceCount];
            highCell = new int[plan.FaceCount];
            for (int face = 0; face < plan.FaceCount; face++)
            {
                var metric = plan.Faces[face];
                if (metric.Active == 0) continue;
                int a = metric.Axis;
                plan.Grid.Shape((GridLocation)(a + 1)).Coordinates(face - plan.FaceOffsets[a], out int i, out int j, out int k);
                lowCell[face] = metric.Low < 0 ? -1 : plan.Grid.Cells.Index(i - (a == 0 ? 1 : 0),
                    j - (a == 1 ? 1 : 0), k - (a == 2 ? 1 : 0));
                highCell[face] = metric.High < 0 ? -1 : plan.Grid.Cells.Index(i, j, k);
            }
            parent = new int[plan.GroupCount];
            parentFace = new int[plan.GroupCount];
            for (int c = 0; c < plan.GroupCount; c++) { parent[c] = -2; parentFace[c] = -1; }
            // Seed all open groups before building the forest. Each such group
            // can close its own residual through a declared open face.
            for (int f = 0; f < plan.FaceCount; f++)
            {
                var metric = plan.Faces[f];
                if (metric.Active == 0 || (metric.Low >= 0 && metric.High >= 0)) continue;
                int group = Math.Max(metric.Low, metric.High);
                if (parent[group] != -2) continue;
                parent[group] = -1;
                parentFace[group] = f;
                order.Add(group);
            }
            int cursor = 0;
            Visit();
            for (int c = 0; c < plan.GroupCount; c++)
            {
                if (parent[c] != -2) continue;
                parent[c] = -1;
                order.Add(c);
                Visit();
            }

            void Visit()
            {
                for (; cursor < order.Count; cursor++)
                {
                    int group = order[cursor];
                    for (int entry = plan.Rows[group]; entry < plan.Rows[group + 1]; entry++)
                    {
                        int signed = plan.Incidence[entry], f = Math.Abs(signed) - 1;
                        var metric = plan.Faces[f];
                        int neighbour = signed > 0 ? metric.High : metric.Low;
                        if (neighbour < 0 || parent[neighbour] != -2) continue;
                        parent[neighbour] = group;
                        parentFace[neighbour] = f;
                        order.Add(neighbour);
                    }
                }
            }
        }

        public CutCellCoupledStep AdvanceCoupled(CutCellInventories state, FlowStep transportingFlow,
            double liquidDensity, double gasDensity, Func<int, CutCellInflow> inflow = null, int maxIterations = 4096) =>
            AdvanceCoupled(state, transportingFlow, liquidDensity, gasDensity, inflow, maxIterations, ProjectCpu);

        public CutCellCoupledStep AdvanceCoupledWithAcceleration(CutCellInventories state, FlowStep transportingFlow,
            double liquidDensity, double gasDensity, D3 worldAccelerationMPerS2,
            Func<int, CutCellInflow> inflow = null, int maxIterations = 4096)
            => AdvanceCoupled(state, transportingFlow, liquidDensity, gasDensity, inflow, maxIterations,
                (next, velocity, dt, cap) =>
                {
                    var correction = MeasurePressureCorrection(dt);
                    return FlowCpuReference.Project(next, velocity, dt, cap,
                        accepted => correction(accepted) <= PressureCorrectionLimit);
                }, worldAccelerationMPerS2: worldAccelerationMPerS2);

        public CutCellCoupledStep AdvanceCoupledWithWetting(CutCellInventories state, FlowStep transportingFlow,
            double liquidDensity, double gasDensity, double liquidViscosityPaS, double gasViscosityPaS,
            Func<int, CutCellInflow> inflow = null, int maxIterations = 4096)
        {
            return AdvanceCoupledWithWetting(state, transportingFlow, liquidDensity, gasDensity,
                liquidViscosityPaS, gasViscosityPaS, inflow, maxIterations, ProjectCpu);
        }

        internal CutCellCoupledStep AdvanceCoupledWithWetting(CutCellInventories state, FlowStep transportingFlow,
            double liquidDensity, double gasDensity, double liquidViscosityPaS, double gasViscosityPaS,
            Func<int, CutCellInflow> inflow, int maxIterations, PeriodicVofProjection.PressureSolve project,
            Func<CutCellScalarWork, CutCellScalarOutput> apply = null, CutCellGeometryBatch clip = null,
            Func<CutCellInventories, CutCellTransportResult, double[], double, double, CutCellMomentumResult> remap = null,
            Func<ContactTractionWork, double[]> traction = null)
        {
            if (trackedPatches == null) throw new InvalidOperationException("Tracked wetting is required.");
            if (!SiUnits.IsFinite(liquidViscosityPaS) || !SiUnits.IsFinite(gasViscosityPaS) ||
                liquidViscosityPaS <= 0 || gasViscosityPaS <= 0)
                throw new ArgumentException("Both dynamic viscosities must be finite and positive.");
            return AdvanceCoupled(state, transportingFlow, liquidDensity, gasDensity, inflow, maxIterations,
                project, apply, clip, (before, after, velocity, rhoL, rhoG) =>
                {
                    var advected = (remap ?? AdvanceMomentum)(before, after, velocity, rhoL, rhoG);
                    var contacts = new List<InterfacePatch>();
                    foreach (int cell in trackedPatches.Keys)
                    {
                        int group = plan.GroupByCell[cell];
                        if (group < 0 || after.Fields.LiquidM3[group] <= 0 || after.Fields.GasM3[group] <= 0) continue;
                        TrackContact(cell, group, after.Fields.LiquidM3[group],
                            EstimateNormal(cell, group, after.Fields.LiquidM3), advected.VelocityMPerS, contacts.Add);
                    }
                    var withTraction = momentum.ApplyContactTraction(advected, after.Fields, contacts, trackedWetting,
                        rhoL, rhoG, liquidViscosityPaS, gasViscosityPaS, transportingFlow.Diagnostics.Dt, traction, planarNoSlip: planarBulkViscosity);
                    return planarBulkViscosity ? momentum.ApplyPlanarBulkViscosity(withTraction, after.Fields,
                        rhoL, rhoG, liquidViscosityPaS, gasViscosityPaS, transportingFlow.Diagnostics.Dt) : withTraction;
                });
        }

        internal CutCellCoupledStep AdvanceCoupled(CutCellInventories state, FlowStep transportingFlow,
            double liquidDensity, double gasDensity, Func<int, CutCellInflow> inflow, int maxIterations,
            PeriodicVofProjection.PressureSolve project,
            Func<CutCellScalarWork, CutCellScalarOutput> apply = null, CutCellGeometryBatch clip = null,
            Func<CutCellInventories, CutCellTransportResult, double[], double, double, CutCellMomentumResult> remap = null,
            D3 worldAccelerationMPerS2 = default, double? adaptiveMaxStepS = null)
        {
            var transported = AdvanceProjected(state, transportingFlow, inflow, apply, clip, adaptiveMaxStepS, out double dt);
            var previousVelocity = Array.ConvertAll(transportingFlow.VelocityMPerS, value => (double)value);
            var advected = remap == null ? AdvanceMomentum(state, transported, previousVelocity, liquidDensity, gasDensity) :
                remap(state, transported, previousVelocity, liquidDensity, gasDensity);
            var nextPlan = new VariableCoefficientProjection(plan.Geometry, plan.Boundaries, cell =>
            {
                int group = plan.GroupByCell[cell];
                return (liquidDensity * transported.Fields.LiquidM3[group] +
                    gasDensity * transported.Fields.GasM3[group]) / plan.Volumes[group];
            }, _ => 0);
            var tentative = Array.ConvertAll(advected.VelocityMPerS, value => (float)value);
            if (worldAccelerationMPerS2.Length != 0)
            {
                var frame = nextPlan.Grid.Frame;
                var acceleration = new D3(D3.Dot(worldAccelerationMPerS2, frame.U),
                    D3.Dot(worldAccelerationMPerS2, frame.V), D3.Dot(worldAccelerationMPerS2, frame.W));
                for (int f = 0; f < nextPlan.FaceCount; f++)
                {
                    var face = nextPlan.Faces[f];
                    if (face.Active != 0)
                        tentative[f] = (float)(tentative[f] + dt * acceleration[face.Axis]);
                }
            }
            Dictionary<int, double> angles = null;
            if (capillarity != null && trackedPatches != null)
            {
                angles = new Dictionary<int, double>();
                var liquid = transported.Fields.LiquidM3;
                foreach (int cell in trackedPatches.Keys)
                {
                    plan.Grid.Cells.Coordinates(cell, out _, out int j, out _);
                    int group = plan.GroupByCell[cell];
                    if (j != capillarity.FirstFluidY || group < 0 || liquid[group] <= 0 ||
                        transported.Fields.GasM3[group] <= 0) continue;
                    var normal = EstimateNormal(cell, group, liquid);
                    var contact = TrackContact(cell, group, liquid[group], normal, advected.VelocityMPerS);
                    if (contact == null) continue;
                    angles.Add(cell, contact.ContactAngleDegrees(normal));
                }
            }
            capillarity?.Apply(nextPlan, transported.Fields, lowCell, highCell, liquidDensity, gasDensity,
                dt, tentative, angles);
            var pressure = project(nextPlan, tentative, dt, maxIterations);
            return new CutCellCoupledStep(transported, advected, pressure, angles?.Count ?? 0);
        }

        public CutCellMomentumResult AdvanceMomentum(CutCellInventories before, CutCellTransportResult transported,
            double[] velocity, double liquidDensity, double gasDensity)
        {
            if (before == null || transported == null || transported.Plan != plan)
                throw new ArgumentException("Momentum requires the scalar transfer from the same cut-cell topology.");
            Validate(before.Planes);
            return momentum.Advance(before, transported, velocity, liquidDensity, gasDensity);
        }

        public CutCellTransportResult AdvanceProjected(CutCellInventories state, FlowStep pressure,
            Func<int, CutCellInflow> inflow = null) => AdvanceProjected(state, pressure, inflow, null, null, null, out _);

        public (FlowStep Pressure, CutCellTransportResult Transport) AdvanceInventoryPhaseExchange(
            CutCellInventories state, double dt, double liquidDensityKgM3, double[] gasMassKg,
            double pressurePa, double filmThicknessM, double liquidConductanceWm2K, double gasConductanceWm2K,
            double lowerK, double upperK, Func<double[], FlowStep> project, Func<int, CutCellInflow> inflow = null)
        {
            if (gasMassKg == null || gasMassKg.Length != plan.GroupCount)
                throw new ArgumentException("Gas mass must match the pressure groups.", nameof(gasMassKg));
            return AdvanceReconstructedPhaseExchange(state, dt, liquidDensityKgM3, (liquid, gas) =>
                HumidInterfaceFlux.SolveFromInventories(state, liquid, gas, liquidDensityKgM3, gasMassKg[gas],
                    pressurePa, filmThicknessM, liquidConductanceWm2K, gasConductanceWm2K, lowerK, upperK),
                project, inflow);
        }

        public (FlowStep Pressure, CutCellTransportResult Transport) AdvanceReconstructedPhaseExchange(
            CutCellInventories state, double dt, double liquidDensityKgM3,
            Func<int, (GasLiquidEquilibrium Surface, double MassFluxKgM2PerS, double LiquidHeatWm2,
                double GasHeatWm2, double LatentHeatJkg)> solve,
            Func<double[], FlowStep> project, Func<int, CutCellInflow> inflow = null)
        {
            if (solve == null) throw new ArgumentNullException(nameof(solve));
            return AdvanceReconstructedPhaseExchange(state, dt, liquidDensityKgM3,
                (liquid, _) => solve(liquid), project, inflow);
        }

        public (FlowStep Pressure, CutCellTransportResult Transport) AdvanceReconstructedPhaseExchange(
            CutCellInventories state, double dt, double liquidDensityKgM3,
            Func<int, int, (GasLiquidEquilibrium Surface, double MassFluxKgM2PerS, double LiquidHeatWm2,
                double GasHeatWm2, double LatentHeatJkg)> solve,
            Func<double[], FlowStep> project, Func<int, CutCellInflow> inflow = null)
        {
            if (solve == null) throw new ArgumentNullException(nameof(solve));
            return AdvanceInterfaceExchange(state, dt, liquidDensityKgM3,
                (liquid, gas, area, availableLiquid, availableVapour) => HumidInterfaceFlux.InventoryTransfer(
                    solve(liquid, gas), area, dt, availableLiquid, availableVapour), project, inflow);
        }

        public (FlowStep Pressure, CutCellTransportResult Transport) AdvancePureWaterStefan(
            CutCellInventories state, double dt, PureWaterStefanInterface surface,
            Func<double[], FlowStep> project, Func<int, CutCellInflow> inflow = null) =>
            AdvancePureWaterStefanCore(state, dt, surface, project, inflow, null);

        public CutCellCoupledStep AdvancePureWaterStefanWithMomentum(
            CutCellInventories state, double dt, PureWaterStefanInterface surface,
            Func<double[], FlowStep> project, Func<int, CutCellInflow> inflow = null)
        {
            var transfers = new List<(int Donor, int Recipient, double MassKg)>();
            var exchanged = AdvancePureWaterStefanCore(state, dt, surface, project, inflow, transfers);
            var velocity = Array.ConvertAll(exchanged.Pressure.VelocityMPerS, value => (double)value);
            var advected = momentum.Advance(state, exchanged.Transport, velocity,
                surface.Saturation.Liquid.DensityKgM3, surface.Saturation.Vapour.DensityKgM3, transfers);
            return new CutCellCoupledStep(exchanged.Transport, advected, exchanged.Pressure);
        }

        private (FlowStep Pressure, CutCellTransportResult Transport) AdvancePureWaterStefanCore(
            CutCellInventories state, double dt, PureWaterStefanInterface surface,
            Func<double[], FlowStep> project, Func<int, CutCellInflow> inflow,
            List<(int Donor, int Recipient, double MassKg)> phaseTransfers)
        {
            if (surface == null) throw new ArgumentNullException(nameof(surface));
            if (state == null) throw new ArgumentNullException(nameof(state));
            Validate(state.Planes);
            double vapourDensity = surface.Saturation.Vapour.DensityKgM3;
            for (int group = 0; group < plan.GroupCount; group++)
                if (Math.Abs(state.VapourKg[group] - state.GasM3[group] * vapourDensity) >
                    1e-10 * plan.Volumes[group] * vapourDensity)
                    throw new ArgumentException("Pure-water Stefan exchange requires saturated vapour inventory.", nameof(state));
            Func<int, CutCellInflow> pureInflow = inflow == null ? null : face =>
            {
                var reservoir = inflow(face);
                if (reservoir.LiquidFraction < 1 &&
                    Math.Abs(reservoir.VapourDensity - vapourDensity) > 1e-10 * vapourDensity)
                    throw new ArgumentException("Pure-water Stefan backflow requires saturated vapour.", nameof(inflow));
                return reservoir;
            };
            return AdvanceInterfaceExchange(state, dt, surface.Saturation.Liquid.DensityKgM3,
                (_, _, area, availableLiquid, availableVapour) =>
                    surface.Transfer(area, dt, availableLiquid, availableVapour), project, pureInflow, phaseTransfers,
                surface);
        }

        private (FlowStep Pressure, CutCellTransportResult Transport) AdvanceInterfaceExchange(
            CutCellInventories state, double dt, double liquidDensityKgM3,
            Func<int, int, double, double, double, (double WaterKg, double LiquidM3, double GasM3,
                double LiquidEnthalpyJ, double GasEnthalpyJ)> calculate,
            Func<double[], FlowStep> project, Func<int, CutCellInflow> inflow,
            List<(int Donor, int Recipient, double MassKg)> phaseTransfers = null,
            PureWaterStefanInterface pureSurface = null)
        {
            if (!SiUnits.IsFinite(liquidDensityKgM3) || liquidDensityKgM3 <= 0 ||
                !SiUnits.IsFinite(dt) || dt <= 0)
                throw new ArgumentException("Phase exchange requires positive density and timestep.");
            // A roundoff-sized phase trace can reconstruct as a full face.
            // Exclude those sections from pure-water heating while retaining their inventories.
            var areas = ReconstructLiquidGasInterfaceAreas(state, pureSurface == null ? 0 : Tolerance);
            int count = plan.GroupCount;
            var increments = new CutCellInventories(new double[count], new double[count],
                new double[count], new double[count], new double[count]);
            var liquidUsed = new double[count];
            var vapourUsed = new double[count];
            for (int group = 0; group < count; group++)
            {
                if (areas[group] <= 0) continue;
                int gas = group;
                // A growing vapour layer thinner than its source volume cannot
                // supply that volume through the old PLIC slab. Deposit into
                // the adjacent resolved vapour on a grid-parallel interface.
                if (pureSurface != null && pureSurface.MassFluxKgM2PerS * areas[group] * dt >
                    state.GasM3[group] * pureSurface.Saturation.Vapour.DensityKgM3)
                    gas = AlignedVapourNeighbour(group);
                Add(group, gas, areas[group]);
            }
            for (int index = 0; index < plan.FaceCount; index++)
            {
                var face = plan.Faces[index];
                int low = face.Low, high = face.High;
                if (face.Active == 0 || low < 0 || high < 0 || low == high) continue;
                double area = plan.Geometry.Areas(face.Axis)[index - plan.FaceOffsets[face.Axis]];
                if (state.GasM3[low] == 0 && state.LiquidM3[high] == 0)
                    Add(low, high, area);
                else if (state.GasM3[high] == 0 && state.LiquidM3[low] == 0)
                    Add(high, low, area);
            }
            return AdvanceWithPhaseExchange(state, increments, dt, project, inflow);

            int AlignedVapourNeighbour(int group)
            {
                for (int entry = plan.Rows[group]; entry < plan.Rows[group + 1]; entry++)
                {
                    int signed = plan.Incidence[entry], index = Math.Abs(signed) - 1;
                    var face = plan.Faces[index];
                    int neighbour = signed > 0 ? face.High : face.Low;
                    if (neighbour < 0 || state.LiquidM3[neighbour] != 0) continue;
                    double aperture = plan.Geometry.Areas(face.Axis)[index - plan.FaceOffsets[face.Axis]];
                    if (Math.Abs(aperture - areas[group]) > 1e-8 * aperture) continue;
                    int cell = signed > 0 ? lowCell[index] : highCell[index];
                    var normal = EstimateNormal(cell, group, state.LiquidM3);
                    if (normal.Length > 0 && Math.Sign(signed) * normal[face.Axis] >= (1 - 1e-10) * normal.Length)
                        return neighbour;
                }
                return group;
            }

            void Add(int liquid, int gas, double area)
            {
                var transfer = calculate(liquid, gas, area,
                    state.LiquidM3[liquid] * liquidDensityKgM3 - liquidUsed[liquid],
                    state.VapourKg[gas] - vapourUsed[gas]);
                liquidUsed[liquid] += Math.Max(0, transfer.WaterKg);
                vapourUsed[gas] += Math.Max(0, -transfer.WaterKg);
                increments.LiquidM3[liquid] += transfer.LiquidM3;
                increments.GasM3[gas] += transfer.GasM3;
                increments.LiquidEnergyJ[liquid] += transfer.LiquidEnthalpyJ;
                increments.GasEnergyJ[gas] += transfer.GasEnthalpyJ;
                increments.VapourKg[gas] += transfer.WaterKg;
                if (phaseTransfers != null && liquid != gas && transfer.WaterKg != 0)
                    phaseTransfers.Add(transfer.WaterKg > 0
                        ? (liquid, gas, transfer.WaterKg) : (gas, liquid, -transfer.WaterKg));
            }
        }

        public (FlowStep Pressure, CutCellTransportResult Transport) AdvanceWithPhaseExchange(
            CutCellInventories state, CutCellInventories increments, double dt,
            Func<double[], FlowStep> project, Func<int, CutCellInflow> inflow = null)
        {
            if (state == null || increments == null || project == null) throw new ArgumentNullException();
            if (!SiUnits.IsFinite(dt) || dt <= 0) throw new ArgumentOutOfRangeException(nameof(dt));
            foreach (var field in increments.Planes)
                if (field == null || field.Length != plan.GroupCount || !Array.TrueForAll(field, SiUnits.IsFinite))
                    throw new ArgumentException("Phase exchange increments must match the pressure groups.");
            Validate(state.Planes);
            var volume = new double[plan.GroupCount];
            var source = new double[plan.GroupCount];
            for (int group = 0; group < volume.Length; group++)
            {
                volume[group] = increments.LiquidM3[group] + increments.GasM3[group];
                source[group] = -volume[group] / dt;
                if (!SiUnits.IsFinite(source[group]) ||
                    state.LiquidM3[group] + increments.LiquidM3[group] < 0 ||
                    state.GasM3[group] + increments.GasM3[group] < 0 ||
                    state.VapourKg[group] + increments.VapourKg[group] < 0)
                    throw new ArgumentException("Phase exchange would overdraw its donor.");
            }
            plan.ValidateBoundaryFlux(source);
            var pressure = project(source);
            if (pressure == null || !pressure.Diagnostics.Converged || pressure.Diagnostics.Dt != dt ||
                pressure.VelocityMPerS.Length != plan.FaceCount || pressure.GroupPressurePa.Length != plan.GroupCount)
                throw new InvalidOperationException("Phase exchange requires a converged pressure projection on the same step.");
            var velocity = new double[plan.FaceCount];
            double maximum = ReconcilePressureFlux(pressure.VelocityMPerS, dt, velocity,
                out double absolute, targetVolume: volume);
            if (maximum > PressureCorrectionLimit)
                throw new InvalidOperationException("Phase exchange pressure flux exceeds the transport correction budget.");
            return (pressure, Advance(state, velocity, dt, inflow, absolute, null, increments: increments));
        }

        internal CutCellTransportResult AdvanceParcelDisplacement(CutCellInventories state, double[] before,
            double[] after, double dt, Func<double[], FlowStep> project)
        {
            if (before.Length != plan.GroupCount || after.Length != plan.GroupCount || !SiUnits.IsFinite(dt) || dt <= 0)
                throw new ArgumentException("Parcel displacement requires matching groups and a positive timestep.");
            var volume = new double[plan.GroupCount];
            var source = new double[volume.Length];
            for (int group = 0; group < volume.Length; group++)
            {
                if (!(before[group] >= 0 && before[group] < plan.Volumes[group] &&
                    after[group] >= 0 && after[group] < plan.Volumes[group]) ||
                    ((before[group] > 0 || after[group] > 0) && state.LiquidM3[group] != 0))
                    throw new ArgumentException("Parcel displacement is supported in gas-only groups with free volume.");
                volume[group] = after[group] - before[group];
                source[group] = -volume[group] / dt;
            }
            Validate(state.Planes, before);
            plan.ValidateBoundaryFlux(source);
            var pressure = project(source);
            if (!pressure.Diagnostics.Converged || pressure.Diagnostics.Dt != dt ||
                pressure.VelocityMPerS.Length != plan.FaceCount)
                throw new InvalidOperationException("Parcel displacement requires a compatible pressure projection.");
            var velocity = new double[plan.FaceCount];
            if (ReconcilePressureFlux(pressure.VelocityMPerS, dt, velocity, out double absolute,
                targetVolume: volume) > PressureCorrectionLimit)
                throw new InvalidOperationException("Parcel displacement pressure flux exceeds the transport correction budget.");
            return Advance(state, velocity, dt, null, absolute, null, displacementBefore: before, displacementAfter: after);
        }

        internal CutCellTransportResult AdvanceProjected(CutCellInventories state, FlowStep pressure,
            Func<int, CutCellInflow> inflow, Func<CutCellScalarWork, CutCellScalarOutput> apply, CutCellGeometryBatch clip,
            double? adaptiveMaxStepS, out double dt)
        {
            if (pressure == null || !pressure.Diagnostics.Converged || pressure.VelocityMPerS.Length != plan.FaceCount)
                throw new ArgumentException("A converged compatible pressure result is required.");
            dt = adaptiveMaxStepS ?? pressure.Diagnostics.Dt;
            if (!SiUnits.IsFinite(dt) || dt <= 0) throw new ArgumentOutOfRangeException(nameof(adaptiveMaxStepS));
            var velocity = new double[plan.FaceCount];
            double maximum = ReconcilePressureFlux(pressure.VelocityMPerS, dt, velocity, out double absolute);
            if (adaptiveMaxStepS.HasValue)
            {
                double courant = OutgoingCourant(velocity, dt);
                double scale = 1;
                if (courant > .5) scale = .45 / courant;
                if (maximum > PressureCorrectionLimit)
                    scale = Math.Min(scale, .9 * PressureCorrectionLimit / maximum);
                if (scale < 1)
                {
                    dt *= scale;
                    maximum = ReconcilePressureFlux(pressure.VelocityMPerS, dt, velocity, out absolute);
                }
            }
            if (maximum > PressureCorrectionLimit)
                throw new InvalidOperationException($"Cut-cell pressure flux correction {maximum:E3} exceeds 1e-6 Courant units.");
            return Advance(state, velocity, dt, inflow, absolute, apply, clip);
        }

        private double OutgoingCourant(double[] velocity, double dt)
        {
            var outgoing = new double[plan.GroupCount];
            double maximum = 0;
            for (int f = 0; f < velocity.Length; f++)
            {
                var metric = plan.Faces[f];
                if (metric.Active == 0 || velocity[f] == 0) continue;
                bool positive = velocity[f] > 0;
                int donor = positive ? metric.Low : metric.High;
                if (donor < 0) continue;
                int cell = positive ? lowCell[f] : highCell[f];
                double volume = Math.Abs(velocity[f]) * metric.Area * dt;
                outgoing[donor] += volume;
                maximum = Math.Max(maximum, volume / plan.Geometry.FluidVolumeM3[cell]);
            }
            for (int group = 0; group < outgoing.Length; group++)
                maximum = Math.Max(maximum, outgoing[group] / plan.Volumes[group]);
            return maximum;
        }

        private FlowStep ProjectCpu(VariableCoefficientProjection nextPlan, float[] tentative, double dt, int maxIterations)
        {
            if (capillarity == null) return FlowCpuReference.Project(nextPlan, tentative, dt, maxIterations);
            var correction = MeasurePressureCorrection(dt);
            return FlowCpuReference.Project(nextPlan, tentative, dt, maxIterations,
                velocity => correction(velocity) <= PressureCorrectionLimit);
        }

        internal Func<float[], double> MeasurePressureCorrection(double dt)
        {
            var scratch = new double[plan.FaceCount];
            var divergence = new double[plan.GroupCount];
            return velocity => ReconcilePressureFlux(velocity, dt, scratch, out _, divergence);
        }

        private double ReconcilePressureFlux(float[] accepted, double dt, double[] velocity,
            out double absolute, double[] divergence = null, double[] targetVolume = null)
        {
            double maximum = 0;
            absolute = 0;
            if (divergence == null) divergence = new double[plan.GroupCount];
            else Array.Clear(divergence, 0, divergence.Length);
            if (targetVolume != null)
                for (int group = 0; group < divergence.Length; group++) divergence[group] = -targetVolume[group];
            for (int f = 0; f < velocity.Length; f++)
            {
                velocity[f] = accepted[f];
                var metric = plan.Faces[f];
                if (metric.Active == 0) continue;
                double q = velocity[f] * metric.Area * dt;
                if (metric.Low >= 0) divergence[metric.Low] += q;
                if (metric.High >= 0) divergence[metric.High] -= q;
            }
            for (int index = order.Count - 1; index >= 0; index--)
            {
                int group = order[index], f = parentFace[group];
                if (f < 0) continue;
                var metric = plan.Faces[f];
                double delta = metric.Low == group ? -divergence[group] : divergence[group];
                velocity[f] += delta / (metric.Area * dt);
                double volume = parent[group] < 0 ? plan.Volumes[group] :
                    Math.Min(plan.Volumes[group], plan.Volumes[parent[group]]);
                maximum = Math.Max(maximum, Math.Abs(delta) / volume);
                absolute += Math.Abs(delta);
                if (parent[group] >= 0) divergence[parent[group]] += divergence[group];
            }
            return maximum;
        }

        public CutCellTransportResult Advance(CutCellInventories state, double[] faceVelocity, double dt,
            Func<int, CutCellInflow> inflow = null) => Advance(state, faceVelocity, dt, inflow, 0, null);

        internal CutCellTransportResult Advance(CutCellInventories state, double[] faceVelocity, double dt,
            Func<int, CutCellInflow> inflow, double pressureCorrection, Func<CutCellScalarWork, CutCellScalarOutput> apply,
            CutCellGeometryBatch clip = null, CutCellInventories increments = null,
            double[] displacementBefore = null, double[] displacementAfter = null)
        {
            if (state == null || faceVelocity == null || faceVelocity.Length != plan.FaceCount ||
                !SiUnits.IsFinite(dt) || dt <= 0) throw new ArgumentException("Invalid cut-cell transport inputs.");
            var fields = state.Planes;
            Validate(fields, displacementBefore);
            int groups = plan.GroupCount, faces = plan.FaceCount;
            var swept = new double[faces];
            var correction = new double[faces];
            var divergence = new double[groups];
            var outgoing = new double[groups];
            var planes = new ClipPlane?[plan.Grid.Cells.Count];
            var normals = trackedPatches == null ? null : new D3?[planes.Length];
            var reservoir = new CutCellInflow[faces];
            var jobs = clip == null ? null : new List<CutCellClipJob>();
            double roundoff = 0;

            for (int f = 0; f < faces; f++)
            {
                var metric = plan.Faces[f];
                double u = faceVelocity[f];
                if (!SiUnits.IsFinite(u) || (metric.Active == 0 && u != 0))
                    throw new ArgumentException("A blocked face has nonzero or nonfinite velocity.");
                if (u == 0) continue;
                double q = u * metric.Area * dt;
                if (!SiUnits.IsFinite(q)) throw new ArgumentException("Nonfinite swept volume.");
                if (q == 0) continue;
                swept[f] = q;
                if (metric.Low >= 0) divergence[metric.Low] += q;
                if (metric.High >= 0) divergence[metric.High] -= q;
                int donor = q > 0 ? metric.Low : metric.High;
                if (donor >= 0) outgoing[donor] += Math.Abs(q);
                else
                {
                    if (inflow == null) throw new ArgumentException("An open inflow requires a reservoir state.");
                    reservoir[f] = inflow(f);
                    reservoir[f].Validate();
                }
            }
            for (int c = 0; c < groups; c++)
            {
                double volume = plan.Volumes[c] - (displacementBefore?[c] ?? 0);
                if (displacementBefore != null) divergence[c] -= displacementAfter[c] - displacementBefore[c];
                if (increments != null) divergence[c] -= increments.LiquidM3[c] + increments.GasM3[c];
                if (Math.Abs(divergence[c]) > Tolerance * volume)
                    throw new ArgumentException($"Cut-cell group {c} has relative volume imbalance {Math.Abs(divergence[c]) / volume:E3} (limit {Tolerance:E3}).");
                if (outgoing[c] > .5 * volume)
                    throw new ArgumentException($"Cut-cell group {c} outgoing CFL {outgoing[c] / volume:G6} exceeds 0.5.");
            }
            for (int f = 0; f < faces; f++)
            {
                double q = swept[f];
                if (q == 0) continue;
                var metric = plan.Faces[f];
                int donor = q > 0 ? metric.Low : metric.High;
                if (donor < 0) continue;
                int cell = q > 0 ? lowCell[f] : highCell[f];
                double widthVolume = Math.Abs(q), volume = plan.Geometry.FluidVolumeM3[cell];
                if (widthVolume > .5 * volume)
                    throw new ArgumentException("Cut-face transfer exceeds half its donor member's accessible volume.");
                double alpha = fields[0][donor] / plan.Volumes[donor];
                if (alpha == 0 || alpha == 1) continue;
                if (jobs != null)
                {
                    jobs.Add(new CutCellClipJob(f, cell, metric.Axis, q > 0, DonorNormal(cell, donor, fields[0], faceVelocity, normals),
                        alpha, volume, widthVolume));
                    continue;
                }
                if (!planes[cell].HasValue)
                    planes[cell] = geometry.Reconstruct(cell, DonorNormal(cell, donor, fields[0], faceVelocity, normals), alpha * volume);
                Record(f, geometry.Sweep(cell, metric.Axis, q > 0, widthVolume, planes[cell].Value),
                    widthVolume, alpha, Tolerance * volume);
            }
            if (jobs != null && jobs.Count > 0)
            {
                var values = clip(geometry, jobs);
                if (values == null || values.Length != jobs.Count)
                    throw new InvalidOperationException("GPU clipping returned an incompatible face list.");
                for (int n = 0; n < jobs.Count; n++)
                {
                    var job = jobs[n];
                    Record(job.Face, values[n], job.SweptVolume, job.Alpha,
                        2e-6 * plan.Grid.CellVolume);
                }
            }

            void Record(int f, double raw, double width, double alpha, double tolerance)
            {
                if (!SiUnits.IsFinite(raw) || raw < -tolerance || raw > width + tolerance)
                    throw new InvalidOperationException("Cut PLIC liquid exceeds the accessible swept slab.");
                double geometric = Math.Max(0, Math.Min(width, raw));
                roundoff += Math.Abs(geometric - raw);
                correction[f] = geometric - alpha * width;
            }

            if (increments != null)
                LimitPhaseExchangeOutflow(state, increments, swept, correction, outgoing, displacementBefore);

            var work = new CutCellScalarWork(plan, fields, swept, correction, outgoing, reservoir, displacementBefore);
            var output = apply == null ? ApplyReference(work) : apply(work);
            var next = output.Fields;
            if (increments != null)
                for (int field = 0; field < next.Length; field++)
                for (int group = 0; group < groups; group++)
                    next[field][group] += increments.Planes[field][group];
            roundoff += output.RoundoffM3;
            for (int c = 0; c < groups; c++)
            {
                double volume = plan.Volumes[c] - (displacementAfter?[c] ?? 0);
                if (displacementAfter != null && displacementAfter[c] > 0 && next[0][c] > Tolerance * volume)
                    throw new InvalidOperationException("Resolved liquid entered a parcel-displaced gas group.");
                if (next[0][c] < -Tolerance * volume || next[1][c] < -Tolerance * volume ||
                    Math.Abs(next[0][c] + next[1][c] - volume) > 4 * Tolerance * volume)
                    throw new InvalidOperationException("Cut-cell phase volume closure failed.");
                // Close the larger phase by subtraction to preserve a trace phase
                // and its advected enthalpy when the complement rounds to the cell volume.
                bool liquidIsSmaller = next[0][c] <= next[1][c];
                double smaller = Math.Max(0, Math.Min(volume, liquidIsSmaller ? next[0][c] : next[1][c]));
                double liquid = liquidIsSmaller ? smaller : volume - smaller;
                double gas = liquidIsSmaller ? volume - smaller : smaller;
                roundoff += Math.Abs(liquid - next[0][c]) + Math.Abs(gas - next[1][c]);
                next[0][c] = liquid;
                next[1][c] = gas;
                // Volume and enthalpy are accumulated independently. If phase
                // exchange exhausts the liquid, carry cancellation roundoff into
                // the remaining gas rather than leave energy in an empty phase.
                // Material inconsistencies still fail validation.
                if (increments != null && liquid == 0 && gas > 0 &&
                    Math.Abs(next[2][c]) <= 1e-15)
                {
                    next[3][c] += next[2][c];
                    next[2][c] = 0;
                }
            }
            Validate(next, displacementAfter);
            return new CutCellTransportResult(plan, new CutCellInventories(next), output.Liquid, output.Gas,
                output.Boundary, reservoir, roundoff, output.LimitedM3, pressureCorrection);
        }

        // Both backends must reserve phase-change withdrawals before limiting
        // geometric advection. The upwind contribution is already committed.
        private void LimitPhaseExchangeOutflow(CutCellInventories state, CutCellInventories increments,
            double[] swept, double[] correction, double[] outgoing, double[] displacement)
        {
            var liquidDemand = new double[plan.GroupCount];
            var gasDemand = new double[plan.GroupCount];
            for (int f = 0; f < swept.Length; f++)
            {
                if (swept[f] == 0) continue;
                int donor = swept[f] > 0 ? plan.Faces[f].Low : plan.Faces[f].High;
                if (donor < 0) continue;
                liquidDemand[donor] += Math.Max(0, correction[f]);
                gasDemand[donor] += Math.Max(0, -correction[f]);
            }

            for (int c = 0; c < plan.GroupCount; c++)
            {
                double volume = plan.Volumes[c] - (displacement?[c] ?? 0);
                double retained = volume > 0 ? 1 - outgoing[c] / volume : 1;
                double liquidSource = increments.LiquidM3[c];
                double liquidEnergy = state.LiquidEnergyJ[c];
                if (liquidEnergy > 0 && increments.LiquidEnergyJ[c] < 0)
                    liquidSource = Math.Min(liquidSource,
                        state.LiquidM3[c] * (increments.LiquidEnergyJ[c] / liquidEnergy));
                liquidDemand[c] = ScaleDemand(liquidDemand[c], state.LiquidM3[c] * retained, liquidSource);
                gasDemand[c] = ScaleDemand(gasDemand[c], state.GasM3[c] * retained, increments.GasM3[c]);
            }

            for (int f = 0; f < swept.Length; f++)
            {
                if (swept[f] == 0) continue;
                int donor = swept[f] > 0 ? plan.Faces[f].Low : plan.Faces[f].High;
                if (donor >= 0) correction[f] *= correction[f] >= 0 ? liquidDemand[donor] : gasDemand[donor];
            }

            double ScaleDemand(double demand, double available, double source)
            {
                if (source >= 0) return 1;
                double budget = available + source;
                if (budget < 0)
                    throw new InvalidOperationException("Phase exchange and upwind transport exceed the donor inventory. Reduce the timestep.");
                // Keep some donor phase to carry its remaining scalar inventory.
                return demand > 0 ? Math.Min(1, .99 * budget / demand) : 1;
            }
        }

        private static CutCellScalarOutput ApplyReference(CutCellScalarWork work)
        {
            var plan = work.Plan;
            var fields = work.Fields;
            var swept = work.Swept;
            var correction = work.Correction;
            var outgoing = work.Outgoing;
            var increase = new double[plan.GroupCount];
            var decrease = new double[plan.GroupCount];
            for (int f = 0; f < plan.FaceCount; f++)
            {
                if (swept[f] == 0) continue;
                var face = plan.Faces[f];
                int donor = swept[f] > 0 ? face.Low : face.High;
                if (donor < 0) continue;
                increase[donor] += Math.Max(0, correction[f]);
                decrease[donor] += Math.Max(0, -correction[f]);
            }
            var reservoir = work.Reservoir;
            int faces = plan.FaceCount;
            var next = new double[5][];
            var remaining = new double[5][];
            for (int field = 0; field < 5; field++)
            {
                next[field] = new double[plan.GroupCount];
                remaining[field] = (double[])fields[field].Clone();
            }
            var boundary = new double[5];
            var liquidFace = new double[faces];
            var gasFace = new double[faces];
            double limited = 0, roundoff = 0;
            for (int f = 0; f < faces; f++)
            {
                double q = swept[f];
                if (q == 0) continue;
                var metric = plan.Faces[f];
                int donor = q > 0 ? metric.Low : metric.High;
                double l, g, el, eg, vapour;
                if (donor >= 0)
                {
                    double delta = correction[f], volume = plan.Volumes[donor] - (work.Displacement?[donor] ?? 0);
                    double fraction = fields[0][donor] / volume;
                    double gasFraction = fields[1][donor] / volume;
                    double capacity = (delta < 0 ? gasFraction : fraction) * (volume - outgoing[donor]);
                    double demand = delta < 0 ? decrease[donor] : increase[donor];
                    double accepted = demand > capacity ? delta * (capacity / demand) : delta;
                    limited += Math.Abs(delta - accepted);
                    double proposed = fraction * Math.Abs(q) + accepted;
                    if (proposed < -Tolerance * volume || proposed > Math.Abs(q) + Tolerance * volume)
                        throw new InvalidOperationException("Cut-cell limiter exceeded the swept volume.");
                    double proposedGas = gasFraction * Math.Abs(q) - accepted;
                    double width = Math.Abs(q);
                    double liquid = proposed <= proposedGas ? Math.Max(0, Math.Min(width, proposed)) : 0;
                    double gas = proposed <= proposedGas ? width - liquid : Math.Max(0, Math.Min(width, proposedGas));
                    if (proposed > proposedGas) liquid = width - gas;
                    if (liquid > remaining[0][donor])
                    {
                        liquid = remaining[0][donor];
                        gas = width - liquid;
                    }
                    if (gas > remaining[1][donor])
                    {
                        gas = remaining[1][donor];
                        liquid = width - gas;
                    }
                    if (liquid > remaining[0][donor] || Math.Abs(liquid - proposed) > Tolerance * volume)
                        throw new InvalidOperationException("Cut-cell transport overdraws a phase donor.");
                    roundoff += Math.Abs(liquid - proposed);
                    l = Math.Sign(q) * liquid;
                    g = Math.Sign(q) * gas;
                    el = Take(2, l, remaining[0][donor]);
                    eg = Take(3, g, remaining[1][donor]);
                    vapour = Take(4, g, remaining[1][donor]);
                    remaining[0][donor] -= liquid;
                    remaining[1][donor] -= gas;

                    double Take(int field, double amount, double available)
                    {
                        double value = Scale(Math.Abs(amount), available, remaining[field][donor]);
                        remaining[field][donor] -= value;
                        return Math.Sign(q) * value;
                    }
                }
                else
                {
                    var source = reservoir[f];
                    l = q * source.LiquidFraction;
                    g = q - l;
                    el = l * source.LiquidEnergyDensity;
                    eg = g * source.GasEnergyDensity;
                    vapour = g * source.VapourDensity;
                }
                liquidFace[f] = l;
                gasFace[f] = g;
                Transfer(0, l); Transfer(1, g); Transfer(2, el); Transfer(3, eg); Transfer(4, vapour);

                void Transfer(int field, double amount)
                {
                    int receiver = q > 0 ? metric.High : metric.Low;
                    if (receiver >= 0) next[field][receiver] += Math.Sign(q) * amount;
                    if (metric.Low < 0) boundary[field] += amount;
                    if (metric.High < 0) boundary[field] -= amount;
                }
            }
            for (int field = 0; field < next.Length; field++)
            for (int group = 0; group < plan.GroupCount; group++)
                next[field][group] += remaining[field][group];

            return new CutCellScalarOutput(next, liquidFace, gasFace, boundary, roundoff, limited);
        }

        private D3 DonorNormal(int cell, int group, double[] liquid, double[] velocity, D3?[] cache)
        {
            if (cache != null && cache[cell].HasValue) return cache[cell].Value;
            var normal = EstimateNormal(cell, group, liquid);
            var contact = contactAtCell?.Invoke(cell);
            if (contact != null && (contact.Wall.Cell != cell || !wettablePatches.Contains(contact.Wall)))
                throw new ArgumentException("Contact requires an exposed fixed-wall patch of the donor cell.");
            if (cache != null)
                contact = TrackContact(cell, group, liquid[group], normal, velocity);
            if (contact != null) normal = contact.InterfaceNormal(normal);
            if (cache != null) cache[cell] = normal;
            return normal;
        }

        internal WettingContact TrackContact(int cell, int group, double liquidVolume, D3 normal, double[] velocity,
            Action<InterfacePatch> onIntersection = null)
        {
            if (normal.Length == 0 || !trackedPatches.TryGetValue(cell, out var patches)) return null;
            var plane = geometry.Reconstruct(cell, normal, liquidVolume / plan.Volumes[group] * plan.Geometry.FluidVolumeM3[cell]);
            var spacing = plan.Grid.Spacing;
            double tolerance = 1e-11 * Math.Min(spacing.X, Math.Min(spacing.Y, spacing.Z));
            InterfacePatch selected = null;
            foreach (var patch in patches)
            {
                bool negative = false, positive = false;
                foreach (var point in patch.Polygon)
                {
                    double distance = plane.Distance(point);
                    negative |= distance < -tolerance;
                    positive |= distance > tolerance;
                }
                if (!negative || !positive) continue;
                onIntersection?.Invoke(patch);
                if (selected == null || patch.AreaM2 > selected.AreaM2) selected = patch;
            }
            if (selected == null) return null;
            D3 tangent = plane.N - selected.NormalLocal * D3.Dot(plane.N, selected.NormalLocal);
            if (tangent.Length < 1e-12) return null;

            plan.Grid.Cells.Coordinates(cell, out int i, out int j, out int k);
            var sampled = new D3(Component(0), Component(1), Component(2));
            return new WettingContact(trackedWetting, selected, D3.Dot(sampled, tangent.Unit()));

            double Component(int axis)
            {
                int lower = plan.FaceIndex(axis, i, j, k);
                int upper = plan.FaceIndex(axis, i + (axis == 0 ? 1 : 0),
                    j + (axis == 1 ? 1 : 0), k + (axis == 2 ? 1 : 0));
                double a = plan.Faces[lower].Area, b = plan.Faces[upper].Area;
                return a + b == 0 ? 0 : (a * velocity[lower] + b * velocity[upper]) / (a + b);
            }
        }

        public double[] ReconstructLiquidGasInterfaceAreas(CutCellInventories fields) =>
            ReconstructLiquidGasInterfaceAreas(fields, 0);

        private double[] ReconstructLiquidGasInterfaceAreas(CutCellInventories fields, double minimumFraction)
        {
            if (fields == null) throw new ArgumentNullException(nameof(fields));
            Validate(fields.Planes);
            var areas = new double[plan.GroupCount];
            for (int cell = 0; cell < plan.GroupByCell.Length; cell++)
            {
                int group = plan.GroupByCell[cell];
                if (group < 0 || Math.Min(fields.LiquidM3[group], fields.GasM3[group]) <=
                    minimumFraction * plan.Volumes[group]) continue;
                var plane = geometry.Reconstruct(cell, EstimateNormal(cell, group, fields.LiquidM3),
                    fields.LiquidM3[group] / plan.Volumes[group] * plan.Geometry.FluidVolumeM3[cell]);
                areas[group] += geometry.LiquidGasInterfaceArea(cell, plane);
            }
            return areas;
        }

        public double[] ReconstructGasFaceAreas(CutCellInventories fields) => ReconstructPhaseFaceAreas(fields, false);

        internal double[] ReconstructLiquidFaceAreas(CutCellInventories fields) => ReconstructPhaseFaceAreas(fields, true);

        private double[] ReconstructPhaseFaceAreas(CutCellInventories fields, bool liquidPhase)
        {
            if (fields == null) throw new ArgumentNullException(nameof(fields));
            if (fields.LiquidM3.Length != plan.GroupCount || fields.GasM3.Length != plan.GroupCount)
                throw new ArgumentException("Phase volumes must match the pressure groups.", nameof(fields));
            for (int group = 0; group < plan.GroupCount; group++)
                if (!SiUnits.IsFinite(fields.LiquidM3[group]) || !SiUnits.IsFinite(fields.GasM3[group]) ||
                    fields.LiquidM3[group] < 0 || fields.GasM3[group] < 0 ||
                    Math.Abs(fields.LiquidM3[group] + fields.GasM3[group] - plan.Volumes[group]) >
                        1e-10 * plan.Grid.CellVolume)
                    throw new ArgumentException("Phase volumes must fill the pressure groups.", nameof(fields));

            var grid = plan.Grid;
            var phase = liquidPhase ? fields.LiquidM3 : fields.GasM3;
            var other = liquidPhase ? fields.GasM3 : fields.LiquidM3;
            var result = new double[plan.FaceCount];
            var planes = new ClipPlane?[grid.Cells.Count];
            for (int face = 0; face < result.Length; face++)
            {
                var metric = plan.Faces[face];
                if (metric.Active == 0 || metric.Low == metric.High ||
                    (metric.Low >= 0 && phase[metric.Low] == 0) ||
                    (metric.High >= 0 && phase[metric.High] == 0)) continue;
                if ((metric.Low < 0 || other[metric.Low] == 0) &&
                    (metric.High < 0 || other[metric.High] == 0))
                {
                    result[face] = metric.Area;
                    continue;
                }
                int axis = metric.Axis;
                grid.Shape((GridLocation)(axis + 1)).Coordinates(face - plan.FaceOffsets[axis],
                    out int i, out int j, out int k);
                int x = i - (axis == 0 ? 1 : 0), y = j - (axis == 1 ? 1 : 0), z = k - (axis == 2 ? 1 : 0);
                if ((metric.Low >= 0 && !grid.Cells.Interior(x, y, z)) ||
                    (metric.High >= 0 && !grid.Cells.Interior(i, j, k)))
                    throw new NotSupportedException("Mixed gas aperture reconstruction requires nonperiodic faces.");
                int low = grid.Cells.Index(x, y, z);
                int high = grid.Cells.Index(i, j, k);
                var gasLow = PhasePlane(low, metric.Low);
                var gasHigh = PhasePlane(high, metric.High);
                double area = metric.Area;
                if (metric.Low >= 0) area = Math.Min(area, geometry.GasFaceSection(low, axis, true, gasLow, gasHigh));
                if (metric.High >= 0) area = Math.Min(area, geometry.GasFaceSection(high, axis, false, gasLow, gasHigh));
                result[face] = area;
            }
            return result;

            ClipPlane? PhasePlane(int cell, int group)
            {
                if (group < 0 || other[group] == 0) return null;
                if (planes[cell].HasValue) return planes[cell];
                var liquid = geometry.Reconstruct(cell, EstimateNormal(cell, group, fields.LiquidM3),
                    fields.LiquidM3[group] / plan.Volumes[group] * plan.Geometry.FluidVolumeM3[cell]);
                return planes[cell] = liquidPhase ? liquid : new ClipPlane(-liquid.N, liquid.N * liquid.Offset);
            }
        }

        internal D3 EstimateNormal(int cell, int group, double[] liquid)
        {
            plan.Grid.Cells.Coordinates(cell, out int i, out int j, out int k);
            double alpha = liquid[group] / plan.Volumes[group];
            return new D3(Difference(0), Difference(1), Difference(2));

            double Difference(int axis) => (SampleAt(axis, -1) - SampleAt(axis, 1)) / plan.Grid.Spacing[axis];

            // The pressure topology masks the solid and out-of-domain faces.
            double SampleAt(int axis, int sign)
            {
                int face = plan.FaceIndex(axis, i + (axis == 0 && sign > 0 ? 1 : 0),
                    j + (axis == 1 && sign > 0 ? 1 : 0), k + (axis == 2 && sign > 0 ? 1 : 0));
                var metric = plan.Faces[face];
                int other = sign < 0 ? metric.Low : metric.High;
                return metric.Active == 0 || other < 0 ? alpha : liquid[other] / plan.Volumes[other];
            }

        }

        private static double Scale(double transfer, double volume, double inventory)
        {
            if (transfer == 0) return 0;
            if (volume <= 0) throw new InvalidOperationException("A transfer originates in an empty cut-cell phase.");
            return inventory * (transfer / volume);
        }

        private void Validate(double[][] fields, double[] displacement = null)
        {
            foreach (double[] field in fields)
                if (field == null || field.Length != plan.GroupCount || !Array.TrueForAll(field, SiUnits.IsFinite))
                    throw new ArgumentException("Invalid cut-cell inventory shape or value.");
            for (int c = 0; c < plan.GroupCount; c++)
            {
                if (fields[0][c] < 0 || fields[1][c] < 0 || fields[4][c] < 0 ||
                    fields[0][c] > plan.Volumes[c] || fields[1][c] > plan.Volumes[c] ||
                    Math.Abs(fields[0][c] + fields[1][c] + (displacement?[c] ?? 0) - plan.Volumes[c]) > Tolerance * plan.Volumes[c] ||
                    (fields[0][c] == 0 && fields[2][c] != 0) ||
                    (fields[1][c] == 0 && (fields[3][c] != 0 || fields[4][c] != 0)))
                    throw new ArgumentException($"Invalid cut-cell phase inventory in group {c}: liquid {fields[0][c]:G17}, gas {fields[1][c]:G17}, liquid enthalpy {fields[2][c]:G17}, gas enthalpy {fields[3][c]:G17}, vapour {fields[4][c]:G17}.");
            }
        }
    }

    internal delegate double[] CutCellGeometryBatch(CutCellVofGeometry geometry, IReadOnlyList<CutCellClipJob> jobs);

    internal readonly struct CutCellClipJob
    {
        internal readonly int Face, Cell, Axis;
        internal readonly bool Positive;
        internal readonly D3 Normal;
        internal readonly double Alpha, FluidVolume, SweptVolume;
        internal CutCellClipJob(int face, int cell, int axis, bool positive, D3 normal,
            double alpha, double fluidVolume, double sweptVolume)
        {
            Face = face; Cell = cell; Axis = axis; Positive = positive; Normal = normal;
            Alpha = alpha; FluidVolume = fluidVolume; SweptVolume = sweptVolume;
        }
    }

    internal sealed class CutCellScalarWork
    {
        internal readonly VariableCoefficientProjection Plan;
        internal readonly double[][] Fields;
        internal readonly double[] Swept, Correction, Outgoing;
        internal readonly CutCellInflow[] Reservoir;
        internal readonly double[] Displacement;
        internal CutCellScalarWork(VariableCoefficientProjection plan, double[][] fields, double[] swept,
            double[] correction, double[] outgoing, CutCellInflow[] reservoir, double[] displacement = null)
        {
            Plan = plan; Fields = fields; Swept = swept; Correction = correction;
            Outgoing = outgoing; Reservoir = reservoir;
            Displacement = displacement;
        }
    }

    internal sealed class CutCellScalarOutput
    {
        internal readonly double[][] Fields;
        internal readonly double[] Liquid, Gas, Boundary;
        internal readonly double RoundoffM3, LimitedM3;
        internal CutCellScalarOutput(double[][] fields, double[] liquid, double[] gas,
            double[] boundary, double roundoff, double limited)
        {
            Fields = fields; Liquid = liquid; Gas = gas; Boundary = boundary;
            RoundoffM3 = roundoff; LimitedM3 = limited;
        }
    }

    public sealed class CutCellCoupledStep
    {
        public readonly CutCellTransportResult Transport;
        public readonly CutCellMomentumResult Momentum;
        public readonly FlowStep Pressure;
        public readonly int CapillaryWettingContacts;
        internal CutCellCoupledStep(CutCellTransportResult transport, CutCellMomentumResult momentum, FlowStep pressure,
            int capillaryWettingContacts = 0)
        {
            Transport = transport; Momentum = momentum; Pressure = pressure;
            CapillaryWettingContacts = capillaryWettingContacts;
        }
    }

    public sealed class CutCellInventories
    {
        internal const int LiquidVolume = 0, GasVolume = 1, LiquidEnergy = 2;
        internal readonly double[][] Planes;
        public double[] LiquidM3 => Planes[LiquidVolume];
        public double[] GasM3 => Planes[GasVolume];
        public double[] LiquidEnergyJ => Planes[LiquidEnergy];
        public double[] GasEnergyJ => Planes[3];
        public double[] VapourKg => Planes[4];
        public CutCellInventories(double[] liquid, double[] gas, double[] liquidEnergy, double[] gasEnergy, double[] vapour)
            : this(new[] { liquid, gas, liquidEnergy, gasEnergy, vapour }) { }
        internal CutCellInventories(double[][] planes) { Planes = planes; }
    }

    public readonly struct CutCellInflow
    {
        public readonly double LiquidFraction, LiquidEnergyDensity, GasEnergyDensity, VapourDensity;
        public readonly D3 VelocityMPerS;
        public CutCellInflow(double fraction, double liquidEnergyDensity, double gasEnergyDensity, double vapourDensity, D3 velocityMPerS = default)
        {
            LiquidFraction = fraction; LiquidEnergyDensity = liquidEnergyDensity;
            GasEnergyDensity = gasEnergyDensity; VapourDensity = vapourDensity; VelocityMPerS = velocityMPerS;
        }
        internal void Validate()
        {
            if (!SiUnits.IsFinite(LiquidFraction) || LiquidFraction < 0 || LiquidFraction > 1 ||
                !SiUnits.IsFinite(LiquidEnergyDensity) || !SiUnits.IsFinite(GasEnergyDensity) ||
                !SiUnits.IsFinite(VapourDensity) || VapourDensity < 0)
                throw new ArgumentException("Invalid open-boundary reservoir.");
        }
    }

    public sealed class CutCellTransportResult
    {
        internal readonly VariableCoefficientProjection Plan;
        internal readonly CutCellInflow[] Reservoir;
        public readonly CutCellInventories Fields;
        public readonly double[] LiquidFaceM3, GasFaceM3, BoundaryInflow;
        public readonly double RoundoffM3, LimitedM3, PressureFluxCorrectionM3;
        internal CutCellTransportResult(VariableCoefficientProjection plan, CutCellInventories fields, double[] liquid, double[] gas,
            double[] boundary, CutCellInflow[] reservoir, double roundoff, double limited, double pressureCorrection)
        {
            Plan = plan; Reservoir = reservoir;
            Fields = fields; LiquidFaceM3 = liquid; GasFaceM3 = gas; BoundaryInflow = boundary;
            RoundoffM3 = roundoff; LimitedM3 = limited; PressureFluxCorrectionM3 = pressureCorrection;
        }
    }
}
