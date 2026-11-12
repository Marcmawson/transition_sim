using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.Materials;
using PhysicsSim.ThreePhase3D.Verification;

internal static class FlowMain
{
    public static int Main(string[] args)
    {
        if (args.Length == 1 && args[0] == "--bubble-rise")
        {
            CheckResolvedBubbleRise();
            return 0;
        }
        if (args.Length == 1 && args[0] == "--bubble-growth-rise")
        {
            CheckResolvedBubbleRise(growing: true);
            return 0;
        }
        if (args.Length == 1 && args[0] == "--pure-stefan")
        {
            CheckPureWaterStefan();
            return 0;
        }
        if (args.Length == 1 && args[0] == "--bubble-growth")
        {
            CheckSeededBubbleGrowth();
            return 0;
        }
        if (args.Length == 1 && args[0] == "--bubble-condensation")
        {
            CheckBubbleCondensation();
            return 0;
        }
        if (args.Length == 1 && args[0] == "--capillary-bubble")
        {
            CheckCapillaryBubble();
            return 0;
        }
        if (args.Length == 1 && args[0] == "--humid-diffusion")
        {
            CheckHumidDiffusion();
            CheckReconstructedGasDiffusion();
            CheckOpenGasDiffusion();
            CheckHumidLowMach();
            HumidPhaseExchangeCheck.Run((plan, tentative, dt, source) =>
                FlowCpuReference.Project(plan, tentative, dt, 4096, null, source), Console.WriteLine);
            HumidPhaseExchangeCheck.RunSphericalFilm(Console.WriteLine);
            HumidPhaseExchangeCheck.RunTransientCondensation(Console.WriteLine);
            return 0;
        }
        var suite = new FlowChecks(FlowCpuReference.Advance);
        try { suite.Run(); }
        finally
        {
            int passed = 0;
            foreach (var row in suite.Rows)
            {
                Console.WriteLine(FormattableString.Invariant(
                    $"{(row.Passed ? "PASS" : "FAIL")} {row.Case} | {row.Quantity} | {row.Actual:G10} | {row.Expected:G10} ± {row.Tolerance:G10}"));
                if (row.Passed) passed++;
            }
            Console.WriteLine($"{passed}/{suite.Rows.Count} passed");
        }
        return suite.Passed ? 0 : 1;
    }

    private static void CheckResolvedBubbleRise(bool growing = false)
    {
        const double dx = 0.001, dt = 0.0004;
        var grid = new Grid3D(8, 8, 8, new D3(dx, dx, dx), RigidFrame.Identity);
        var centre = new D3(0.004, 0.003, 0.004);
        var sphere = SurfacePrimitives.Transform(SurfacePrimitives.Sphere(0.0016, 12, 24),
            RigidFrame.Euler(centre, 0, 0, 0));
        var seed = CutCellGeometry.Build(grid, iceSurface: sphere);
        var domain = CutCellGeometry.Build(grid);
        var surface = new PureWaterStefanInterface(101325, 370, 377, growing ? 10000 : 0, 0);
        var saturation = surface.Saturation;
        double rhoL = saturation.Liquid.DensityKgM3, rhoG = saturation.Vapour.DensityKgM3;
        var sides = new FlowBoundary[6];
        if (growing) sides[3] = new FlowBoundary(FlowBoundaryKind.OpenPressure);
        var boundaries = new BoundaryConditions3D(sides);
        var plan = new VariableCoefficientProjection(domain, boundaries,
            cell => rhoL + (rhoG - rhoL) * seed.IceVolumeM3[cell] / grid.CellVolume, _ => 0);
        var gas = new double[plan.GroupCount];
        var centres = new D3[plan.GroupCount];
        for (int cell = 0; cell < plan.GroupByCell.Length; cell++)
        {
            int group = plan.GroupByCell[cell];
            if (group < 0) continue;
            grid.Cells.Coordinates(cell, out int x, out int y, out int z);
            gas[group] = seed.IceVolumeM3[cell];
            centres[group] = grid.Centre(x, y, z);
        }
        var liquid = new double[gas.Length];
        for (int group = 0; group < gas.Length; group++) liquid[group] = plan.Volumes[group] - gas[group];
        var state = new CutCellInventories(liquid, gas,
            Array.ConvertAll(liquid, v => v * rhoL * saturation.Liquid.EnthalpyJkg),
            Array.ConvertAll(gas, v => v * rhoG * saturation.Vapour.EnthalpyJkg),
            Array.ConvertAll(gas, v => v * rhoG));
        var transport = new CutCellVofTransport(plan);
        double volume = Sum(gas);
        D3 initial = Centre(gas);
        if (growing)
        {
            var noGravity = RunGrowing(0);
            var withGravity = RunGrowing(-9.81);
            var rise = Centre(withGravity.GasM3) - initial;
            double control = (Centre(noGravity.GasM3) - initial).Y;
            if (!(rise.Y - control > 1e-8 && Sum(withGravity.GasM3) > volume))
                throw new InvalidOperationException("The phase-changing vapour seed did not grow and rise.");
            Console.WriteLine(FormattableString.Invariant(
                $"S13 growing bubble passed: 6 steps, rise {rise.Y:G8} m, no-gravity rise {control:G8} m, vapour gained {Sum(withGravity.GasM3) - volume:G8} m3."));
            return;
        }
        var still = Run(default);
        var rising = Run(new D3(0, -9.81, 0));
        var displacement = Centre(rising.GasM3) - initial;
        if ((Centre(still.GasM3) - initial).Length > 1e-12 || displacement.Y <= 1e-8 ||
            Math.Sqrt(displacement.X * displacement.X + displacement.Z * displacement.Z) > 0.02 * displacement.Y)
            throw new InvalidOperationException($"The resolved vapour seed did not rise symmetrically: {displacement.Y:G8} m.");
        Console.WriteLine(FormattableString.Invariant(
            $"S13 resolved bubble rise passed: 6 steps, vertical centre displacement {displacement.Y:G8} m, vapour volume {volume:G8} m3."));

        CutCellInventories Run(D3 gravity)
        {
            var fields = state;
            var pressure = FlowCpuReference.Project(plan, new float[plan.FaceCount], dt, 32768);
            for (int step = 0; step < 6; step++)
            {
                var result = transport.AdvanceCoupledWithAcceleration(fields, pressure, rhoL, rhoG,
                    gravity, maxIterations: 32768);
                fields = result.Transport.Fields;
                pressure = result.Pressure;
                for (int field = 0; field < state.Planes.Length; field++)
                {
                    double before = Sum(state.Planes[field]);
                    if (Math.Abs(Sum(fields.Planes[field]) - before) > Math.Abs(before) * 1e-9)
                        throw new InvalidOperationException("The closed bubble fixture failed an inventory balance.");
                }
            }
            return fields;
        }

        CutCellInventories RunGrowing(double gravityY)
        {
            var fields = state;
            var velocity = new float[plan.FaceCount];
            double water = Sum(state.LiquidM3) * rhoL + Sum(state.VapourKg);
            double energy = Sum(state.LiquidEnergyJ) + Sum(state.GasEnergyJ);
            double waterIn = 0, energyIn = 0, vapourProduced = 0, gasIn = 0;
            for (int step = 0; step < 6; step++)
            {
                var current = new VariableCoefficientProjection(domain, boundaries, cell =>
                {
                    int group = plan.GroupByCell[cell];
                    return (rhoL * fields.LiquidM3[group] + rhoG * fields.GasM3[group]) / plan.Volumes[group];
                }, _ => 0);
                var tentative = (float[])velocity.Clone();
                for (int face = 0; face < current.FaceCount; face++)
                    if (current.Faces[face].Active != 0 && current.Faces[face].Axis == 1)
                        tentative[face] = (float)(tentative[face] + dt * gravityY);
                var result = new CutCellVofTransport(current).AdvancePureWaterStefanWithMomentum(fields, dt, surface, source =>
                {
                    vapourProduced += -Sum(source) * dt / (1 / rhoG - 1 / rhoL) / rhoG;
                    return FlowCpuReference.Project(current, tentative, dt, 32768, null, source);
                }, _ => new CutCellInflow(1, rhoL * saturation.Liquid.EnthalpyJkg, 0, 0));
                var incomingMomentum = new CutCellMomentum(current).GatherGroupMomentum(fields,
                    result.Pressure.VelocityMPerS, rhoL, rhoG);
                var outgoingMomentum = new double[3];
                for (int face = 0; face < current.FaceCount; face++)
                    outgoingMomentum[current.Faces[face].Axis] += result.Momentum.MomentumKgMPerS[face];
                for (int axis = 0; axis < 3; axis++)
                    if (Math.Abs(outgoingMomentum[axis] - Sum(incomingMomentum[axis]) -
                        result.Momentum.BoundaryMomentumKgMPerS[axis] - result.Momentum.WallImpulseKgMPerS[axis]) > 1e-15)
                        throw new InvalidOperationException("The growing bubble failed its momentum balance.");
                fields = result.Transport.Fields;
                velocity = Array.ConvertAll(result.Momentum.VelocityMPerS, value => (float)value);
                var boundary = result.Transport.BoundaryInflow;
                waterIn += boundary[0] * rhoL + boundary[4];
                energyIn += boundary[2] + boundary[3];
                gasIn += boundary[1];
                if (Math.Abs(Sum(fields.GasM3) - volume - vapourProduced - gasIn) > 1e-16 ||
                    Math.Abs(Sum(fields.LiquidM3) * rhoL + Sum(fields.VapourKg) - water - waterIn) > 1e-13 ||
                    Math.Abs(Sum(fields.LiquidEnergyJ) + Sum(fields.GasEnergyJ) - energy - energyIn) > 1e-8)
                    throw new InvalidOperationException($"The growing bubble failed its phase or outlet balance at step {step + 1}.");
            }
            return fields;
        }

        D3 Centre(double[] volumes)
        {
            D3 firstMoment = default;
            for (int group = 0; group < volumes.Length; group++) firstMoment += centres[group] * volumes[group];
            return firstMoment / Sum(volumes);
        }
        double Sum(double[] values) => System.Linq.Enumerable.Sum(values);
    }

    private static void CheckCapillaryBubble()
    {
        const double seed = 0.0005, pressure = 101325, conductivity = 0.68, duration = 0.02;
        var ambient = new PureWaterStefanInterface(pressure, 370, 377, 0, 0);
        double boilingK = ambient.Saturation.Liquid.TemperatureK;
        double sigma = new Water95().SurfaceTensionNm(boilingK);
        foreach (double offset in new[] { 2.0, -2.0 })
        {
            double bulkK = boilingK + offset;
            var initial = PureWaterStefanInterface.SphericalConduction(seed, pressure, sigma,
                bulkK, conductivity, 370, 377);
            double initialMass = Mass(seed, initial);
            var coarse = Advance(4);
            var fine = Advance(16);
            var reference = Advance(64);
            double error = Math.Abs(fine.Radius - reference.Radius);
            if (!(error < Math.Abs(coarse.Radius - reference.Radius) / 10 && error < 1e-8) ||
                Math.Sign(reference.Radius - seed) != Math.Sign(offset) ||
                Math.Sign(reference.Heat) != Math.Sign(offset))
                throw new InvalidOperationException("Capillary bubble evolution did not refine in the expected direction.");
            Console.WriteLine(FormattableString.Invariant(
                $"S13 capillary bubble passed: bulk offset {offset:G} K, radius {reference.Radius:G8} m, latent heat into interface {reference.Heat:G8} J, refinement difference {error:G6} m."));

            (double Radius, double Heat) Advance(int steps)
            {
                double mass = initialMass, radius = seed, heat = 0, dt = duration / steps;
                var surface = initial;
                for (int step = 0; step < steps; step++)
                {
                    double midMass = mass + 0.5 * dt * Area(radius) * surface.MassFluxKgM2PerS;
                    var mid = State(midMass);
                    var exchange = mid.Interface.Transfer(Area(mid.RadiusM), dt, 1, mass);
                    mass += exchange.WaterKg;
                    heat += exchange.WaterKg * mid.Interface.Saturation.LatentEnthalpyJkg;
                    (radius, surface) = State(mass);
                    if (Math.Abs(Mass(radius, surface) - mass) > 1e-10 * mass ||
                        Math.Abs(surface.Saturation.PressurePa - (pressure + 2 * sigma / radius)) > 2e-5 ||
                        Math.Abs(exchange.LiquidEnthalpyJ + exchange.VapourEnthalpyJ) > 1e-14)
                        throw new InvalidOperationException("Capillary bubble pressure, mass or phase enthalpy did not close.");
                }
                return (radius, heat);
            }

            (double RadiusM, PureWaterStefanInterface Interface) State(double mass) =>
                PureWaterStefanInterface.SphericalFromVapourMass(mass, seed * 0.5, seed * 2,
                    pressure, sigma, bulkK, conductivity, 370, 377);
        }
        double Area(double radius) => 4 * Math.PI * radius * radius;
        double Mass(double radius, PureWaterStefanInterface surface) =>
            (4 * Math.PI / 3 * radius * radius * radius) * surface.Saturation.Vapour.DensityKgM3;
    }

    private static void CheckBubbleCondensation()
    {
        const double pressure = 101325, seedRadius = 0.0005, cutoff = 0.00025, conductivity = 0.68, subcooling = 2;
        var saturation = new PureWaterStefanInterface(pressure, 370, 377, 0, 0).Saturation;
        double bulkK = saturation.Liquid.TemperatureK - subcooling;
        double density = saturation.Vapour.DensityKgM3, latent = saturation.LatentEnthalpyJkg;
        double recession = 2 * conductivity * subcooling / (density * latent);
        double referenceTime = seedRadius * seedRadius / recession;
        double resolvedTime = (seedRadius * seedRadius - cutoff * cutoff) / recession;
        double originalMass = Mass(seedRadius);
        double coarse = Resolve(32).Radius;
        var fine = Resolve(128);
        double error = Math.Abs(fine.Radius / cutoff - 1);
        if (!(error < Math.Abs(coarse / cutoff - 1) / 12 && error < 1e-4))
            throw new InvalidOperationException("The shrinking bubble radius did not refine to the spherical reference.");

        var tail = Surface(fine.Radius).CondenseUnresolvedVapour(fine.Mass);
        double tailHeat = -tail.WaterKg * latent;
        double extinction = resolvedTime + fine.Radius * fine.Radius / recession;
        if (fine.Mass + tail.WaterKg != 0 ||
            Math.Abs(fine.LiquidKg + tail.LiquidM3 * saturation.Liquid.DensityKgM3 - originalMass) > 1e-20 ||
            Math.Abs(fine.Heat + tailHeat - originalMass * latent) > 1e-14 ||
            Math.Abs(tail.LiquidEnthalpyJ + tail.VapourEnthalpyJ) > 1e-15 ||
            Math.Abs(extinction / referenceTime - 1) > 1e-4)
            throw new InvalidOperationException("Bubble extinction failed its mass, enthalpy or time balance.");
        Console.WriteLine(FormattableString.Invariant(
            $"S13 V31 bubble condensation passed: cutoff radius {fine.Radius:G8} m, extinction {extinction:G8} s, condensed {originalMass:G8} kg, latent heat {fine.Heat + tailHeat:G8} J, final vapour 0 kg."));

        PureWaterStefanInterface Surface(double radius) => PureWaterStefanInterface.SphericalConduction(
            radius, pressure, 0, bulkK, conductivity, 370, 377);
        double Area(double radius) => 4 * Math.PI * radius * radius;
        double Mass(double radius) => Area(radius) * radius * density / 3;
        double Radius(double mass) => Math.Pow(mass * 3 / (4 * Math.PI * density), 1.0 / 3);
        (double Radius, double Mass, double LiquidKg, double Heat) Resolve(int steps)
        {
            double mass = originalMass, radius = seedRadius, liquid = 0, heat = 0, dt = resolvedTime / steps;
            for (int step = 0; step < steps; step++)
            {
                double midpoint = Radius(mass + 0.5 * dt * Area(radius) * Surface(radius).MassFluxKgM2PerS);
                var surface = Surface(midpoint);
                var exchange = surface.Transfer(Area(midpoint), dt, 0, mass);
                if (!(exchange.WaterKg < 0) || Math.Abs(exchange.LiquidEnthalpyJ + exchange.VapourEnthalpyJ) > 1e-14)
                    throw new InvalidOperationException("The resolved condensation step did not conserve enthalpy.");
                mass += exchange.WaterKg;
                radius = Radius(mass);
                liquid += exchange.LiquidM3 * saturation.Liquid.DensityKgM3;
                heat -= surface.LiquidHeatWm2 * Area(midpoint) * dt;
            }
            return (radius, mass, liquid, heat);
        }
    }

    private static void CheckSeededBubbleGrowth()
    {
        const double pressure = 101325, initialRadius = 0.0005, conductivity = 0.68, duration = 0.1;
        const double lower = 370, upper = 382, superheat = 2;
        var ambient = new PureWaterStefanInterface(pressure, lower, upper, 0, 0);
        double temperature = ambient.Saturation.Liquid.TemperatureK + superheat;
        double density = ambient.Saturation.Vapour.DensityKgM3;
        double latent = ambient.Saturation.LatentEnthalpyJkg;
        double analyticRadius = Math.Sqrt(initialRadius * initialRadius +
            2 * conductivity * superheat * duration / (density * latent));
        var coarse = Run(16);
        var fine = Run(64);
        double error = Math.Abs(fine.Radius / analyticRadius - 1);
        if (!(error < Math.Abs(coarse.Radius / analyticRadius - 1) / 10 && error < 0.001))
            throw new InvalidOperationException("Spherical bubble growth did not converge to the quasi-steady radius.");
        double referenceHeat = (Mass(analyticRadius) - Mass(initialRadius)) * latent;
        if (Math.Abs(fine.Heat / referenceHeat - 1) > 0.002)
            throw new InvalidOperationException("Bubble growth heat disagrees with the reference.");

        var water = new Water95();
        double tension = water.SurfaceTensionNm(ambient.Saturation.Liquid.TemperatureK);
        double criticalRadius = 2 * tension / (water.Saturation(temperature).PressurePa - pressure);
        var critical = AtRadius(criticalRadius, tension);
        var large = AtRadius(2 * criticalRadius, tension);
        var small = AtRadius(0.5 * criticalRadius, tension);
        var flat = AtRadius(2 * criticalRadius, 0);
        if (Math.Abs(critical.Saturation.Liquid.TemperatureK - temperature) > 1e-8 ||
            !(large.MassFluxKgM2PerS > 0 && large.MassFluxKgM2PerS < flat.MassFluxKgM2PerS &&
                small.MassFluxKgM2PerS < 0))
            throw new InvalidOperationException("The capillary pressure did not give the expected growth threshold.");
        Console.WriteLine(FormattableString.Invariant(
            $"S13 V30 spherical growth passed: radius {fine.Radius:G8} m, heat {fine.Heat:G8} J, relative radius error {error:G6}, capillary threshold {criticalRadius:G8} m."));

        PureWaterStefanInterface AtRadius(double radius, double sigma) =>
            PureWaterStefanInterface.SphericalConduction(radius, pressure, sigma, temperature, conductivity, lower, upper);
        double Mass(double radius) => 4 * Math.PI / 3 * radius * radius * radius * density;
        (double Radius, double Heat) Run(int steps)
        {
            double mass = Mass(initialRadius), radius = initialRadius, heat = 0;
            double dt = duration / steps;
            for (int step = 0; step < steps; step++)
            {
                var start = AtRadius(radius, 0);
                double midpointMass = mass + 0.5 * dt * 4 * Math.PI * radius * radius * start.MassFluxKgM2PerS;
                double midpointRadius = Math.Pow(midpointMass * 3 / (4 * Math.PI * density), 1.0 / 3);
                var surface = AtRadius(midpointRadius, 0);
                double area = 4 * Math.PI * midpointRadius * midpointRadius;
                var exchange = surface.Transfer(area, dt, 1, mass);
                if (!(exchange.WaterKg > 0) || Math.Abs(exchange.LiquidEnthalpyJ + exchange.VapourEnthalpyJ) > 1e-14)
                    throw new InvalidOperationException("Bubble growth did not close the phase enthalpy balance.");
                mass += exchange.WaterKg;
                radius = Math.Pow(mass * 3 / (4 * Math.PI * density), 1.0 / 3);
                heat += surface.LiquidHeatWm2 * area * dt;
            }
            if (Math.Abs(heat - (Mass(radius) - Mass(initialRadius)) * latent) > 1e-13)
                throw new InvalidOperationException("Bubble latent heat did not match its vapour inventory.");
            return (radius, heat);
        }
    }

    private static void CheckPureWaterStefan()
    {
        const double pressure = 101325, dx = 0.01, dt = 0.002;
        var grid = new Grid3D(1, 3, 1, new D3(dx, dx, dx), RigidFrame.Identity);
        double area = grid.FaceArea(1);
        var sides = new FlowBoundary[6];
        sides[3] = new FlowBoundary(FlowBoundaryKind.OpenPressure);
        foreach (double liquidHeat in new[] { 10000.0, -10000.0 })
        foreach (double initialHeight in new[] { 0.01, 0.015 })
        {
            const int steps = 4;
            var surface = new PureWaterStefanInterface(pressure, 370, 377, liquidHeat, 0);
            var liquid = surface.Saturation.Liquid;
            var vapour = surface.Saturation.Vapour;
            Near(Math.Abs(liquid.TemperatureK - 373.1243), 5e-5);
            Near(Math.Abs(liquid.TemperatureK - vapour.TemperatureK), 0);
            if (!(liquid.DensityKgM3 / vapour.DensityKgM3 > 1500))
                throw new InvalidOperationException("The Stefan fixture requires the physical liquid/vapour density contrast.");
            var plan = new VariableCoefficientProjection(CutCellGeometry.Build(grid),
                new BoundaryConditions3D(sides), _ => vapour.DensityKgM3, _ => 0);
            var transport = new CutCellVofTransport(plan);
            var fields = PlanarLiquidInventory.Create(plan, new D3(0, initialHeight, 0), new D3(0, 1, 0),
                liquid.DensityKgM3 * liquid.EnthalpyJkg, vapour.DensityKgM3 * vapour.EnthalpyJkg, vapour.DensityKgM3);
            double Sum(double[] values) => System.Linq.Enumerable.Sum(values);
            double initialLiquid = Sum(fields.LiquidM3);
            double initialWater = initialLiquid * liquid.DensityKgM3 + Sum(fields.VapourKg);
            double initialHeat = Sum(fields.LiquidEnergyJ) + Sum(fields.GasEnergyJ);
            double boundaryWater = 0, boundaryHeat = 0;
            for (int step = 0; step < steps; step++)
            {
                var result = transport.AdvancePureWaterStefanWithMomentum(fields, dt, surface,
                    source => FlowCpuReference.Project(plan, new float[plan.FaceCount], dt, 4096, null, source),
                    _ => new CutCellInflow(0, 0, vapour.DensityKgM3 * vapour.EnthalpyJkg, vapour.DensityKgM3));
                fields = result.Transport.Fields;
                if (initialHeight == dx && liquidHeat > 0)
                    Near(fields.LiquidM3[plan.GroupByCell[grid.Cells.Index(0, 1, 0)]], 1e-18);
                var boundary = result.Transport.BoundaryInflow;
                Near(Math.Abs(boundary[0]), 1e-18);
                boundaryWater += boundary[4];
                boundaryHeat += boundary[2] + boundary[3];
                double outlet = result.Pressure.VelocityMPerS[plan.FaceIndex(1, 0, 3, 0)];
                Near(Math.Abs(outlet - surface.VapourMinusLiquidVelocityMPerS), 1e-7);
            }
            double frontShift = (Sum(fields.LiquidM3) - initialLiquid) / area;
            double analyticShift = -liquidHeat * dt * steps /
                (surface.Saturation.LatentEnthalpyJkg * liquid.DensityKgM3);
            Near(Math.Abs(frontShift - analyticShift), 1e-11);
            Near(Math.Abs(Sum(fields.LiquidM3) * liquid.DensityKgM3 + Sum(fields.VapourKg) -
                initialWater - boundaryWater), 1e-14);
            Near(Math.Abs(Sum(fields.LiquidEnergyJ) + Sum(fields.GasEnergyJ) - initialHeat - boundaryHeat), 1e-9);
            Console.WriteLine(FormattableString.Invariant(
                $"S13 V29 planar Stefan passed ({(initialHeight == dx ? "initially aligned" : "initially mixed")}): flux {surface.MassFluxKgM2PerS:G8} kg/m2/s, front shift {frontShift:G8} m, velocity jump {surface.VapourMinusLiquidVelocityMPerS:G8} m/s."));
        }

        void Near(double error, double tolerance)
        {
            if (!(error <= tolerance))
                throw new InvalidOperationException($"Planar Stefan error {error:G8} exceeds {tolerance:G8}.");
        }
    }

    private static void CheckHumidLowMach()
    {
        const double pressure = 101325, gasMass = 0.001, dt = 0.01;
        var model = new HumidGas();
        var ambient = model.Evaluate(293.15, pressure, 0.005);
        var warmer = model.Evaluate(298.15, pressure, 0.005);
        var wetter = model.Evaluate(293.15, pressure, 0.01);
        var gravity = new D3(0, -9.81, 0);
        double thermal = HumidGasLowMach.BuoyancyMPerS2(warmer, ambient, gravity).Y;
        double species = HumidGasLowMach.BuoyancyMPerS2(wetter, ambient, gravity).Y;
        double expansion = HumidGasLowMach.ProjectionSourceM3PerS(ambient, warmer, gasMass, dt);
        double recoveredVolume = gasMass / ambient.DensityKgM3 - dt * expansion;
        if (!(thermal > 0 && species > 0 && expansion < 0) ||
            Math.Abs(recoveredVolume - gasMass / warmer.DensityKgM3) > 1e-15 ||
            HumidGasLowMach.BuoyancyMPerS2(ambient, ambient, gravity).Length != 0 ||
            HumidGasLowMach.ProjectionSourceM3PerS(ambient, ambient, gasMass, dt) != 0)
            throw new InvalidOperationException("Humid low Mach source or buoyancy failed its limiting checks.");
        Console.WriteLine(FormattableString.Invariant(
            $"S12 humid low Mach passed: thermal buoyancy {thermal:G8} m/s2, species buoyancy {species:G8} m/s2, projection source {expansion:G8} m3/s."));
    }

    private static void CheckReconstructedGasDiffusion()
    {
        CheckReconstructedGasDiffusion(null, 0.01);
        CheckReconstructedGasDiffusion(SurfacePrimitives.Box(new D3(0, 0, 0.008),
            new D3(0.02, 0.02, 0.01)), 0.008);
    }

    private static void CheckReconstructedGasDiffusion(TriangleSurface solid, double openWidth)
    {
        const double dx = 0.01, rho = 1.2, dt = 0.1, diffusivity = 2.5e-5;
        var grid = new Grid3D(2, 2, 1, new D3(dx, dx, dx), RigidFrame.Identity);
        var plan = new VariableCoefficientProjection(CutCellGeometry.Build(grid, solid), BoundaryConditions3D.Closed,
            _ => rho, _ => 0);
        var fields = PlanarLiquidInventory.Create(plan, new D3(0, 0.015, 0), new D3(0, 1, 0), 0, 10, 0);
        var gasMass = Array.ConvertAll(fields.GasM3, volume => volume * rho);
        int left = plan.GroupByCell[grid.Cells.Index(0, 1, 0)];
        int right = plan.GroupByCell[grid.Cells.Index(1, 1, 0)];
        fields.VapourKg[left] = gasMass[left] * 0.02;
        fields.VapourKg[right] = gasMass[right] * 0.01;
        var areas = new CutCellVofTransport(plan).ReconstructGasFaceAreas(fields);
        var interfaceAreas = new CutCellVofTransport(plan).ReconstructLiquidGasInterfaceAreas(fields);
        if (Math.Abs(interfaceAreas[left] + interfaceAreas[right] - 2 * dx * openWidth) > 1e-12)
            throw new InvalidOperationException("The reconstructed interface area does not match the planar section.");
        double area = areas[plan.FaceIndex(0, 1, 1, 0)];
        if (Math.Abs(area - 0.5 * dx * openWidth) > 1e-12 || areas[plan.FaceIndex(0, 1, 0, 0)] != 0)
            throw new InvalidOperationException("The reconstructed aperture does not match the half-wetted face.");
        var vapourH = Array.ConvertAll(gasMass, _ => 2.5e6);
        var airH = new double[gasMass.Length];
        var result = HumidGasDiffusion.AdvanceReconstructed(plan, fields, dt, gasMass, vapourH, airH,
            _ => diffusivity);
        double expected = dt * rho * diffusivity * area / dx * 0.01;
        if (Math.Abs(fields.VapourKg[left] - result.VapourKg[left] - expected) > 1e-16 ||
            Math.Abs(result.VapourKg[right] - fields.VapourKg[right] - expected) > 1e-16)
            throw new InvalidOperationException("Vapour transfer did not use the reconstructed gas aperture.");
        Console.WriteLine(FormattableString.Invariant(
            $"S12 reconstructed gas diffusion passed: aperture {area:G8} m2, vapour transferred {expected:G8} kg."));
    }

    private static void CheckOpenGasDiffusion()
    {
        const double dx = 0.01, rho = 1.2, dt = 0.1, diffusivity = 2.5e-5;
        var grid = new Grid3D(1, 1, 1, new D3(dx, dx, dx), RigidFrame.Identity);
        double gasMass = rho * grid.CellVolume;
        foreach (bool positive in new[] { false, true })
        {
            var sides = new FlowBoundary[6];
            sides[positive ? 1 : 0] = new FlowBoundary(FlowBoundaryKind.OpenPressure);
            var plan = new VariableCoefficientProjection(CutCellGeometry.Build(grid), new BoundaryConditions3D(sides),
                _ => rho, _ => 0);
            var zero = new double[1];
            var fields = new CutCellInventories(zero, (double[])plan.Volumes.Clone(), zero,
                new[] { 1.0 }, new[] { 0.01 * gasMass });
            double boundaryFraction = positive ? 0.005 : 0.02;
            var update = HumidGasDiffusion.AdvanceOpen(plan, fields, dt, new[] { gasMass },
                new[] { 2.5e6 }, new[] { 20000.0 }, _ => diffusivity,
                _ => new HumidGasDiffusion.Reservoir(rho, boundaryFraction, 2.6e6, 30000));
            double expected = dt * rho * diffusivity * dx * 2 * (boundaryFraction - 0.01);
            double expectedHeat = expected * (expected > 0 ? 2.6e6 - 20000 : 2.5e6 - 30000);
            if (Math.Abs(update.VapourInKg - expected) > 1e-16 ||
                Math.Abs(update.EnthalpyInJ - expectedHeat) > 1e-9 ||
                Math.Abs(update.Fields.VapourKg[0] - fields.VapourKg[0] - update.VapourInKg) > 1e-16 ||
                Math.Abs(update.Fields.GasEnergyJ[0] - fields.GasEnergyJ[0] - update.EnthalpyInJ) > 1e-14)
                throw new InvalidOperationException("Open gas diffusion disagrees with the reservoir balance.");
            Console.WriteLine(FormattableString.Invariant(
                $"S12 open gas diffusion passed: vapour in {update.VapourInKg:G8} kg, enthalpy in {update.EnthalpyInJ:G8} J."));
        }
    }


    private static void CheckHumidDiffusion()
    {
        const double spacing = 0.01, rho = 1.2, dt = 0.2;
        var grid = new Grid3D(2, 1, 1, new D3(spacing, spacing, spacing), RigidFrame.Identity);
        var plan = new VariableCoefficientProjection(CutCellGeometry.Build(grid), BoundaryConditions3D.Closed, _ => rho, _ => 0);
        double mass = rho * grid.CellVolume;
        var zero = new double[2];
        var vapour = new[] { 0.02 * mass, 0.005 * mass };
        var energy = new[] { 2.0, 3.0 };
        var state = new CutCellInventories(zero, (double[])plan.Volumes.Clone(), zero, energy, vapour);
        var model = new HumidGas();
        double diffusivity = model.BinaryDiffusivityM2s(293.15, 101325, 0.0125);
        var vapourH = new[] { 2.5e6, 2.6e6 };
        var airH = new[] { 20000.0, 30000.0 };
        double Area(int index)
        {
            var face = plan.Faces[index];
            return face.Low >= 0 && face.High >= 0 ? face.Area : 0;
        }
        var next = HumidGasDiffusion.Advance(plan, state, dt, new[] { mass, mass }, vapourH, airH, Area, _ => diffusivity);
        double expected = dt * rho * diffusivity * spacing * (0.02 - 0.005);
        Near(next.VapourKg[0], vapour[0] - expected, 1e-15);
        Near(next.VapourKg[1], vapour[1] + expected, 1e-15);
        Near(next.GasEnergyJ[0], energy[0] - expected * (vapourH[0] - airH[1]), 1e-8);
        Near(next.GasEnergyJ[0] + next.GasEnergyJ[1], 5, 1e-14);
        try
        {
            HumidGasDiffusion.Advance(plan, state, 1000, new[] { mass, mass }, vapourH, airH, Area, _ => diffusivity);
        }
        catch (InvalidOperationException)
        {
            Console.WriteLine(FormattableString.Invariant(
                $"S12 humid diffusion passed: {vapour[0] - next.VapourKg[0]:G8} kg vapour transferred; equal dry-air counterflux, enthalpy conserved."));
            return;
        }
        throw new InvalidOperationException("The gas diffusion timestep guard did not reject an unstable step.");

        void Near(double value, double reference, double tolerance)
        {
            if (!(Math.Abs(value - reference) <= tolerance))
                throw new InvalidOperationException($"Humid diffusion expected {reference:G8}, got {value:G8}.");
        }
    }
}
