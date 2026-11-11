using System;
using System.Linq;
using PhysicsSim.ThreePhase3D.Verification;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.PhaseChange;
using PhysicsSim.ThreePhase3D.Materials;

internal static class SurfaceGeometryMain
{
    private static int Main(string[] args)
    {
        if (args.Contains("--wetting")) return BeakerVofMain.Main(args);
        var grid = new Grid3D(15, 15, 15, new D3(.012, .008, .01));
        var transport = new GeometricVof3D(grid, 997, 1.2);
        var alpha = new double[transport.CellCount];
        int centre = transport.CellIndex(7, 7, 7);
        const double a = 3, b = 5;
        for (int curved = 0; curved <= 1; curved++)
        {
            for (int k = 0; k < 15; k++)
            for (int j = 0; j < 15; j++)
            for (int i = 0; i < 15; i++)
            {
                double x = (i - 7) * grid.Spacing.X, y = (j - 7) * grid.Spacing.Y;
                // Cell-area average of a quadratic height, exact across the
                // central 3x3 columns where the graph stays within one z cell.
                double height = .25 * grid.Spacing.Z - curved *
                    (a * (x * x + grid.Spacing.X * grid.Spacing.X / 12) +
                     b * (y * y + grid.Spacing.Y * grid.Spacing.Y / 12));
                alpha[transport.CellIndex(i, j, k)] =
                    Math.Max(0, Math.Min(1, height / grid.Spacing.Z - (k - 7) + .5));
            }
            Check(SurfaceGeometry.Evaluate(transport, alpha)[centre], 2 * curved * (a + b));
            for (int cell = 0; cell < alpha.Length; cell++) alpha[cell] = 1 - alpha[cell];
            Check(SurfaceGeometry.Evaluate(transport, alpha)[centre], -2 * curved * (a + b));
        }

        Array.Clear(alpha, 0, alpha.Length);
        alpha[centre] = .5;
        var unresolved = SurfaceGeometry.Evaluate(transport, alpha);
        if (unresolved[centre].Support != CurvatureSupport.Unresolved ||
            !double.IsNaN(unresolved[centre].InverseMetres) ||
            unresolved[0].Support != CurvatureSupport.PurePhase)
            throw new InvalidOperationException("Incorrect support diagnostic for an isolated mixed cell.");
        Console.WriteLine("Isolated mixed cell: unresolved; pure cell: pure phase.");
        CheckAlternateAxis();
        CheckSphericalDrop();
        CheckThermocapillarySlab();
        CheckWetting();
        double coarseContactError = CheckSessileDrop(16);
        if (!(CheckSessileDrop(24) < coarseContactError))
            throw new InvalidOperationException("Contact patch localisation did not improve under refinement.");
        CheckSessileSlipResponse();
        CheckSessileCapillaryPressure(90);
        CheckSessileCapillaryPressure(75);
        CheckSessileCapillaryPressure(105);
        CheckSessileCapillaryPressure(90, true);
        CheckSessileCapillaryPressure(90, true, true);
        return 0;
    }

    private static void CheckSphericalDrop()
    {
        const int n = 24, samples = 12;
        const double h = .001, radius = .007, sigma = .072, dt = .0001;
        var grid = new Grid3D(n, n, n, new D3(h, h, h));
        var transport = new GeometricVof3D(grid, 997, 1.2);
        var alpha = new double[transport.CellCount];
        for (int k = 0; k < n; k++)
        for (int j = 0; j < n; j++)
        for (int i = 0; i < n; i++)
        {
            int inside = 0;
            for (int z = 0; z < samples; z++)
            for (int y = 0; y < samples; y++)
            for (int x = 0; x < samples; x++)
            {
                double dx = (i + (x + .5) / samples - n / 2.0) * h;
                double dy = (j + (y + .5) / samples - n / 2.0) * h;
                double dz = (k + (z + .5) / samples - n / 2.0) * h;
                if (dx * dx + dy * dy + dz * dz < radius * radius) inside++;
            }
            alpha[transport.CellIndex(i, j, k)] = inside / (double)(samples * samples * samples);
        }

        var flow = new PeriodicVofProjection(grid, 997, 1.2);
        var state = VofChecks.Fields(grid, alpha, VofChecks.Uniform(alpha.Length, 0, 0, 0));
        var baseline = flow.Advance(state, dt);
        var disabled = flow.AdvanceCapillary(state, dt, 0);
        if (!baseline.CellPressurePa.SequenceEqual(disabled.CellPressurePa) ||
            !baseline.Fields.FaceVelocity.SelectMany(axis => axis)
                .SequenceEqual(disabled.Fields.FaceVelocity.SelectMany(axis => axis)))
            throw new InvalidOperationException("Zero surface tension changed the S05 projection.");
        Console.WriteLine("Zero surface tension: identical to the S05 projection.");

        double expected = 2 * sigma / radius;
        double initialVolume = alpha.Sum() * grid.CellVolume;
        for (int iteration = 1; iteration <= 3; iteration++)
        {
            var step = flow.AdvanceCapillary(state, dt, sigma, 16384);
            double jump = step.CellPressurePa[transport.CellIndex(n / 2, n / 2, n / 2)] - step.CellPressurePa[0];
            double speed = step.Fields.FaceVelocity.Max(axis => axis.Max(value => Math.Abs(value)));
            double drift = step.Fields.LiquidFraction.Sum() * grid.CellVolume - initialVolume;
            Console.WriteLine($"Drop step {iteration}: jump={jump:G8} Pa (expected {expected:G8}), max face speed={speed:G8} m/s, volume drift={drift:G8} m^3, extrapolated faces={step.ExtrapolatedCurvatureFaces}");
            if (!step.Pressure.Converged || !(Math.Abs(jump / expected - 1) <= .05) ||
                !(speed <= .0002) || !(Math.Abs(drift) <= initialVolume * 1e-10))
                throw new InvalidOperationException("Stationary sphere capillary check failed.");
            state = step.Fields;
        }
    }

    private static void CheckThermocapillarySlab()
    {
        const int n = 20;
        const double h = .001, dt = .0001, meanTemperature = 310, amplitude = 10;
        var grid = new Grid3D(n, n, 3, new D3(h, h, h));
        var transport = new GeometricVof3D(grid, 997, 1.2);
        var alpha = new double[transport.CellCount];
        var temperature = new double[alpha.Length];
        var sigma = new double[alpha.Length];
        var normalSigma = new double[alpha.Length];
        for (int k = 0; k < 3; k++)
        for (int j = 0; j < n; j++)
        for (int i = 0; i < n; i++)
        {
            int cell = transport.CellIndex(i, j, k);
            alpha[cell] = j == 5 || j == 14 ? .5 : j > 5 && j < 14 ? 1 : 0;
            temperature[cell] = meanTemperature + amplitude * Math.Sin(2 * Math.PI * i / n);
            sigma[cell] = WaterTransport.SurfaceTension(temperature[cell]);
            normalSigma[cell] = WaterTransport.SurfaceTension(meanTemperature + .5 * j);
        }

        var force = MarangoniStress.ForceDensity(transport, alpha, sigma);
        var normalForce = MarangoniStress.ForceDensity(transport, alpha, normalSigma);
        if (normalForce.Any(f => f.Length > 1e-10) || force.Any(f => f.Y != 0 || f.Z != 0))
            throw new InvalidOperationException("Marangoni force must be tangential.");
        double stress = 0;
        for (int j = 0; j < n / 2; j++) stress += h * force[transport.CellIndex(0, j, 0)].X;
        double expected = WaterTransport.SurfaceTensionDerivative(meanTemperature) * amplitude * 2 * Math.PI / (n * h);
        double netForce = force.Sum(f => f.X) * grid.CellVolume;

        var state = VofChecks.Fields(grid, alpha, VofChecks.Uniform(alpha.Length, 0, 0, 0));
        var solver = new PeriodicVofProjection(grid, 997, 1.2);
        var step = solver.AdvanceThermocapillary(state, dt, temperature);
        double coldwardSpeed = -step.Fields.FaceVelocity[0][transport.CellIndex(0, 4, 0)];
        var uniform = solver.AdvanceThermocapillary(state, dt,
            Enumerable.Repeat(meanTemperature, alpha.Length).ToArray());
        var isothermal = solver.AdvanceCapillary(state, dt, WaterTransport.SurfaceTension(meanTemperature));
        if (!uniform.CellPressurePa.SequenceEqual(isothermal.CellPressurePa) ||
            !uniform.Fields.FaceVelocity.SelectMany(axis => axis)
                .SequenceEqual(isothermal.Fields.FaceVelocity.SelectMany(axis => axis)))
            throw new InvalidOperationException("Uniform temperature changed the isothermal result.");

        Console.WriteLine($"Thermocapillary slab: stress={stress:G8} Pa (expected {expected:G8}), net force={netForce:G8} N, coldward speed={coldwardSpeed:G8} m/s");
        if (!(Math.Abs(stress / expected - 1) < .03) || !(Math.Abs(netForce) < 1e-16) ||
            !(coldwardSpeed > 0) || !step.Pressure.Converged)
            throw new InvalidOperationException("Thermocapillary slab check failed.");
    }

    private static void CheckWetting()
    {
        var profile = new WettingProfile("Synthetic flat wall", "Analytical test fixture", 65, .001, .01,
            new[] { new WettingMeasurement(0, 80), new WettingMeasurement(.02, 100) },
            new[] { new WettingMeasurement(0, 50), new WettingMeasurement(.02, 30) });
        if (profile.AngleDegrees(0, 70) != 70 || profile.AngleDegrees(0, 90) != 80 ||
            profile.AngleDegrees(.01, 65) != 90 || profile.AngleDegrees(-.01, 65) != 40)
            throw new InvalidOperationException("Wetting branches or hysteresis limits failed.");
        try
        {
            profile.AngleDegrees(.03, 65);
            throw new InvalidOperationException("An uncalibrated contact-line speed was accepted.");
        }
        catch (ArgumentOutOfRangeException) { }

        var grid = new Grid3D(4, 1, 1, new D3(1, 1, 1));
        var slab = SurfacePrimitives.Box(new D3(-1, -1, -1), new D3(5, .4, 2));
        var geometry = CutCellGeometry.Build(grid, slab);
        int cell = grid.Cells.Index(1, 0, 0);
        var patch = geometry.Patches.First(p => p.Cell == cell && p.FluidExposedAreaM2 > 0);
        var contact = new WettingContact(profile, patch, -.01);
        var normal = contact.InterfaceNormal(new D3(-1, 0, 0));
        double angle = Math.Acos(D3.Dot(normal, patch.NormalLocal)) * 180 / Math.PI;
        var traction = profile.WallTractionPa(new D3(2, 7, 0), patch.NormalLocal, .004, .01);
        if (Math.Abs(angle - 40) > 1e-12 || (traction - new D3(-4, 0, 0)).Length > 1e-12)
            throw new InvalidOperationException("Contact-angle orientation or Navier traction failed.");

        var open = new FlowBoundary(FlowBoundaryKind.OpenPressure);
        var wall = new FlowBoundary(FlowBoundaryKind.NoSlip);
        var plan = new VariableCoefficientProjection(geometry,
            new BoundaryConditions3D(open, open, wall, wall, wall, wall), _ => 997, _ => 0);
        var liquid = plan.Volumes.Select((v, i) => v * (.2 + .15 * i)).ToArray();
        var gas = plan.Volumes.Zip(liquid, (volume, water) => volume - water).ToArray();
        var state = new CutCellInventories(liquid, gas, new double[gas.Length],
            new double[gas.Length], new double[gas.Length]);
        var velocity = new double[plan.FaceCount];
        for (int i = 0; i <= 4; i++)
        {
            int f = plan.FaceIndex(0, i, 0, 0);
            velocity[f] = .06 / plan.Faces[f].Area;
        }
        var transport = new CutCellVofTransport(plan, donor => donor == cell ? contact : null);
        var result = transport.Advance(state, velocity, 1, _ => new CutCellInflow(0, 0, 0, 0));
        var clipped = new CutCellVofGeometry(geometry);
        int group = plan.GroupByCell[cell];
        double target = liquid[group] / plan.Volumes[group] * geometry.FluidVolumeM3[cell];
        var plane = clipped.Reconstruct(cell, normal, target);
        double swept = clipped.Sweep(cell, 0, true, .06, plane);
        double ledger = result.Fields.LiquidM3.Sum() - liquid.Sum() - result.BoundaryInflow[0];
        if (Math.Abs(clipped.LiquidVolume(cell, plane) - target) > 1e-10 ||
            Math.Abs(result.LiquidFaceM3[plan.FaceIndex(0, 2, 0, 0)] - swept) > 1e-10 ||
            Math.Abs(ledger) > 1e-10)
            throw new InvalidOperationException("Wetting reconstruction or transport ledger failed.");
        var tracked = CutCellVofTransport.WithTrackedWetting(plan, profile);
        var slow = velocity.Select(value => value / 10).ToArray();
        var detected = tracked.TrackContact(cell, group, liquid[group], new D3(-1, 0, 0), slow);
        var separated = tracked.TrackContact(cell, group, liquid[group], new D3(0, -1, 0), slow);
        if (detected == null || Math.Abs(detected.SignedSpeedMPerS + .01) > 1e-8 || separated != null)
            throw new InvalidOperationException("Contact tracking must distinguish an intersecting plane from a separated interface.");
        var autoResult = tracked.Advance(state, slow, 10, _ => new CutCellInflow(0, 0, 0, 0));
        int outlet = plan.FaceIndex(0, 2, 0, 0);
        if (Math.Abs(autoResult.LiquidFaceM3[outlet] - result.LiquidFaceM3[outlet]) > 1e-8)
            throw new InvalidOperationException("Tracked wetting did not reproduce the prescribed contact flux.");
        Console.WriteLine($"Tracked wetting: receding speed={detected.SignedSpeedMPerS:G8} m/s; separated interface excluded.");
        Console.WriteLine($"Synthetic wetting: receding angle={angle:G8}°, traction={traction.X:G8} Pa, liquid ledger={ledger:G8} m^3");
    }

    private static double CheckSessileDrop(int n)
    {
        const double extent = .024, wallY = .0013, radius = .006, speed = .01;
        const double angle = 60;
        double h = extent / n;
        double contactRadius = radius * Math.Sin(angle * Math.PI / 180);
        var grid = new Grid3D(n, n / 2, n, new D3(h, h, h));
        var wall = SurfacePrimitives.Box(new D3(-h, -h, -h), new D3(extent + h, wallY, extent + h));
        var geometry = CutCellGeometry.Build(grid, wall);
        var plan = new VariableCoefficientProjection(geometry, BoundaryConditions3D.Closed, _ => 997, _ => 0);
        var profile = new WettingProfile("Synthetic flat wall", "Translating spherical-cap fixture", angle, .0001, h,
            new[] { new WettingMeasurement(0, 65), new WettingMeasurement(.02, 75) },
            new[] { new WettingMeasurement(0, 55), new WettingMeasurement(.02, 45) });
        var solver = CutCellVofTransport.WithTrackedWetting(plan, profile);
        var velocity = new double[plan.FaceCount];
        for (int f = 0; f < velocity.Length; f++)
            if (plan.Faces[f].Active != 0 && plan.Faces[f].Axis == 0) velocity[f] = speed;

        // Two independently sampled snapshots of a prescribed translating cap.
        // The second snapshot is shifted by 1 mm, equivalent to 0.1 s at speed.
        double maximumRadialError = 0;
        foreach (double xCentre in new[] { .0115, .0125 })
        {
            var centre = new D3(xCentre, wallY - radius * .5, .012);
            var liquid = SampleSessileCap(plan, wallY, centre, radius);

            int advancing = 0, receding = 0, wrongSign = 0, contacts = 0;
            double radialError = 0, angleError = 0;
            foreach (int cell in geometry.Patches.Select(p => p.Cell).Distinct())
            {
                int group = plan.GroupByCell[cell];
                if (group < 0 || liquid[group] <= 0 || liquid[group] >= plan.Volumes[group]) continue;
                var normal = solver.EstimateNormal(cell, group, liquid);
                var contact = solver.TrackContact(cell, group, liquid[group], normal, velocity);
                if (contact == null) continue;
                var p = contact.Wall.CentroidLocalM - centre;
                double r = Math.Sqrt(p.X * p.X + p.Z * p.Z);
                radialError = Math.Max(radialError, Math.Abs(r - contactRadius));
                if (p.X > h && contact.SignedSpeedMPerS > 0) advancing++;
                if (p.X < -h && contact.SignedSpeedMPerS < 0) receding++;
                if (Math.Abs(p.X) > h && p.X * contact.SignedSpeedMPerS <= 0) wrongSign++;
                double imposed = Math.Acos(D3.Dot(contact.InterfaceNormal(normal), contact.Wall.NormalLocal)) * 180 / Math.PI;
                double cosine = D3.Dot(normal.Unit(), contact.Wall.NormalLocal);
                double apparent = Math.Acos(Math.Max(-1, Math.Min(1, cosine))) * 180 / Math.PI;
                double target = profile.AngleDegrees(contact.SignedSpeedMPerS, apparent);
                angleError = Math.Max(angleError, Math.Abs(imposed - target));
                contacts++;
            }
            Console.WriteLine($"Sessile cap {n} cells, centre={xCentre:G4} m: contacts={contacts}, advancing={advancing}, receding={receding}, wrong sign={wrongSign}, radius error={radialError:G5} m, angle error={angleError:G5} deg");
            if (advancing == 0 || receding == 0 || wrongSign != 0 || !(radialError <= h) || !(angleError <= 1e-10))
                throw new InvalidOperationException("Synthetic sessile-cap tracking check failed.");
            maximumRadialError = Math.Max(maximumRadialError, radialError);
        }
        return maximumRadialError;
    }

    private static double[] SampleSessileCap(VariableCoefficientProjection plan, double wallY, D3 centre, double radius)
    {
        var grid = plan.Grid;
        double h = grid.Spacing.X;
        var liquid = new double[plan.GroupCount];
        var shape = grid.Cells;
        for (int k = 0; k < shape.Z; k++)
        for (int j = 0; j < shape.Y; j++)
        for (int i = 0; i < shape.X; i++)
        {
            int cell = shape.Index(i, j, k), group = plan.GroupByCell[cell];
            if (group < 0) continue;
            var lower = grid.Lower(i, j, k);
            double yLow = Math.Max(wallY, lower.Y), height = lower.Y + h - yLow;
            int inside = 0;
            const int samples = 10;
            for (int z = 0; z < samples; z++)
            for (int y = 0; y < samples; y++)
            for (int x = 0; x < samples; x++)
            {
                var p = new D3(lower.X + h * (x + .5) / samples, yLow + height * (y + .5) / samples,
                    lower.Z + h * (z + .5) / samples) - centre;
                if (D3.Dot(p, p) < radius * radius) inside++;
            }
            liquid[group] += plan.Geometry.FluidVolumeM3[cell] * (inside / (double)(samples * samples * samples));
        }
        return liquid;
    }

    private static void CheckSessileSlipResponse()
    {
        const int n = 16;
        const double h = .0015, wallY = .0013, radius = .006, dt = .001;
        const double rhoL = 997, rhoG = 1.2, muL = .001, muG = .000018;
        var grid = new Grid3D(n, n / 2, n, new D3(h, h, h));
        var wall = SurfacePrimitives.Box(new D3(-h, -h, -h), new D3((n + 1) * h, wallY, (n + 1) * h));
        var geometry = CutCellGeometry.Build(grid, wall);
        var plan = new VariableCoefficientProjection(geometry, BoundaryConditions3D.Closed, _ => rhoL, _ => 0);
        var liquid = SampleSessileCap(plan, wallY, new D3(.0115, wallY - radius * .5, .012), radius);
        var gas = plan.Volumes.Zip(liquid, (volume, water) => volume - water).ToArray();
        var initial = new CutCellInventories(liquid, gas, new double[gas.Length], new double[gas.Length], new double[gas.Length]);

        // Four face fluxes close around a loop crossing the cap's footprint.
        var seed = new float[plan.FaceCount];
        int[] loop = { plan.FaceIndex(0, 11, 0, 7), plan.FaceIndex(1, 11, 1, 7),
            plan.FaceIndex(0, 11, 1, 7), plan.FaceIndex(1, 10, 1, 7) };
        for (int i = 0; i < loop.Length; i++)
        {
            if (plan.Faces[loop[i]].Active == 0)
                throw new InvalidOperationException("The sessile fixture circulation crosses a blocked face.");
            seed[loop[i]] = (float)((i < 2 ? 1 : -1) * 1e-9 / plan.Faces[loop[i]].Area);
        }
        var initialFlow = FlowCpuReference.Project(plan, seed, dt, 16384);
        double shortImpulse = Run(.00005);
        double longImpulse = Run(.0005);
        if (!(shortImpulse > longImpulse && longImpulse > 0))
            throw new InvalidOperationException("A larger slip length must reduce the first-step wall impulse in the fixture.");

        double Run(double slip)
        {
            var profile = new WettingProfile("Synthetic planar wall", "Sessile cap slip sensitivity", 60, slip, h,
                new[] { new WettingMeasurement(0, 65), new WettingMeasurement(.02, 75) },
                new[] { new WettingMeasurement(0, 55), new WettingMeasurement(.02, 45) });
            var solver = CutCellVofTransport.WithTrackedWetting(plan, profile);
            var fields = initial;
            var flow = initialFlow;
            double firstImpulse = 0, moved = 0;
            int patches = 0;
            for (int step = 0; step < 3; step++)
            {
                var trial = solver.AdvanceCoupledWithWetting(fields, flow, rhoL, rhoG, muL, muG, maxIterations: 16384);
                fields = trial.Transport.Fields;
                flow = trial.Pressure;
                patches += trial.Momentum.ContactTractionPatches;
                if (step == 0) firstImpulse = trial.Momentum.WallImpulseKgMPerS.Sum(Math.Abs);
                if (Math.Abs(fields.LiquidM3.Sum() - liquid.Sum()) > 1e-15 ||
                    trial.Momentum.MaxDualMassErrorKg > 1e-12 || !flow.Diagnostics.Converged)
                    throw new InvalidOperationException("Sessile cap volume, mass or pressure check failed.");
                moved = fields.LiquidM3.Zip(liquid, (current, prior) => Math.Abs(current - prior)).Sum();
            }
            if (patches == 0 || !(moved > 1e-13))
                throw new InvalidOperationException("The sessile cap must move and apply contact traction.");
            Console.WriteLine($"Evolving sessile cap: slip={slip:G5} m, first wall impulse={firstImpulse:G8} kg m/s, contact patches={patches}, liquid redistribution={moved:G8} m^3");
            return firstImpulse;
        }
    }

    private static void CheckSessileCapillaryPressure(double angle, bool dynamicWetting = false, bool bulkViscosity = false)
    {
        const int n = 24, firstFluidY = 1;
        const double h = .001, wallY = h, radius = .007, sigma = .072, dt = .0001;
        const double rhoL = 997, rhoG = 1.2;
        var grid = new Grid3D(n, 14, n, new D3(h, h, h));
        var slab = SurfacePrimitives.Box(new D3(-h, -h, -h), new D3((n + 1) * h, wallY, (n + 1) * h));
        var geometry = CutCellGeometry.Build(grid, slab);
        var plan = new VariableCoefficientProjection(geometry, BoundaryConditions3D.Closed, _ => rhoG, _ => 0);
        var profile = new WettingProfile("Synthetic planar wall", "Dynamic capillary fixture", 90, .0001, h,
            new[] { new WettingMeasurement(0, 90.5), new WettingMeasurement(.02, 91) },
            new[] { new WettingMeasurement(0, 89.5), new WettingMeasurement(.02, 89) });
        var solver = dynamicWetting ? CutCellVofTransport.WithPlanarWallWetting(plan, firstFluidY, sigma, profile, bulkViscosity) :
            CutCellVofTransport.WithPlanarWallCapillarity(plan, firstFluidY, sigma, angle);
        var liquid = SampleSessileCap(plan, wallY, new D3(.012, wallY - radius * Math.Cos(angle * Math.PI / 180), .012), radius);
        var gas = plan.Volumes.Zip(liquid, (volume, water) => volume - water).ToArray();
        var state = new CutCellInventories(liquid, gas, new double[gas.Length], new double[gas.Length], new double[gas.Length]);
        if (bulkViscosity)
        {
            CheckRigidRotation(plan, state, rhoL, rhoG, dt);
            CheckPlanarNoSlip(plan, rhoL, rhoG, dt);
        }
        var seed = new float[plan.FaceCount];
        if (dynamicWetting)
        {
            int[] loop = { plan.FaceIndex(0, 19, 1, 11), plan.FaceIndex(1, 19, 2, 11),
                plan.FaceIndex(0, 19, 2, 11), plan.FaceIndex(1, 18, 2, 11) };
            for (int i = 0; i < loop.Length; i++)
                seed[loop[i]] = (float)((i < 2 ? 1 : -1) * 1e-10 / plan.Faces[loop[i]].Area);
        }
        var flow = FlowCpuReference.Project(plan, seed, dt, 16384);
        int core = plan.GroupByCell[grid.Cells.Index(n / 2, firstFluidY, n / 2)];
        int ambient = plan.GroupByCell[grid.Cells.Index(0, firstFluidY, 0)];
        double expected = 2 * sigma / radius;
        double speedLimit = angle == 90 && !dynamicWetting ? .00025 : .005 * Math.Sqrt(sigma / (rhoL * radius));
        for (int step = 1; step <= 2; step++)
        {
            var trial = dynamicWetting
                ? solver.AdvanceCoupledWithWetting(state, flow, rhoL, rhoG, .001, .000018, maxIterations: 16384)
                : solver.AdvanceCoupled(state, flow, rhoL, rhoG, maxIterations: 16384);
            if (dynamicWetting && (trial.CapillaryWettingContacts == 0 || trial.Momentum.ContactTractionPatches == 0))
                throw new InvalidOperationException("The dynamic capillary fixture must apply wetting and contact traction.");
            if (dynamicWetting && !bulkViscosity && step == 1)
            {
                var wettingOnly = CutCellVofTransport.WithTrackedWetting(plan, profile)
                    .AdvanceCoupledWithWetting(state, flow, rhoL, rhoG, .001, .000018, maxIterations: 16384);
                double jumpWithoutCapillarity = wettingOnly.Pressure.GroupPressurePa[core] -
                    wettingOnly.Pressure.GroupPressurePa[ambient];
                if (!(Math.Abs(jumpWithoutCapillarity) < 1) ||
                    !wettingOnly.Momentum.VelocityMPerS.SequenceEqual(trial.Momentum.VelocityMPerS))
                    throw new InvalidOperationException("The comparison must isolate the capillary pressure increment.");
                Console.WriteLine($"Wetting-only comparison pressure jump={jumpWithoutCapillarity:G8} Pa");
            }
            if (bulkViscosity)
            {
                double loss = trial.Momentum.BulkViscousEnergyLossJ;
                if (!(loss > 0))
                    throw new InvalidOperationException("Bulk viscosity must dissipate kinetic energy.");
                Console.WriteLine($"Viscous sessile cap step {step}: bulk energy loss={loss:E5} J");
                if (step == 1)
                {
                    var reference = CutCellVofTransport.WithPlanarWallWetting(plan, firstFluidY, sigma, profile)
                        .AdvanceCoupledWithWetting(state, flow, rhoL, rhoG, .001, .000018, maxIterations: 16384);
                    var total = new double[3];
                    for (int f = 0; f < plan.FaceCount; f++)
                        total[plan.Faces[f].Axis] += trial.Momentum.MomentumKgMPerS[f] -
                            reference.Momentum.MomentumKgMPerS[f];
                    for (int axis = 0; axis < 3; axis++)
                        total[axis] -= trial.Momentum.WallImpulseKgMPerS[axis] -
                            reference.Momentum.WallImpulseKgMPerS[axis];
                    if (total.Any(value => Math.Abs(value) > 1e-20))
                        throw new InvalidOperationException("Viscous momentum change must match the wall impulse.");
                    Console.WriteLine($"Viscous momentum ledger residual={total.Max(value => Math.Abs(value)):E5} kg m/s");
                }
            }
            flow = trial.Pressure;
            state = trial.Transport.Fields;
            double jump = flow.GroupPressurePa[core] - flow.GroupPressurePa[ambient];
            double speed = flow.VelocityMPerS.Max(value => Math.Abs(value));
            double drift = state.LiquidM3.Sum() - liquid.Sum();
            Console.WriteLine($"{angle:G}-degree sessile cap step {step}: jump={jump:G8} Pa (expected {expected:G8}), max face speed={speed:G8} m/s, drift={drift:E4} m^3, dynamic contacts={trial.CapillaryWettingContacts}");
            if (!flow.Diagnostics.Converged || !(Math.Abs(jump / expected - 1) <= .05) ||
                !(speed <= speedLimit) || !(Math.Abs(drift) <= 1e-15))
                throw new InvalidOperationException("Wall-compatible capillary pressure check failed.");
        }
    }

    private static void CheckPlanarNoSlip(VariableCoefficientProjection plan, double rhoL, double rhoG, double dt)
    {
        const double speed = .01, viscosity = .001;
        var state = new CutCellInventories((double[])plan.Volumes.Clone(), new double[plan.GroupCount],
            new double[plan.GroupCount], new double[plan.GroupCount], new double[plan.GroupCount]);
        var velocity = new double[plan.FaceCount];
        for (int f = 0; f < velocity.Length; f++)
            if (plan.Faces[f].Active != 0 && plan.Faces[f].Axis == 0) velocity[f] = speed;
        var transport = new CutCellVofTransport(plan);
        var stationary = transport.Advance(state, new double[plan.FaceCount], dt);
        var momentum = new CutCellMomentum(plan);
        var initial = momentum.Advance(state, stationary, velocity, rhoL, rhoG);
        var stressed = momentum.ApplyContactTraction(initial, state,
            new System.Collections.Generic.List<InterfacePatch>(), null, rhoL, rhoG, viscosity, .000018, dt,
            planarNoSlip: true);
        double area = plan.Grid.Cells.X * plan.Grid.Cells.Z * plan.Grid.FaceArea(1);
        double expected = -dt * viscosity * area * speed / (.5 * plan.Grid.Spacing.Y);
        double actual = stressed.WallImpulseKgMPerS[0];
        if (!(Math.Abs(actual / expected - 1) <= 1e-9) ||
            stressed.ContactTractionPatches != 0 || stressed.WallImpulseKgMPerS[1] != 0 ||
            stressed.WallImpulseKgMPerS[2] != 0)
            throw new InvalidOperationException("Planar no-slip wall impulse differs from the analytical plug result.");
        Console.WriteLine($"Planar no-slip plug: wall impulse={actual:E5} kg m/s, expected={expected:E5} kg m/s");
    }

    private static void CheckRigidRotation(VariableCoefficientProjection plan, CutCellInventories state,
        double rhoL, double rhoG, double dt)
    {
        var velocity = new double[plan.FaceCount];
        for (int f = 0; f < velocity.Length; f++)
        {
            var face = plan.Faces[f];
            if (face.Active == 0 || face.Axis == 2) continue;
            plan.Grid.Shape((GridLocation)(face.Axis + 1)).Coordinates(f - plan.FaceOffsets[face.Axis],
                out int i, out int j, out _);
            // A rigid rotation has nonzero cross derivatives that cancel in shear.
            velocity[f] = face.Axis == 0 ? -(j + .5) * plan.Grid.Spacing.Y :
                (i + .5) * plan.Grid.Spacing.X;
        }
        var transport = new CutCellVofTransport(plan);
        var stationary = transport.Advance(state, new double[plan.FaceCount], dt);
        var momentum = new CutCellMomentum(plan);
        var initial = momentum.Advance(state, stationary, velocity, rhoL, rhoG);
        var stressed = momentum.ApplyPlanarBulkViscosity(initial, state, rhoL, rhoG, .001, .000018, dt);
        double impulse = stressed.MomentumKgMPerS.Zip(initial.MomentumKgMPerS, (a, b) => Math.Abs(a - b)).Max();
        if (!(impulse <= 1e-23))
            throw new InvalidOperationException("Symmetric viscous stress must vanish under rigid rotation.");
        Console.WriteLine($"Rigid rotation maximum bulk impulse={impulse:E5} kg m/s");
    }

    private static void CheckAlternateAxis()
    {
        // The z axis has the strongest fraction gradient but has only five
        // cells. The same inclined plane is a valid height function along x.
        var grid = new Grid3D(11, 5, 5, new D3(.01, .01, .01));
        var transport = new GeometricVof3D(grid, 997, 1.2);
        var alpha = new double[transport.CellCount];
        for (int k = 0; k < 5; k++)
        for (int j = 0; j < 5; j++)
        for (int i = 0; i < 11; i++)
        {
            double offset = 1.5 * (k - 2) - (i - 5) + .5;
            alpha[transport.CellIndex(i, j, k)] =
                (ClampedIntegral(offset + .75) - ClampedIntegral(offset - .75)) / 1.5;
        }
        var sample = SurfaceGeometry.Evaluate(transport, alpha)[transport.CellIndex(5, 2, 2)];
        if (sample.Support != CurvatureSupport.AlternateAxis || !(Math.Abs(sample.InverseMetres) <= 1e-10))
            throw new InvalidOperationException("The inclined plane should use the alternate axis and have zero curvature.");
        Console.WriteLine("Inclined plane: alternate axis; zero curvature.");
    }

    private static double ClampedIntegral(double x)
    {
        if (x <= 0) return 0;
        if (x >= 1) return x - .5;
        return .5 * x * x;
    }

    private static void Check(SurfaceCurvature actual, double expected)
    {
        if (actual.Support != CurvatureSupport.HeightFunction ||
            !(Math.Abs(actual.InverseMetres - expected) <= 1e-10))
            throw new InvalidOperationException($"Expected curvature {expected:R}, got {actual.InverseMetres:R} ({actual.Support}).");
        Console.WriteLine($"Curvature {actual.InverseMetres:G8} 1/m (expected {expected:G8})");
    }
}
