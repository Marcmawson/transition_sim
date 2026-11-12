using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Solids;

internal static class IceBodyMain
{
    private static void Main()
    {
        var body = IceBody.Box(917, new D3(0.04, 0.03, 0.02), RigidFrame.Identity);
        Near(body.MassKg, 917 * 0.04 * 0.03 * 0.02, 1e-15);
        double inertia = body.MassKg * (0.04 * 0.04 + 0.03 * 0.03) / 12;
        Near(body.PrincipalInertiaKgM2.Z, inertia, 1e-15);

        var forced = body.Advance(0.1, new D3(0, -9.81 * body.MassKg, 0), new D3(0, 0, inertia * 4));
        Near(forced.Frame.Origin.Y, -0.04905, 1e-12);
        Near(forced.VelocityMPerS.Y, -0.981, 1e-12);
        Near(forced.AngularVelocityRadPerS.Z, 0.4, 1e-12);
        Near(forced.Frame.U.Y, Math.Sin(0.02), 1e-12);
        Near(body.Frame.Origin.Length, 0, 0);

        var kicked = body.ApplyImpulse(new D3(0, 0.001, 0), new D3(0.01, 0, 0));
        Near(kicked.MomentumKgMPerS.Y, 0.001, 1e-15);
        Near(kicked.AngularMomentumKgM2PerS.Z, 1e-5, 1e-15);
        Near(kicked.VelocityAt(new D3(0.01, 0, 0)).Y,
            kicked.VelocityMPerS.Y + 0.01 * kicked.AngularVelocityRadPerS.Z, 1e-12);
        var drift = kicked;
        for (int i = 0; i < 1000; i++) drift = drift.Advance(0.001, default, default);
        Near((drift.MomentumKgMPerS - kicked.MomentumKgMPerS).Length, 0, 0);
        Near((drift.AngularMomentumKgM2PerS - kicked.AngularMomentumKgM2PerS).Length, 0, 0);
        Near(drift.KineticEnergyJ, kicked.KineticEnergyJ, 1e-12);
        Near(drift.Frame.U.Y, Math.Sin(kicked.AngularVelocityRadPerS.Z), 1e-12);
        var rotated = new IceBody(body.MassKg, body.PrincipalInertiaKgM2,
            RigidFrame.Euler(default, 0, 0, Math.PI / 2), default, new D3(0, 1e-5, 0));
        Near(rotated.AngularVelocityRadPerS.Y, 1e-5 / body.PrincipalInertiaKgM2.X, 1e-12);
        CheckWallImpact(body);
        CheckWallSweep(body);
        CheckRotatingSweep(body);
        CheckSimultaneousImpact(body);
        CheckSweptManifold(body);
        CheckRestingSupport(body);
        CheckBeakerImpact();
        Console.WriteLine("S11 rigid ice body passed: box inertia, force, torque, off-centre impulse, point velocity and free motion.");
    }

    private static void CheckBeakerImpact()
    {
        const int segments = 24;
        const double half = 0.005, impactTime = 0.005, restitution = 0.5;
        var glass = new GeometryInputs();
        double angle = Math.PI / segments;
        var outward = new D3(Math.Cos(angle), 0, Math.Sin(angle));
        var up = new D3(0, 1, 0);
        double floorY = glass.externalBaseHeightM + glass.baseThicknessM;
        double apothem = glass.innerRadiusM * Math.Cos(angle);
        var hull = new D3[8];
        for (int i = 0; i < hull.Length; i++)
            hull[i] = new D3((i & 1) == 0 ? -half : half,
                (i & 2) == 0 ? -half : half, (i & 4) == 0 ? -half : half);

        foreach (double downwardSpeed in new[] { 0.0, 0.4 })
        {
            double floorGap = downwardSpeed > 0 ? downwardSpeed * impactTime : half;
            var centre = outward * (apothem - half - 0.4 * impactTime) + up * (floorY + half + floorGap);
            var frame = RigidFrame.Euler(centre, -angle, 0, 0);
            Near((frame.U - outward).Length, 0, 1e-12);
            var box = IceBody.Box(917, new D3(2 * half, 2 * half, 2 * half), frame);
            var velocity = outward * 0.4 - up * downwardSpeed;
            var body = new IceBody(box.MassKg, box.PrincipalInertiaKgM2, frame, velocity * box.MassKg, default);
            var clear = IceWallContact.AdvanceAndResolveBeakerImpact(body, hull, glass, segments,
                RigidFrame.Identity, impactTime / 2, restitution, out D3 clearReaction);
            if (clear.Hit) throw new InvalidOperationException("The short beaker sweep should stay clear.");
            Near(clearReaction.Length, 0, 0);

            var hit = IceWallContact.AdvanceAndResolveBeakerImpact(body, hull, glass, segments,
                RigidFrame.Identity, 2 * impactTime, restitution, out D3 reaction);
            if (!hit.Hit) throw new InvalidOperationException("The beaker sweep missed the wall.");
            Near(hit.ElapsedSeconds, impactTime, 1e-12);
            Near((hit.Body.VelocityMPerS + velocity * restitution).Length, 0, 1e-10);
            Near(hit.Body.AngularVelocityRadPerS.Length, 0, 1e-8);
            Near((reaction + hit.Body.MomentumKgMPerS - body.MomentumKgMPerS).Length, 0, 1e-12);
            Near(hit.Body.KineticEnergyJ, body.KineticEnergyJ * restitution * restitution, 1e-12);
            foreach (D3 vertex in hull)
            {
                D3 point = hit.Body.Frame.ToWorld(vertex);
                if (point.Y < floorY - 1e-12) throw new InvalidOperationException("The ice crossed the beaker floor.");
                for (int i = 0; i < segments; i++)
                {
                    double facet = (2 * i + 1) * Math.PI / segments;
                    if (point.X * Math.Cos(facet) + point.Z * Math.Sin(facet) > apothem + 1e-12)
                        throw new InvalidOperationException("The ice crossed a beaker facet.");
                }
            }
        }
        Console.WriteLine("S11 V25 beaker impact passed: side-only and simultaneous side/floor at 5 ms.");
    }

    private static void CheckRestingSupport(IceBody box)
    {
        var hull = new D3[8];
        for (int i = 0; i < hull.Length; i++)
            hull[i] = new D3((i & 1) == 0 ? -0.02 : 0.02,
                (i & 2) == 0 ? -0.015 : 0.015, (i & 4) == 0 ? -0.01 : 0.01);
        var normal = new D3(0, 1, 0);
        var frame = RigidFrame.Euler(new D3(0, 0.015, 0), 0, 0, 0);
        double weight = box.MassKg * 9.81;
        const double dt = 0.001;
        foreach (double slide in new[] { 0.0, 0.1 })
        {
            var body = new IceBody(box.MassKg, box.PrincipalInertiaKgM2, frame,
                new D3(box.MassKg * slide, 0, 0), default);
            double reaction = 0;
            double initialEnergy = body.KineticEnergyJ;
            for (int step = 0; step < 1000; step++)
            {
                body = IceWallContact.AdvanceSupported(body, hull, default, normal, dt,
                    new D3(0, -weight, 0), default, out D3 impulse);
                reaction += impulse.Y;
                Near(impulse.Y / dt, -weight, 1e-9);
                Near(body.Frame.Origin.Y, frame.Origin.Y, 1e-10);
                Near(body.VelocityMPerS.Y, 0, 1e-10);
                Near(body.AngularVelocityRadPerS.Length, 0, 1e-8);
                foreach (D3 vertex in hull)
                    if (body.Frame.ToWorld(vertex).Y < -1e-10)
                        throw new InvalidOperationException("The supported hull penetrated the wall.");
            }
            Near(reaction, -weight, 1e-10);
            Near(body.Frame.Origin.X, slide, 1e-12);
            Near(body.KineticEnergyJ, initialEnergy, 1e-12);
        }

        var resting = new IceBody(box.MassKg, box.PrincipalInertiaKgM2, frame, default, default);
        var lifted = IceWallContact.AdvanceSupported(resting, hull, default, normal, dt,
            new D3(0, weight, 0), default, out D3 noReaction);
        Near(noReaction.Length, 0, 0);
        Near(lifted.VelocityMPerS.Y, 9.81 * dt, 1e-12);
        Console.WriteLine(FormattableString.Invariant(
            $"S11 V25 resting support passed: {weight:G8} N for 1 s, with frictionless sliding and lift-off."));
    }

    private static void CheckSweptManifold(IceBody box)
    {
        var hull = new D3[8];
        for (int i = 0; i < hull.Length; i++)
            hull[i] = new D3((i & 1) == 0 ? -0.02 : 0.02,
                (i & 2) == 0 ? -0.015 : 0.015, (i & 4) == 0 ? -0.01 : 0.01);
        const double incident = 20, restitution = 0.5, dt = 0.02;
        var body = new IceBody(box.MassKg, box.PrincipalInertiaKgM2,
            RigidFrame.Euler(new D3(0, 0.115, 0), 0, 0, 0),
            new D3(0.1, -incident, 0) * box.MassKg, default);
        var normal = new D3(0, 1, 0);
        foreach (bool reverse in new[] { false, true })
        {
            if (reverse) Array.Reverse(hull);
            var impact = IceWallContact.AdvanceAndResolvePlaneImpact(body, hull, default, normal,
                dt, restitution, out D3 reaction);
            if (!impact.Hit) throw new InvalidOperationException("Missed face-first impact.");
            Near(impact.ElapsedSeconds, 0.005, 1e-12);
            Near(impact.Body.Frame.Origin.Y, 0.015, 1e-12);
            Near(impact.Body.VelocityMPerS.Y, incident * restitution, 1e-10);
            Near(impact.Body.AngularVelocityRadPerS.Length, 0, 1e-8);
            Near(reaction.Y, -body.MassKg * incident * (1 + restitution), 1e-10);
            foreach (D3 vertex in hull)
            {
                D3 point = impact.Body.Frame.ToWorld(vertex);
                if (point.Y < -1e-12) throw new InvalidOperationException("Face-first impact penetrated the wall.");
                if (vertex.Y < 0) Near(impact.Body.VelocityAt(point).Y, incident * restitution, 2e-10);
            }
        }
        var clear = IceWallContact.AdvanceAndResolvePlaneImpact(body, hull, default, normal,
            0.002, restitution, out D3 noReaction);
        if (clear.Hit || noReaction.Length != 0)
            throw new InvalidOperationException("A clear sweep produced an impulse.");
        Console.WriteLine("S11 V25 swept face impact passed: all four corners rebounded at first contact.");
    }

    private static void CheckSimultaneousImpact(IceBody box)
    {
        const double incident = 0.4;
        var normal = new D3(0, 1, 0);
        var body = new IceBody(box.MassKg, box.PrincipalInertiaKgM2,
            RigidFrame.Euler(new D3(0, 0.015, 0), 0, 0, 0),
            new D3(0.1, -incident, 0) * box.MassKg, default);
        var corners = new[]
        {
            new D3(-0.02, 0, -0.01), new D3(0.02, 0, -0.01),
            new D3(-0.02, 0, 0.01), new D3(0.02, 0, 0.01)
        };
        foreach (double restitution in new[] { 0.0, 0.5, 1.0 })
        {
            var updated = IceWallContact.ResolveSimultaneousImpact(body, corners, normal, restitution, out D3 reaction);
            foreach (D3 corner in corners)
                Near(updated.VelocityAt(corner).Y, restitution * incident, 2e-10);
            Near(updated.AngularVelocityRadPerS.Length, 0, 1e-8);
            Near(updated.VelocityMPerS.X, 0.1, 1e-12);
            Near(reaction.Y, -body.MassKg * (1 + restitution) * incident, 1e-10);
            Near(body.KineticEnergyJ - updated.KineticEnergyJ,
                0.5 * body.MassKg * incident * incident * (1 - restitution * restitution), 1e-12);
            Array.Reverse(corners);
        }
        Console.WriteLine("S11 V25 four-corner impact passed: common rebound, zero spin, wall impulse and energy loss.");
    }

    private static void CheckRotatingSweep(IceBody box)
    {
        var vertices = new D3[8];
        for (int i = 0; i < vertices.Length; i++)
            vertices[i] = new D3((i & 1) == 0 ? -0.02 : 0.02,
                (i & 2) == 0 ? -0.015 : 0.015, (i & 4) == 0 ? -0.01 : 0.01);

        const double angleAtImpact = 0.2, spin = 10;
        double height = 0.02 * Math.Sin(angleAtImpact) + 0.015 * Math.Cos(angleAtImpact);
        var body = new IceBody(box.MassKg, box.PrincipalInertiaKgM2,
            RigidFrame.Euler(new D3(0, height, 0), 0, 0, 0), default,
            new D3(0, 0, spin * box.PrincipalInertiaKgM2.Z));
        double interval = 2 * Math.PI / spin;
        var end = body.Advance(interval, default, default);
        foreach (D3 vertex in vertices)
            if (body.Frame.ToWorld(vertex).Y <= 0 || end.Frame.ToWorld(vertex).Y <= 0)
                throw new InvalidOperationException("The rotating fixture must have clear endpoints.");

        var sweep = IceWallContact.AdvancePrincipalRotationToPlane(body, vertices, default, new D3(0, 1, 0), interval);
        if (!sweep.Hit) throw new InvalidOperationException("The rotating hull missed an interior impact.");
        Near(sweep.ElapsedSeconds, angleAtImpact / spin, 1e-9);
        Near(sweep.ContactPointM.Y, 0, 1e-10);
        foreach (D3 vertex in vertices)
            if (sweep.Body.Frame.ToWorld(vertex).Y < -1e-10)
                throw new InvalidOperationException("The spinning ice penetrated the plane.");
        var clear = IceWallContact.AdvancePrincipalRotationToPlane(body, vertices, default, new D3(0, 1, 0), 0.01);
        if (clear.Hit) throw new InvalidOperationException("Reported contact before the analytic time.");
        Near(clear.ElapsedSeconds, 0.01, 0);
        var glass = new GeometryInputs();
        double floor = glass.externalBaseHeightM + glass.baseThicknessM;
        var inBeaker = new IceBody(body.MassKg, body.PrincipalInertiaKgM2,
            RigidFrame.Euler(new D3(0, floor + height, 0), 0, 0, 0), default, body.AngularMomentumKgM2PerS);
        var beakerHit = IceWallContact.AdvanceAndResolveBeakerImpact(inBeaker, vertices, glass, 24,
            RigidFrame.Identity, interval, 0.5, out D3 wallReaction);
        if (!beakerHit.Hit || wallReaction.Y >= 0)
            throw new InvalidOperationException("The rotating ice missed the beaker floor impulse.");
        Near(beakerHit.ElapsedSeconds, angleAtImpact / spin, 1e-9);
        Near(beakerHit.ContactPointM.Y, floor, 1e-10);
        Near((wallReaction + beakerHit.Body.MomentumKgMPerS).Length, 0, 1e-12);
        foreach (D3 vertex in vertices)
            if (beakerHit.Body.Frame.ToWorld(vertex).Y < floor - 1e-10)
                throw new InvalidOperationException("The rotating ice penetrated the beaker floor.");

        const int segments = 24;
        double facetAngle = Math.PI / segments;
        double apothem = glass.innerRadiusM * Math.Cos(facetAngle);
        double reach = 0.02 * Math.Cos(angleAtImpact) + 0.015 * Math.Sin(angleAtImpact);
        var narrowHull = Array.ConvertAll(vertices, vertex => new D3(vertex.X, vertex.Y, vertex.Z * 0.4));
        var sideBox = IceBody.Box(917, new D3(0.04, 0.03, 0.008),
            RigidFrame.Euler(new D3(apothem - reach, floor + 0.04, 0), 0, 0, 0));
        var againstSide = new IceBody(sideBox.MassKg, sideBox.PrincipalInertiaKgM2, sideBox.Frame,
            default, new D3(0, 0, spin * sideBox.PrincipalInertiaKgM2.Z));
        var sideHit = IceWallContact.AdvanceAndResolveBeakerImpact(againstSide, narrowHull, glass, segments,
            RigidFrame.Euler(default, facetAngle, 0, 0), interval, 0.5, out D3 sideReaction);
        if (!sideHit.Hit || sideReaction.X <= 0)
            throw new InvalidOperationException("The rotating ice missed the beaker side impulse.");
        Near(sideHit.ElapsedSeconds, angleAtImpact / spin, 1e-9);
        Near(sideHit.ContactPointM.X, apothem, 1e-10);
        Near((sideReaction + sideHit.Body.MomentumKgMPerS).Length, 0, 1e-12);
        Console.WriteLine("S11 V25 rotating beaker impacts passed: separate floor and side hits at 20 ms between clear endpoints.");
        Console.WriteLine(FormattableString.Invariant(
            $"S11 V25 rotating sweep passed: impact at {sweep.ElapsedSeconds * 1000:F6} ms with both endpoints clear."));
    }

    private static void CheckWallSweep(IceBody box)
    {
        var vertices = new D3[8];
        for (int i = 0; i < vertices.Length; i++)
            vertices[i] = new D3((i & 1) == 0 ? -0.02 : 0.02,
                (i & 2) == 0 ? -0.015 : 0.015, (i & 4) == 0 ? -0.01 : 0.01);

        const double speed = 20, timestep = 0.02;
        var frame = RigidFrame.Euler(new D3(0, 0.12, 0), 0.2, 0.25, 0.15);
        var falling = new IceBody(box.MassKg, box.PrincipalInertiaKgM2, frame,
            new D3(0, -speed * box.MassKg, 0), default);
        var normal = new D3(0, 1, 0);
        double supportDepth = 0.02 * Math.Abs(frame.U.Y) + 0.015 * Math.Abs(frame.V.Y) + 0.01 * Math.Abs(frame.W.Y);
        double expectedTime = (frame.Origin.Y - supportDepth) / speed;
        var sweep = IceWallContact.AdvanceTranslationToPlane(falling, vertices, default, normal, timestep);
        if (!sweep.Hit) throw new InvalidOperationException("The fast ice drop missed the wall.");
        Near(sweep.ElapsedSeconds, expectedTime, 1e-12);
        Near(sweep.ContactPointM.Y, 0, 1e-12);
        foreach (D3 vertex in vertices)
            if (sweep.Body.Frame.ToWorld(vertex).Y < -1e-12)
                throw new InvalidOperationException("The ice penetrated the plane at first contact.");

        var rebound = IceWallContact.ResolveImpact(sweep.Body, sweep.ContactPointM, normal, 0.5, out _);
        Near(rebound.VelocityAt(sweep.ContactPointM).Y, 0.5 * speed, 1e-12);
        var shortSweep = IceWallContact.AdvanceTranslationToPlane(falling, vertices, default, normal, expectedTime / 2);
        if (shortSweep.Hit) throw new InvalidOperationException("Reported contact before the impact time.");
        Near(shortSweep.Body.Frame.Origin.Y, frame.Origin.Y - speed * expectedTime / 2, 1e-12);
        var immediate = IceWallContact.AdvanceTranslationToPlane(sweep.Body, vertices, default, normal, timestep);
        if (!immediate.Hit) throw new InvalidOperationException("Missed an approaching contact at the start of the step.");
        Near(immediate.ElapsedSeconds, 0, 1e-12);
        Console.WriteLine(FormattableString.Invariant(
            $"S11 V25 translation sweep passed: impact at {sweep.ElapsedSeconds * 1000:F6} ms within a {timestep * 1000:G} ms step."));
    }

    private static void CheckWallImpact(IceBody box)
    {
        const double incident = 0.4;
        var body = new IceBody(box.MassKg, box.PrincipalInertiaKgM2,
            RigidFrame.Euler(new D3(0, 0.015, 0), 0, 0, 0),
            new D3(0.1, -incident, 0) * box.MassKg, default);
        var normal = new D3(0, 1, 0);
        foreach (double x in new[] { 0.0, 0.02 })
        foreach (double restitution in new[] { 0.0, 0.5, 1.0 })
        {
            var point = new D3(x, 0, 0);
            var result = IceWallContact.ResolveImpact(body, point, normal, restitution, out D3 wallReaction);
            double effectiveMass = 1 / (1 / body.MassKg + x * x / body.PrincipalInertiaKgM2.Z);
            double impulse = effectiveMass * (1 + restitution) * incident;
            double loss = 0.5 * effectiveMass * (1 - restitution * restitution) * incident * incident;
            Near(result.VelocityAt(point).Y, restitution * incident, 1e-12);
            Near(result.VelocityMPerS.X, body.VelocityMPerS.X, 1e-12);
            Near(wallReaction.Y, -impulse, 1e-12);
            Near((result.MomentumKgMPerS + wallReaction - body.MomentumKgMPerS).Length, 0, 1e-12);
            Near(result.AngularMomentumKgM2PerS.Z, x * impulse, 1e-12);
            Near(body.KineticEnergyJ - result.KineticEnergyJ, loss, 1e-12);
        }

        var separating = new IceBody(body.MassKg, body.PrincipalInertiaKgM2, body.Frame,
            new D3(0, incident * body.MassKg, 0), default);
        var unchanged = IceWallContact.ResolveImpact(separating, default, normal, 1, out D3 reaction);
        if (unchanged != separating || reaction.Length != 0)
            throw new InvalidOperationException("A separating body must receive no wall impulse.");
        Console.WriteLine("S11 V25 point impact passed: central and off-centre impulses, restitution, wall reaction and energy loss.");
    }

    private static void Near(double actual, double expected, double tolerance)
    {
        if (!SiUnits.IsFinite(actual) || Math.Abs(actual - expected) > tolerance)
            throw new InvalidOperationException($"Expected {expected:G17}, got {actual:G17}.");
    }
}
