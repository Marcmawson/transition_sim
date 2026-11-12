using System;
using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Solids
{
    public static class IceWallContact
    {
        private const double RotatingSweepToleranceM = 1e-10;

        public readonly struct Sweep
        {
            public readonly IceBody Body;
            public readonly double ElapsedSeconds;
            public readonly bool Hit;
            public readonly D3 ContactPointM;

            internal Sweep(IceBody body, double elapsedSeconds, bool hit, D3 contactPointM)
            {
                Body = body;
                ElapsedSeconds = elapsedSeconds;
                Hit = hit;
                ContactPointM = contactPointM;
            }
        }

        // Exact time of first contact for a translating convex body and a
        // fixed plane. The caller resolves the contact at the returned pose.
        public static Sweep AdvanceTranslationToPlane(IceBody body, IReadOnlyList<D3> verticesLocalM,
            D3 planePointM, D3 planeNormal, double seconds)
        {
            if (body == null) throw new ArgumentNullException(nameof(body));
            if (verticesLocalM == null || verticesLocalM.Count == 0)
                throw new ArgumentException("A contact hull requires vertices.", nameof(verticesLocalM));
            if (!SiUnits.IsFinite(seconds) || seconds <= 0)
                throw new ArgumentOutOfRangeException(nameof(seconds));
            if (body.AngularMomentumKgM2PerS.Length != 0)
                throw new NotSupportedException("This plane sweep requires constant orientation.");

            D3 normal = planeNormal.Unit();
            double closest = double.PositiveInfinity;
            D3 support = default;
            foreach (D3 vertex in verticesLocalM)
            {
                D3 point = body.Frame.ToWorld(vertex);
                double distance = D3.Dot(point - planePointM, normal);
                if (distance >= closest) continue;
                closest = distance;
                support = point;
            }

            const double distanceToleranceM = 1e-12;
            if (closest < -distanceToleranceM)
                throw new InvalidOperationException("The ice hull starts behind the contact plane.");
            double speed = D3.Dot(body.VelocityMPerS, normal);
            double hitTime = speed < 0 ? Math.Max(0, closest) / -speed : double.PositiveInfinity;
            if (hitTime > seconds)
                return new Sweep(body.Advance(seconds, default, default), seconds, false, default);

            var atImpact = hitTime > 0 ? body.Advance(hitTime, default, default) : body;
            return new Sweep(atImpact, hitTime, true, support + body.VelocityMPerS * hitTime);
        }

        // Torque-free rotation about a principal axis keeps angular velocity
        // constant. A normal speed bound lets the sweep skip only clear time.
        public static Sweep AdvancePrincipalRotationToPlane(IceBody body, IReadOnlyList<D3> verticesLocalM,
            D3 planePointM, D3 planeNormal, double seconds)
        {
            if (body == null) throw new ArgumentNullException(nameof(body));
            if (verticesLocalM == null || verticesLocalM.Count == 0)
                throw new ArgumentException("A contact hull requires vertices.", nameof(verticesLocalM));
            if (!SiUnits.IsFinite(seconds) || seconds <= 0)
                throw new ArgumentOutOfRangeException(nameof(seconds));
            var omega = body.AngularVelocityRadPerS;
            if (omega.Length == 0)
                return AdvanceTranslationToPlane(body, verticesLocalM, planePointM, planeNormal, seconds);
            var angularMomentum = body.AngularMomentumKgM2PerS;
            if (D3.Cross(omega.Unit(), angularMomentum.Unit()).Length > 1e-12)
                throw new NotSupportedException("The rotating plane sweep requires principal-axis spin.");

            D3 normal = planeNormal.Unit();
            double radius = 0;
            foreach (D3 vertex in verticesLocalM) radius = Math.Max(radius, vertex.Length);
            double speedBound = Math.Abs(D3.Dot(body.VelocityMPerS, normal)) + omega.Length * radius;
            if (!SiUnits.IsFinite(speedBound))
                throw new ArgumentException("The hull speed bound must be finite.", nameof(body));
            const double toleranceM = RotatingSweepToleranceM;
            double elapsed = 0;
            for (int iteration = 0; iteration < 512; iteration++)
            {
                var pose = elapsed == 0 ? body : body.Advance(elapsed, default, default);
                double gap = double.PositiveInfinity;
                D3 support = default;
                foreach (D3 vertex in verticesLocalM)
                {
                    D3 point = pose.Frame.ToWorld(vertex);
                    double distance = D3.Dot(point - planePointM, normal);
                    if (distance >= gap) continue;
                    gap = distance;
                    support = point;
                }
                if (gap < -toleranceM)
                    throw new InvalidOperationException("The ice hull starts behind or crossed the contact plane.");
                if (gap <= toleranceM)
                    return new Sweep(pose, elapsed, true, support);
                if (elapsed == seconds)
                    return new Sweep(pose, seconds, false, default);

                double next = Math.Min(seconds, elapsed + gap / speedBound);
                if (next <= elapsed) break;
                elapsed = next;
            }
            throw new InvalidOperationException("The rotating plane sweep did not converge to a safe result.");
        }

        // Advance to the first impact, resolve all touching vertices, and
        // return the elapsed time so the caller can process the remainder.
        public static Sweep AdvanceAndResolvePlaneImpact(IceBody body, IReadOnlyList<D3> verticesLocalM,
            D3 planePointM, D3 planeNormal, double seconds, double restitution, out D3 wallReactionNs)
        {
            if (!SiUnits.IsFinite(restitution) || restitution < 0 || restitution > 1)
                throw new ArgumentOutOfRangeException(nameof(restitution));
            var sweep = AdvancePrincipalRotationToPlane(body, verticesLocalM, planePointM, planeNormal, seconds);
            wallReactionNs = default;
            if (!sweep.Hit) return sweep;

            D3 normal = planeNormal.Unit();
            double firstGap = D3.Dot(sweep.ContactPointM - planePointM, normal);
            var contacts = new List<D3>();
            foreach (D3 vertex in verticesLocalM)
            {
                D3 point = sweep.Body.Frame.ToWorld(vertex);
                double gap = D3.Dot(point - planePointM, normal);
                if (gap - firstGap <= RotatingSweepToleranceM) contacts.Add(point);
            }
            var rebound = ResolveSimultaneousImpact(sweep.Body, contacts, normal, restitution, out wallReactionNs);
            return new Sweep(rebound, sweep.ElapsedSeconds, true, sweep.ContactPointM);
        }

        // First impact for a non-spinning hull wholly inside the beaker below its rim.
        // The side planes coincide with the facets of ApparatusSurface.Beaker.
        public static Sweep AdvanceAndResolveBeakerTranslation(IceBody body, IReadOnlyList<D3> verticesLocalM,
            GeometryInputs glass, int segments, RigidFrame beakerFrame, double seconds, double restitution,
            out D3 wallReactionNs)
        {
            if (body == null) throw new ArgumentNullException(nameof(body));
            if (verticesLocalM == null || verticesLocalM.Count == 0)
                throw new ArgumentException("A contact hull requires vertices.", nameof(verticesLocalM));
            if (glass == null || segments < 3 || !SiUnits.IsFinite(glass.innerRadiusM) ||
                glass.innerRadiusM <= 0 || !SiUnits.IsFinite(glass.RimHeightM) ||
                !SiUnits.IsFinite(glass.externalBaseHeightM + glass.baseThicknessM) ||
                glass.cavityHeightM <= 0)
                throw new ArgumentException("A beaker requires a finite cavity and at least three facets.", nameof(glass));
            if (!SiUnits.IsFinite(seconds) || seconds <= 0)
                throw new ArgumentOutOfRangeException(nameof(seconds));
            if (!SiUnits.IsFinite(restitution) || restitution < 0 || restitution > 1)
                throw new ArgumentOutOfRangeException(nameof(restitution));
            if (body.AngularMomentumKgM2PerS.Length != 0)
                throw new NotSupportedException("The beaker translation sweep requires constant orientation.");

            double floorY = glass.externalBaseHeightM + glass.baseThicknessM;
            double apothem = glass.innerRadiusM * Math.Cos(Math.PI / segments);
            var planes = new (D3 Point, D3 Normal)[segments + 1];
            planes[0] = (beakerFrame.ToWorld(new D3(0, floorY, 0)), beakerFrame.V);
            for (int i = 0; i < segments; i++)
            {
                double angle = (2 * i + 1) * Math.PI / segments;
                D3 outward = beakerFrame.U * Math.Cos(angle) + beakerFrame.W * Math.Sin(angle);
                planes[i + 1] = (beakerFrame.Origin + outward * apothem, -outward);
            }

            var earliest = new Sweep(body, seconds, false, default);
            foreach (var plane in planes)
            {
                var candidate = AdvanceTranslationToPlane(body, verticesLocalM, plane.Point, plane.Normal, seconds);
                if (candidate.Hit && (!earliest.Hit || candidate.ElapsedSeconds < earliest.ElapsedSeconds))
                    earliest = candidate;
            }

            var atEnd = earliest.Hit ? earliest.Body : body.Advance(seconds, default, default);
            var points = new List<D3>();
            var normals = new List<D3>();
            foreach (D3 vertex in verticesLocalM)
            {
                D3 point = atEnd.Frame.ToWorld(vertex);
                if (beakerFrame.ToLocal(body.Frame.ToWorld(vertex)).Y > glass.RimHeightM ||
                    beakerFrame.ToLocal(point).Y > glass.RimHeightM)
                    throw new NotSupportedException("The beaker sweep requires the hull to stay below the rim.");
                if (!earliest.Hit) continue;
                foreach (var plane in planes)
                {
                    if (D3.Dot(point - plane.Point, plane.Normal) > RotatingSweepToleranceM) continue;
                    points.Add(point);
                    normals.Add(plane.Normal);
                }
            }
            wallReactionNs = default;
            if (!earliest.Hit) return new Sweep(atEnd, seconds, false, default);
            var rebound = ResolveContactManifold(atEnd, points, i => normals[i], restitution, out wallReactionNs);
            return new Sweep(rebound, earliest.ElapsedSeconds, true, earliest.ContactPointM);
        }

        // Apply the external impulse, resolve resting normal contact at
        // zero restitution, then drift with the constrained momentum.
        public static IceBody AdvanceSupported(IceBody body, IReadOnlyList<D3> verticesLocalM,
            D3 planePointM, D3 planeNormal, double seconds, D3 forceN, D3 torqueNm, out D3 wallReactionNs)
        {
            if (body == null) throw new ArgumentNullException(nameof(body));
            if (verticesLocalM == null || verticesLocalM.Count == 0)
                throw new ArgumentException("A contact hull requires vertices.", nameof(verticesLocalM));
            if (!SiUnits.IsFinite(seconds) || seconds <= 0)
                throw new ArgumentOutOfRangeException(nameof(seconds));

            const double distanceToleranceM = 1e-10;
            D3 normal = planeNormal.Unit();
            var contacts = new List<D3>();
            foreach (D3 vertex in verticesLocalM)
            {
                D3 point = body.Frame.ToWorld(vertex);
                double gap = D3.Dot(point - planePointM, normal);
                if (gap < -distanceToleranceM)
                    throw new InvalidOperationException("The ice hull starts behind the support plane.");
                if (gap <= distanceToleranceM) contacts.Add(point);
            }
            if (contacts.Count == 0)
                throw new InvalidOperationException("The supported step requires a touching hull.");

            var kicked = new IceBody(body.MassKg, body.PrincipalInertiaKgM2, body.Frame,
                body.MomentumKgMPerS + forceN * seconds, body.AngularMomentumKgM2PerS + torqueNm * seconds);
            var constrained = ResolveSimultaneousImpact(kicked, contacts, normal, 0, out D3 reaction);
            var updated = constrained.Advance(seconds, default, default);
            foreach (D3 vertex in verticesLocalM)
                if (D3.Dot(updated.Frame.ToWorld(vertex) - planePointM, normal) < -distanceToleranceM)
                    throw new InvalidOperationException("Supported drift penetrates the wall. A new impact solve or shorter step is required.");

            wallReactionNs = reaction;
            return updated;
        }

        // Contacts are supplied at the same impact pose. The restitution
        // targets are fixed from the incident state for the entire solve.
        public static IceBody ResolveSimultaneousImpact(IceBody body, IReadOnlyList<D3> pointsM,
            D3 wallNormal, double restitution, out D3 wallReactionNs)
        {
            if (body == null) throw new ArgumentNullException(nameof(body));
            if (pointsM == null || pointsM.Count == 0)
                throw new ArgumentException("At least one impact point is required.", nameof(pointsM));
            if (!SiUnits.IsFinite(restitution) || restitution < 0 || restitution > 1)
                throw new ArgumentOutOfRangeException(nameof(restitution));

            D3 normal = wallNormal.Unit();
            return ResolveContactManifold(body, pointsM, _ => normal, restitution, out wallReactionNs);
        }

        private static IceBody ResolveContactManifold(IceBody body, IReadOnlyList<D3> pointsM,
            Func<int, D3> normalAt, double restitution, out D3 wallReactionNs)
        {
            int count = pointsM.Count;
            var targets = new double[count];
            var inverseMass = new double[count];
            var impulses = new double[count];
            for (int i = 0; i < count; i++)
            {
                D3 normal = normalAt(i);
                targets[i] = -restitution * Math.Min(0, D3.Dot(body.VelocityAt(pointsM[i]), normal));
                inverseMass[i] = NormalInverseMass(body, pointsM[i], normal);
            }

            var trial = body;
            const double tolerance = 1e-12;
            for (int iteration = 0; iteration < 256; iteration++)
            {
                for (int i = 0; i < count; i++)
                {
                    D3 normal = normalAt(i);
                    double speed = D3.Dot(trial.VelocityAt(pointsM[i]), normal);
                    double next = Math.Max(0, impulses[i] + (targets[i] - speed) / inverseMass[i]);
                    double delta = next - impulses[i];
                    if (delta != 0) trial = trial.ApplyImpulse(normal * delta, pointsM[i]);
                    impulses[i] = next;
                }

                double residual = 0;
                for (int i = 0; i < count; i++)
                {
                    double speedError = D3.Dot(trial.VelocityAt(pointsM[i]), normalAt(i)) - targets[i];
                    residual = Math.Max(residual, impulses[i] > 0 ? Math.Abs(speedError) : Math.Max(0, -speedError));
                }
                if (residual > tolerance) continue;

                double energyTolerance = 1e-10 * body.KineticEnergyJ + 1e-15;
                if (trial.KineticEnergyJ - body.KineticEnergyJ > energyTolerance)
                    throw new InvalidOperationException("The simultaneous restitution solve increased kinetic energy.");
                wallReactionNs = default;
                for (int i = 0; i < count; i++) wallReactionNs -= normalAt(i) * impulses[i];
                return trial;
            }
            throw new InvalidOperationException("Simultaneous wall impact did not converge.");
        }

        // Resolve one already detected point of impact against a fixed,
        // frictionless wall. The normal points into the admissible region.
        public static IceBody ResolveImpact(IceBody body, D3 pointM, D3 wallNormal,
            double restitution, out D3 wallReactionNs)
        {
            if (body == null) throw new ArgumentNullException(nameof(body));
            if (!SiUnits.IsFinite(restitution) || restitution < 0 || restitution > 1)
                throw new ArgumentOutOfRangeException(nameof(restitution));
            D3 normal = wallNormal.Unit();
            double approach = D3.Dot(body.VelocityAt(pointM), normal);
            wallReactionNs = default;
            if (approach >= 0) return body;

            double inverseMass = NormalInverseMass(body, pointM, normal);
            D3 impulse = normal * (-(1 + restitution) * approach / inverseMass);
            var updated = body.ApplyImpulse(impulse, pointM);
            wallReactionNs = -impulse;
            return updated;
        }

        private static double NormalInverseMass(IceBody body, D3 pointM, D3 normal)
        {
            D3 arm = pointM - body.Frame.Origin;
            D3 angularImpulse = D3.Cross(arm, normal);
            double inverseMass = 1 / body.MassKg +
                D3.Dot(angularImpulse, body.InverseInertiaWorld(angularImpulse));
            if (!SiUnits.IsFinite(inverseMass) || inverseMass <= 0)
                throw new InvalidOperationException("The impact effective mass is invalid.");
            return inverseMass;
        }
    }
}
