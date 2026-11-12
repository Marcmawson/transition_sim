using System;
using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Solids
{
    public static class IceWallContact
    {
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

            D3 arm = pointM - body.Frame.Origin;
            D3 angularImpulsePerNs = D3.Cross(arm, normal);
            double inverseMass = 1 / body.MassKg +
                D3.Dot(angularImpulsePerNs, body.InverseInertiaWorld(angularImpulsePerNs));
            if (!SiUnits.IsFinite(inverseMass) || inverseMass <= 0)
                throw new InvalidOperationException("The impact effective mass is invalid.");
            D3 impulse = normal * (-(1 + restitution) * approach / inverseMass);
            var updated = body.ApplyImpulse(impulse, pointM);
            wallReactionNs = -impulse;
            return updated;
        }
    }
}
