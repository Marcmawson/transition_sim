using System;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Solids
{
    public static class IceWallContact
    {
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
