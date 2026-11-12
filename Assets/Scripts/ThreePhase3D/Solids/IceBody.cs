using System;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Solids
{
    // Constant-mass rigid ice state. The frame origin is the centre of mass
    // and its axes are the principal axes of inertia.
    public sealed class IceBody
    {
        public double MassKg { get; }
        public D3 PrincipalInertiaKgM2 { get; }
        public RigidFrame Frame { get; }
        public D3 MomentumKgMPerS { get; }
        public D3 AngularMomentumKgM2PerS { get; }

        public IceBody(double massKg, D3 principalInertiaKgM2, RigidFrame frame,
            D3 momentumKgMPerS, D3 angularMomentumKgM2PerS)
        {
            if (!SiUnits.IsFinite(massKg) || massKg <= 0)
                throw new ArgumentOutOfRangeException(nameof(massKg));
            if (principalInertiaKgM2.X <= 0 || principalInertiaKgM2.Y <= 0 || principalInertiaKgM2.Z <= 0)
                throw new ArgumentOutOfRangeException(nameof(principalInertiaKgM2));
            MassKg = massKg;
            PrincipalInertiaKgM2 = principalInertiaKgM2;
            Frame = frame ?? throw new ArgumentNullException(nameof(frame));
            MomentumKgMPerS = momentumKgMPerS;
            AngularMomentumKgM2PerS = angularMomentumKgM2PerS;
        }

        public static IceBody Box(double densityKgM3, D3 dimensionsM, RigidFrame frame)
        {
            if (!SiUnits.IsFinite(densityKgM3) || densityKgM3 <= 0)
                throw new ArgumentOutOfRangeException(nameof(densityKgM3));
            if (dimensionsM.X <= 0 || dimensionsM.Y <= 0 || dimensionsM.Z <= 0)
                throw new ArgumentOutOfRangeException(nameof(dimensionsM));
            double mass = densityKgM3 * dimensionsM.X * dimensionsM.Y * dimensionsM.Z;
            double xx = dimensionsM.X * dimensionsM.X, yy = dimensionsM.Y * dimensionsM.Y, zz = dimensionsM.Z * dimensionsM.Z;
            var inertia = new D3(mass * (yy + zz) / 12, mass * (xx + zz) / 12, mass * (xx + yy) / 12);
            return new IceBody(mass, inertia, frame, default, default);
        }

        public D3 VelocityMPerS => MomentumKgMPerS / MassKg;
        public D3 AngularVelocityRadPerS => AngularVelocity(Frame, AngularMomentumKgM2PerS);
        public double KineticEnergyJ => 0.5 * (D3.Dot(MomentumKgMPerS, VelocityMPerS) +
            D3.Dot(AngularMomentumKgM2PerS, AngularVelocityRadPerS));

        public D3 VelocityAt(D3 worldPointM) =>
            VelocityMPerS + D3.Cross(AngularVelocityRadPerS, worldPointM - Frame.Origin);

        public IceBody ApplyImpulse(D3 impulseNs, D3 worldPointM) =>
            new IceBody(MassKg, PrincipalInertiaKgM2, Frame, MomentumKgMPerS + impulseNs,
                AngularMomentumKgM2PerS + D3.Cross(worldPointM - Frame.Origin, impulseNs));

        // Force and torque about the centre of mass are in world coordinates.
        // Return a trial state so a coupled fluid solve can reject an update.
        public IceBody Advance(double seconds, D3 forceN, D3 torqueNm)
        {
            if (!SiUnits.IsFinite(seconds) || seconds <= 0)
                throw new ArgumentOutOfRangeException(nameof(seconds));
            double half = 0.5 * seconds;
            D3 momentum = MomentumKgMPerS + forceN * seconds;
            D3 angularMomentum = AngularMomentumKgM2PerS + torqueNm * seconds;
            D3 midpointAngularMomentum = AngularMomentumKgM2PerS + torqueNm * half;
            var midpointFrame = Rotate(Frame, Frame.Origin, AngularVelocityRadPerS * half);
            D3 rotation = AngularVelocity(midpointFrame, midpointAngularMomentum) * seconds;
            D3 centre = Frame.Origin + (MomentumKgMPerS + momentum) * (half / MassKg);
            return new IceBody(MassKg, PrincipalInertiaKgM2, Rotate(Frame, centre, rotation),
                momentum, angularMomentum);
        }

        private D3 AngularVelocity(RigidFrame frame, D3 momentum) =>
            frame.U * (D3.Dot(momentum, frame.U) / PrincipalInertiaKgM2.X) +
            frame.V * (D3.Dot(momentum, frame.V) / PrincipalInertiaKgM2.Y) +
            frame.W * (D3.Dot(momentum, frame.W) / PrincipalInertiaKgM2.Z);

        private static RigidFrame Rotate(RigidFrame frame, D3 centre, D3 rotation)
        {
            double angle = rotation.Length;
            if (angle == 0) return new RigidFrame(centre, frame.U, frame.V, frame.W);
            D3 axis = rotation / angle;
            double cosine = Math.Cos(angle), sine = Math.Sin(angle);
            D3 Turn(D3 v) => v * cosine + D3.Cross(axis, v) * sine +
                axis * (D3.Dot(axis, v) * (1 - cosine));
            D3 u = Turn(frame.U).Unit();
            D3 v = Turn(frame.V);
            v = (v - u * D3.Dot(u, v)).Unit();
            return new RigidFrame(centre, u, v, D3.Cross(u, v));
        }
    }
}
