using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Geometry;

namespace PhysicsSim.ThreePhase3D.PhaseChange
{
    public readonly struct WettingMeasurement
    {
        public readonly double SpeedMPerS, AngleDegrees;

        public WettingMeasurement(double speedMPerS, double angleDegrees)
        {
            if (!SiUnits.IsFinite(speedMPerS) || speedMPerS < 0)
                throw new ArgumentOutOfRangeException(nameof(speedMPerS));
            if (!SiUnits.IsFinite(angleDegrees) || angleDegrees <= 0 || angleDegrees >= 180)
                throw new ArgumentOutOfRangeException(nameof(angleDegrees));
            SpeedMPerS = speedMPerS;
            AngleDegrees = angleDegrees;
        }
    }

    // Empirical dynamic-angle law measured at a declared matching distance.
    // Advancing and receding speeds in the calibration are magnitudes.
    public sealed class WettingProfile
    {
        private readonly WettingMeasurement[] advancing, receding;
        public string SurfacePreparation { get; }
        public string MeasurementSource { get; }
        public double EquilibriumAngleDegrees { get; }
        public double SlipLengthM { get; }
        public double MatchingDistanceM { get; }

        public WettingProfile(string surfacePreparation, string measurementSource, double equilibriumAngleDegrees,
            double slipLengthM, double matchingDistanceM, WettingMeasurement[] advancing, WettingMeasurement[] receding)
        {
            if (string.IsNullOrWhiteSpace(surfacePreparation) || string.IsNullOrWhiteSpace(measurementSource))
                throw new ArgumentException("Wetting calibration requires surface preparation and measurement provenance.");
            if (!SiUnits.IsFinite(slipLengthM) || !SiUnits.IsFinite(matchingDistanceM) ||
                slipLengthM <= 0 || matchingDistanceM <= slipLengthM)
                throw new ArgumentException("Matching distance must exceed the positive Navier slip length.");

            this.advancing = CopyCurve(advancing, nameof(advancing));
            this.receding = CopyCurve(receding, nameof(receding));
            if (!SiUnits.IsFinite(equilibriumAngleDegrees) ||
                equilibriumAngleDegrees < this.receding[0].AngleDegrees ||
                equilibriumAngleDegrees > this.advancing[0].AngleDegrees)
                throw new ArgumentOutOfRangeException(nameof(equilibriumAngleDegrees));

            SurfacePreparation = surfacePreparation;
            MeasurementSource = measurementSource;
            EquilibriumAngleDegrees = equilibriumAngleDegrees;
            SlipLengthM = slipLengthM;
            MatchingDistanceM = matchingDistanceM;
        }

        // A stationary line can retain any angle within the hysteresis band.
        public double AngleDegrees(double signedSpeedMPerS, double apparentAngleDegrees)
        {
            if (!SiUnits.IsFinite(signedSpeedMPerS))
                throw new ArgumentOutOfRangeException(nameof(signedSpeedMPerS));
            if (!SiUnits.IsFinite(apparentAngleDegrees) || apparentAngleDegrees < 0 || apparentAngleDegrees > 180)
                throw new ArgumentOutOfRangeException(nameof(apparentAngleDegrees));
            if (signedSpeedMPerS == 0)
                return Math.Max(receding[0].AngleDegrees, Math.Min(advancing[0].AngleDegrees, apparentAngleDegrees));

            var curve = signedSpeedMPerS > 0 ? advancing : receding;
            double speed = Math.Abs(signedSpeedMPerS);
            if (speed > curve[curve.Length - 1].SpeedMPerS)
                throw new ArgumentOutOfRangeException(nameof(signedSpeedMPerS), "Contact-line speed exceeds the calibrated range.");
            int high = 1;
            while (speed > curve[high].SpeedMPerS) high++;
            var a = curve[high - 1];
            var b = curve[high];
            double weight = (speed - a.SpeedMPerS) / (b.SpeedMPerS - a.SpeedMPerS);
            return a.AngleDegrees + weight * (b.AngleDegrees - a.AngleDegrees);
        }

        // Wall-on-fluid tangential traction for a stationary wall [Pa].
        public D3 WallTractionPa(D3 fluidVelocityMPerS, D3 wallNormal, double distanceM, double viscosityPaS)
        {
            if (!SiUnits.IsFinite(distanceM) || distanceM <= 0 ||
                !SiUnits.IsFinite(viscosityPaS) || viscosityPaS < 0)
                throw new ArgumentException("Wall distance must be positive and viscosity nonnegative.");
            var n = wallNormal.Unit();
            var tangent = fluidVelocityMPerS - n * D3.Dot(n, fluidVelocityMPerS);
            return tangent * (-viscosityPaS * WallGradientPerVelocity(distanceM));
        }

        internal double WallGradientPerVelocity(double distanceM) => 1 / (distanceM + SlipLengthM);

        private static WettingMeasurement[] CopyCurve(WettingMeasurement[] curve, string name)
        {
            if (curve == null || curve.Length < 2 || curve[0].SpeedMPerS != 0)
                throw new ArgumentException("A calibration needs a zero-speed limit and at least one moving sample.", name);
            var copy = (WettingMeasurement[])curve.Clone();
            for (int i = 0; i < copy.Length; i++)
                if (copy[i].AngleDegrees <= 0 || (i > 0 && copy[i].SpeedMPerS <= copy[i - 1].SpeedMPerS))
                    throw new ArgumentException("Calibration angles must be valid and speeds strictly increasing.", name);
            return copy;
        }
    }

    public sealed class WettingContact
    {
        public WettingProfile Profile { get; }
        public InterfacePatch Wall { get; }
        public double SignedSpeedMPerS { get; }

        public WettingContact(WettingProfile profile, InterfacePatch wall, double signedSpeedMPerS)
        {
            Profile = profile ?? throw new ArgumentNullException(nameof(profile));
            Wall = wall ?? throw new ArgumentNullException(nameof(wall));
            if (!SiUnits.IsFinite(signedSpeedMPerS)) throw new ArgumentOutOfRangeException(nameof(signedSpeedMPerS));
            SignedSpeedMPerS = signedSpeedMPerS;
        }

        internal double ContactAngleDegrees(D3 estimatedNormal)
        {
            double cosine = Math.Max(-1, Math.Min(1, D3.Dot(estimatedNormal.Unit(), Wall.NormalLocal.Unit())));
            return Profile.AngleDegrees(SignedSpeedMPerS, Math.Acos(cosine) * 180 / Math.PI);
        }

        internal D3 InterfaceNormal(D3 estimatedNormal)
        {
            var wall = Wall.NormalLocal.Unit();
            var outward = estimatedNormal.Unit();
            double cosine = Math.Max(-1, Math.Min(1, D3.Dot(outward, wall)));
            double angle = ContactAngleDegrees(outward) * Math.PI / 180;
            var tangent = outward - wall * cosine;
            if (tangent.Length < 1e-12)
                throw new InvalidOperationException("The wall-parallel contact-line direction is unresolved.");
            return tangent.Unit() * Math.Sin(angle) + wall * Math.Cos(angle);
        }
    }
}
