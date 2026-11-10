using System;
using System.Collections.Generic;
using System.Runtime.Serialization;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Geometry;

namespace PhysicsSim.ThreePhase3D.Verification
{
    [DataContract]
    public sealed class CheckResult
    {
        [DataMember] public string name;
        [DataMember] public string validationId;
        [DataMember] public bool passed;
        [DataMember] public string detail;
        [DataMember] public CheckMetric[] metrics = new CheckMetric[0];
    }

    [DataContract]
    public sealed class CheckMetric
    {
        [DataMember] public string name;
        [DataMember] public string unit;
        [DataMember] public string definition;
        [DataMember] public double actual;
        [DataMember] public double? minimum_allowed;
        [DataMember] public double? maximum_allowed;
        [DataMember] public bool passed;
    }

    public static partial class FoundationChecks
    {
        public static List<CheckResult> RunS00()
        {
            var results = new List<CheckResult>();
            Check(results, "SI temperature, pressure, mass and volume", "V01", () =>
            {
                Near(SiUnits.CelsiusToKelvin(-50), 223.15, 1e-12);
                Near(SiUnits.KelvinToCelsius(289.15), 16, 1e-12);
                Near(SiUnits.AbsolutePressure(-1000, 101325), 100325, 0);
                Near(SiUnits.GaugePressure(100325, 101325), -1000, 0);
                // Synthetic density for dimensional verification, not a water-property implementation.
                Near(SiUnits.MassFromVolume(0.00025, 1000), 0.25, 1e-15);
                Near(SiUnits.VolumeFromMass(0.25, 1000), 0.00025, 1e-15);
                Near(SiUnits.CylinderVolume(0.1, 0.2), Math.PI * 0.002, 1e-16);
                Throws(() => SiUnits.CelsiusToKelvin(-300));
                Throws(() => SiUnits.AbsolutePressure(-200000, 101325));
                Throws(() => SiUnits.VolumeFromMass(1, 0));
            });
            Check(results, "Preview permits explicit missing physics data; physical run is blocked", "V01", () =>
            {
                var report = new ScenarioConfigSI().Validate();
                Require(report.CanPreview, report.ToString());
                Require(!report.PhysicalInputsReady && report.MissingData.Count >= 4, "Missing material data was accepted.");
            });
            Check(results, "Reject incompatible configuration and nonfinite values", "V01", () =>
            {
                var s = new ScenarioConfigSI(); s.physical.geometry.baseThicknessM = -0.001;
                Require(!s.Validate().CanPreview, "Negative base accepted.");
                s = new ScenarioConfigSI(); s.physical.ambientRelativeHumidity = double.NaN;
                Require(!s.Validate().CanPreview, "NaN humidity accepted.");
                s = new ScenarioConfigSI(); s.physical.gravityYMetresPerSecondSquared = 9.80665;
                Require(!s.Validate().CanPreview, "Upward gravity accepted.");
                s = new ScenarioConfigSI(); s.numerics.cellsZ = 1;
                Require(!s.Validate().CanPreview, "Two-dimensional grid accepted.");
            });
            Check(results, "Mass/volume agreement uses a supplied synthetic provider", "V01", () =>
            {
                var s = new ScenarioConfigSI(); s.physical.initialPhase = InitialPhase.Liquid;
                s.physical.amountBasis = AmountBasis.MassAndVolume; s.physical.sampleVolumeM3 = 0.00025;
                Require(s.Validate((phase, t, p) => 1000).CanPreview, "Consistent synthetic amount rejected.");
                s.physical.sampleVolumeM3 *= 2;
                Require(!s.Validate((phase, t, p) => 1000).CanPreview, "Inconsistent amounts accepted.");
                Require(!s.Validate((phase, t, p) => double.NaN).CanPreview, "Invalid provider accepted.");
            });
            Check(results, "Configuration serialization retains SI identity", "V01", () =>
            {
                var s = new ScenarioConfigSI(); byte[] bytes = RecordJson.Encode(s);
                var restored = RecordJson.Decode<ScenarioConfigSI>(bytes);
                Require(RecordJson.Hash(bytes) == RecordJson.Hash(RecordJson.Encode(restored)), "Configuration changed on reload.");
                Near(restored.physical.geometry.baseThicknessM, 0.003, 0);
            });
            Check(results, "Glass preview scale, winding and polygonal volume", "V01", () =>
            {
                var g = new GeometryInputs(); int segments = 96;
                var mesh = ApparatusSurface.Beaker(g, segments);
                double expected = (Math.PI * g.OuterRadiusM * g.OuterRadiusM * (g.cavityHeightM + g.baseThicknessM) - g.CavityVolumeM3) *
                    (segments * Math.Sin(2 * Math.PI / segments) / (2 * Math.PI));
                Near(mesh.SignedVolumeM3(), expected, 1e-16);
                double minY = double.MaxValue, maxY = double.MinValue;
                for (int i = 1; i < mesh.positionsM.Length; i += 3)
                { minY = Math.Min(minY, mesh.positionsM[i]); maxY = Math.Max(maxY, mesh.positionsM[i]); }
                Near(minY, g.externalBaseHeightM, 0); Near(maxY, g.RimHeightM, 0);
                Require(mesh.SignedVolumeM3() > 0, "Surface winding is inverted.");
                Require(ApparatusSurface.SupportRing(g, 96).SignedVolumeM3() > 0, "Ring winding is inverted.");
            });
            return results;
        }

        static void Check(List<CheckResult> results, string name, string id, Action action)
        {
            try { action(); results.Add(new CheckResult { name = name, validationId = id, passed = true, detail = "All declared assertions passed." }); }
            catch (Exception ex) { results.Add(new CheckResult { name = name, validationId = id, passed = false, detail = ex.ToString() }); }
        }
        static void Require(bool value, string reason) { if (!value) throw new InvalidOperationException(reason); }
        static void Near(double actual, double expected, double absoluteTolerance)
        { Require(SiUnits.IsFinite(actual) && Math.Abs(actual - expected) <= absoluteTolerance, "actual=" + actual.ToString("R") + "; expected=" + expected.ToString("R") + "; tolerance=" + absoluteTolerance.ToString("R")); }
        static void Throws(Action action)
        {
            bool failed = false;
            try { action(); } catch (ArgumentException) { failed = true; } catch (InvalidOperationException) { failed = true; }
            Require(failed, "Expected invalid operation was accepted.");
        }
    }
}
