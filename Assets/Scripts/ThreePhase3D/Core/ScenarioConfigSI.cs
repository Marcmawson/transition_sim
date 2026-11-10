using System;
using System.Collections.Generic;
using System.Runtime.Serialization;

namespace PhysicsSim.ThreePhase3D.Core
{
    public enum InitialPhase { Empty, Ice, Liquid, Vapour }
    public enum AmountBasis { Mass, Volume, MassAndVolume }

    [Serializable, DataContract]
    public sealed class GeometryInputs
    {
        [DataMember] public double innerRadiusM = 0.06;
        [DataMember] public double cavityHeightM = 0.16;
        [DataMember] public double wallThicknessM = 0.003;
        [DataMember] public double baseThicknessM = 0.003;
        [DataMember] public double externalBaseHeightM = 0.12;
        [DataMember] public double tripodRingRadiusM = 0.04;
        [DataMember] public double tripodTubeRadiusM = 0.004;
        [DataMember] public double tripodFootRadiusM = 0.09;
        [DataMember] public string provenance = "SYNTHETIC cylindrical beaker; no pouring spout; dimensions are chosen test inputs.";
        public double OuterRadiusM => innerRadiusM + wallThicknessM;
        public double RimHeightM => externalBaseHeightM + baseThicknessM + cavityHeightM;
        public double CavityVolumeM3 => SiUnits.CylinderVolume(innerRadiusM, cavityHeightM);
    }

    [Serializable, DataContract]
    public sealed class PhysicalInputs
    {
        [DataMember] public bool synthetic = true;
        [DataMember] public GeometryInputs geometry = new GeometryInputs();
        [DataMember] public double domainWidthM = 0.4;
        [DataMember] public double domainHeightM = 0.7;
        [DataMember] public double domainDepthM = 0.4;
        [DataMember] public double gravityYMetresPerSecondSquared = -9.80665;
        [DataMember] public double ambientTemperatureK = 289.15;
        [DataMember] public double ambientAbsolutePressurePa = 101325;
        [DataMember] public double ambientRelativeHumidity = 0.5;
        [DataMember] public InitialPhase initialPhase = InitialPhase.Empty;
        [DataMember] public AmountBasis amountBasis = AmountBasis.Mass;
        [DataMember] public double sampleMassKg = 0.25;
        [DataMember] public double sampleVolumeM3 = 0;
        [DataMember] public double sampleTemperatureK = 223.15;
        [DataMember] public double absorbedBurnerPowerW = 0;
        [DataMember] public double burnerGaussianWidthM = 0.015;
        [DataMember] public string ambientProvenance = "Declared test: 16 deg C, 101325 Pa, RH 50%; not measured room conditions.";
        [DataMember] public string[] missingMeasurements = {
            "MAT019 actual beaker dimensions", "MAT020 tripod grade/rho/cp/k",
            "MAT021 contact resistance and patches", "MAT022 absorbed burner power",
            "MAT023 radial flux profile", "MAT026 humidity and ventilation",
            "MAT028 wetting/slip data", "MAT029 nucleation/degassing protocol",
            "MAT030 glass radiation data", "MAT033 aerosol seeds", "MAT034 ice contact data"
        };
    }

    [Serializable, DataContract]
    public sealed class MaterialReference
    {
        [DataMember] public string id;
        [DataMember] public string source;
        [DataMember] public string version;
        [DataMember] public string sha256;
        [DataMember] public bool providerVerified;
        [DataMember] public double minTemperatureK;
        [DataMember] public double maxTemperatureK;
        [DataMember] public double minPressurePa;
        [DataMember] public double maxPressurePa;
        [DataMember] public string missingData;
    }

    [Serializable, DataContract]
    public sealed class NumericalSettings
    {
        [DataMember] public int cellsX = 64;
        [DataMember] public int cellsY = 96;
        [DataMember] public int cellsZ = 64;
        [DataMember] public double maximumStepS = 0.001;
        [DataMember] public double pressureRelativeTolerance = 1e-6;
        [DataMember] public double nonlinearRelativeTolerance = 1e-6;
        [DataMember] public double diagnosticIntervalS = 0.1;
        [DataMember] public double machReviewLimit = 0.1;
        [DataMember] public int randomSeed = 0;
    }

    [Serializable, DataContract]
    public sealed class DisplaySettings
    {
        [DataMember] public int cylinderSegments = 96;
        [DataMember] public float glassOpacity = 0.16f;
        [DataMember] public double playbackSpeed = 1;
        [DataMember] public bool showSetupReport = true;
        [DataMember] public bool lightAddsPhysicalHeat = false;
    }

    public sealed class ConfigurationReport
    {
        public readonly List<string> Errors = new List<string>();
        public readonly List<string> MissingData = new List<string>();
        public readonly List<string> Warnings = new List<string>();
        public bool CanPreview => Errors.Count == 0;
        public bool PhysicalInputsReady => CanPreview && MissingData.Count == 0;
        public override string ToString() => "Errors: " + string.Join("; ", Errors) +
            "\nMissing physical data: " + string.Join("; ", MissingData) + "\nNotes: " + string.Join("; ", Warnings);
    }

    [Serializable, DataContract]
    public sealed class ScenarioConfigSI
    {
        [DataMember] public string schemaVersion = "1.0";
        [DataMember] public string scenarioId = "S00-empty-synthetic-beaker";
        [DataMember] public PhysicalInputs physical = new PhysicalInputs();
        [DataMember] public NumericalSettings numerics = new NumericalSettings();
        [DataMember] public DisplaySettings display = new DisplaySettings();
        [DataMember] public MaterialReference[] materials = {
            new MaterialReference { id = "H2O", source = "D01 D02 D03 D04 D05 D12", version = "PENDING S02", missingData = "Verified EOS/transport tables and phase validity ranges" },
            new MaterialReference { id = "HumidAir", source = "D06 D07 D08 D09 D13", version = "PENDING S02/S12", missingData = "Verified humid-mixture EOS and transport" },
            new MaterialReference { id = "DURAN", source = "D10", version = "PT_TTS_1019 EN Feb 2026", missingData = "MAT014 heat capacity curve; full material ranges" },
            new MaterialReference { id = "Tripod", source = "PENDING", version = "PENDING", missingData = "MAT020 grade and thermal properties" }
        };

        // Physics-ID: SI-CONFIGURATION-GATE
        // Implements: R004 R043 R052
        // Model: Specification.md sections 2, 3 and 5.
        // Sources: D00 (SI Brochure sections 2.3 and 3); project configuration contract.
        // Source-Locators: D00 sections 2.3 and 3; Specification.md sections 2, 3 and 5.
        // Inputs-SI: geometry [m], ambient/sample T [K], p_abs [Pa], amount [kg or m3].
        // Outputs-SI: dimensional errors and missing-data report; no physical state produced.
        // State-Ownership: report only; never edits, clamps or supplies missing inputs.
        // Conservation: optional mass/volume consistency uses supplied density, not a guessed constant.
        // Discretisation: finite/range checks; synthetic tests may supply a density delegate.
        // Validity: preview checks do not certify material providers or future solver readiness.
        // Validation: V01
        // Evidence: PENDING
        // Status: IMPLEMENTED_UNVERIFIED
        // Limitations: material validity rectangles are necessary metadata, not full EOS stability tests.
        public ConfigurationReport Validate(Func<InitialPhase, double, double, double> verifiedDensity = null)
        {
            var report = new ConfigurationReport();
            if (schemaVersion != "1.0" || string.IsNullOrWhiteSpace(scenarioId)) report.Errors.Add("Unsupported schema or missing scenario ID.");
            if (physical == null || physical.geometry == null || numerics == null || display == null)
            { report.Errors.Add("Missing configuration section."); return report; }
            var p = physical; var g = p.geometry; var n = numerics;
            CheckPositive(report, g.innerRadiusM, "inner radius m");
            CheckPositive(report, g.cavityHeightM, "cavity height m");
            CheckPositive(report, g.wallThicknessM, "wall thickness m");
            CheckPositive(report, g.baseThicknessM, "base thickness m");
            CheckPositive(report, g.externalBaseHeightM, "tripod height m");
            CheckPositive(report, g.tripodRingRadiusM, "ring radius m");
            CheckPositive(report, g.tripodTubeRadiusM, "tube radius m");
            CheckPositive(report, g.tripodFootRadiusM, "foot radius m");
            CheckPositive(report, p.domainWidthM, "domain width m");
            CheckPositive(report, p.domainHeightM, "domain height m");
            CheckPositive(report, p.domainDepthM, "domain depth m");
            if (2 * Math.Max(g.OuterRadiusM, g.tripodFootRadiusM + g.tripodTubeRadiusM) >= Math.Min(p.domainWidthM, p.domainDepthM) ||
                g.RimHeightM >= p.domainHeightM) report.Errors.Add("Domain must contain apparatus and exterior space.");
            if (g.tripodRingRadiusM + g.tripodTubeRadiusM >= g.OuterRadiusM ||
                3 * g.tripodTubeRadiusM >= g.externalBaseHeightM) report.Errors.Add("Tripod does not fit beneath beaker base.");
            if (!SiUnits.IsFinite(p.gravityYMetresPerSecondSquared) || p.gravityYMetresPerSecondSquared > 0) report.Errors.Add("y is up; gravity y must be finite and nonpositive.");
            CheckPositive(report, p.ambientTemperatureK, "ambient T K");
            CheckPositive(report, p.sampleTemperatureK, "sample T K");
            CheckPositive(report, p.ambientAbsolutePressurePa, "ambient p_abs Pa");
            if (!SiUnits.IsFinite(p.ambientRelativeHumidity) || p.ambientRelativeHumidity < 0 || p.ambientRelativeHumidity > 1)
                report.Errors.Add("Initial RH must lie in [0,1]; supersaturation needs a separately supported case.");
            if (!SiUnits.IsFinite(p.absorbedBurnerPowerW) || p.absorbedBurnerPowerW < 0) report.Errors.Add("Absorbed burner W must be nonnegative.");
            CheckPositive(report, p.burnerGaussianWidthM, "burner width m");
            if (!Enum.IsDefined(typeof(InitialPhase), p.initialPhase) || !Enum.IsDefined(typeof(AmountBasis), p.amountBasis)) report.Errors.Add("Unknown initial phase or amount basis.");
            if (p.initialPhase != InitialPhase.Empty)
            {
                if (p.amountBasis != AmountBasis.Volume) CheckPositive(report, p.sampleMassKg, "sample mass kg");
                if (p.amountBasis != AmountBasis.Mass) CheckPositive(report, p.sampleVolumeM3, "sample volume m3");
                if (verifiedDensity == null) report.MissingData.Add("No verified density provider for initial mass/volume and occupied geometry.");
                else
                {
                    double rho = verifiedDensity(p.initialPhase, p.sampleTemperatureK, p.ambientAbsolutePressurePa);
                    if (!SiUnits.IsFinite(rho) || rho <= 0) report.Errors.Add("Density provider returned invalid kg/m3.");
                    else if (p.amountBasis == AmountBasis.MassAndVolume &&
                        Math.Abs(p.sampleMassKg - p.sampleVolumeM3 * rho) > 1e-10 * Math.Max(p.sampleMassKg, p.sampleVolumeM3 * rho))
                        report.Errors.Add("Initial mass and volume disagree with supplied density (1e-10 relative consistency tolerance).");
                }
            }
            if (n.cellsX < 4 || n.cellsY < 4 || n.cellsZ < 4 || n.cellsX > 2048 || n.cellsY > 2048 || n.cellsZ > 2048)
                report.Errors.Add("All three grid dimensions must lie in [4,2048]; allocation is checked separately in S03.");
            CheckPositive(report, n.maximumStepS, "maximum dt s");
            CheckPositive(report, n.diagnosticIntervalS, "diagnostic interval s");
            CheckPositive(report, n.pressureRelativeTolerance, "pressure relative tolerance");
            CheckPositive(report, n.nonlinearRelativeTolerance, "nonlinear relative tolerance");
            CheckPositive(report, n.machReviewLimit, "Mach review limit");
            CheckPositive(report, display.playbackSpeed, "playback speed");
            if (display.cylinderSegments < 16 || display.cylinderSegments > 512 ||
                !SiUnits.IsFinite(display.glassOpacity) || display.glassOpacity <= 0 || display.glassOpacity > 1)
                report.Errors.Add("Invalid display tessellation or opacity.");
            if (display.lightAddsPhysicalHeat) report.Errors.Add("Rendering-light heat coupling is not implemented; supply a separate validated radiometric model.");
            if (string.IsNullOrWhiteSpace(g.provenance)) report.Errors.Add("Geometry provenance is required.");
            if (!p.synthetic && p.missingMeasurements != null) report.MissingData.AddRange(p.missingMeasurements);
            else report.Warnings.Add("Synthetic geometry/ambient inputs; no apparatus validation claim.");
            if (materials == null || materials.Length == 0) report.MissingData.Add("Material references and validity ranges absent.");
            else foreach (var m in materials)
            {
                if (m == null || string.IsNullOrWhiteSpace(m.id)) { report.Errors.Add("Material ID absent."); continue; }
                if (!m.providerVerified || string.IsNullOrWhiteSpace(m.sha256) ||
                    !ValidRange(m.minTemperatureK, m.maxTemperatureK) || !ValidRange(m.minPressurePa, m.maxPressurePa))
                    report.MissingData.Add(m.id + ": verified provider, hash or validity range missing. " + m.missingData);
                else if (p.ambientTemperatureK < m.minTemperatureK || p.ambientTemperatureK > m.maxTemperatureK ||
                    p.ambientAbsolutePressurePa < m.minPressurePa || p.ambientAbsolutePressurePa > m.maxPressurePa ||
                    (p.initialPhase != InitialPhase.Empty && (p.sampleTemperatureK < m.minTemperatureK || p.sampleTemperatureK > m.maxTemperatureK)))
                    report.Errors.Add(m.id + ": requested state lies outside declared material range.");
            }
            report.Warnings.Add("S00 preview allocates no fluid fields and does not advance physical time.");
            return report;
        }

        static bool ValidRange(double min, double max) => SiUnits.IsFinite(min) && SiUnits.IsFinite(max) && min > 0 && max > min;
        static void CheckPositive(ConfigurationReport report, double value, string name)
        { if (!SiUnits.IsFinite(value) || value <= 0) report.Errors.Add(name + " must be finite and positive."); }
    }
}
