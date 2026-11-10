using UnityEngine;

namespace PhaseTransition.Physics
{
    /// <summary>
    /// A spatial SI enthalpy grid for a small beaker sample. Each occupied cell has its own
    /// enthalpy and phase fractions, so locally melted water can warm while ice remains
    /// elsewhere, and local boiling can begin while most of the sample remains liquid.
    /// This is conduction-only: it does not yet advect water or vapour.
    /// </summary>
    public sealed class BeakerEnergyLedger : MonoBehaviour
    {
        [Header("Initial sample")]
        [Min(0.001f)] public float sampleMassKg = 0.25f;
        [Range(-100f, 0f)] public float initialIceTemperatureC = -50f;

        [Header("Beaker geometry (SI)")]
        [Min(0.01f)] public float beakerWidthM = 0.10f;
        [Min(0.005f)] public float beakerDepthM = 0.025f;
        [Min(0.02f)] public float beakerHeightM = 0.12f;
        [Range(4, 64)] public int gridWidth = 20;
        [Range(4, 96)] public int gridHeight = 24;

        [Header("Experimental input")]
        [Min(0f)] public float heaterPowerW = 500f;
        public bool heaterEnabled = true;
        [Min(0f)] public float altitudeM = 0f;

        public struct CellState
        {
            public bool containsSample;
            public double massKg;
            public double temperatureC;
            public double iceMassFraction;
            public double liquidMassFraction;
            public double vapourMassFraction;
        }

        public double SpecificEnthalpyJPerKg { get; private set; }
        public double AverageTemperatureC { get; private set; }
        public double IceMassFraction { get; private set; }
        public double LiquidMassFraction { get; private set; }
        public double VapourMassFraction { get; private set; }
        public double AmbientPressurePa { get; private set; }
        public double WaterLevelM { get; private set; }
        public double TotalSampleMassKg { get; private set; }

        private const double IceDensityKgPerM3 = 917.0;
        private const double LiquidDensityKgPerM3 = 998.2;
        private const double IceConductivityWPerMK = 2.2;
        private const double LiquidConductivityWPerMK = 0.60;
        private const double VapourConductivityWPerMK = 0.025;
        private const double VapourDensityKgPerM3 = 0.60;

        private double[,] cellMassKg;
        private double[,] cellEnthalpyJPerKg;
        private WaterThermodynamicsSI.EquilibriumState[,] cellThermo;
        private double[,] energyChangeJ;
        private bool initialized;

        public int GridWidth => gridWidth;
        public int GridHeight => gridHeight;

        private void Start() => ResetSample();

        private void Update()
        {
            if (!initialized) ResetSample();
            AmbientPressurePa = WaterThermodynamicsSI.AmbientPressureAtAltitudePa(altitudeM);

            // Explicit Fourier conduction is stable at this SI cell size for ice and water.
            // Substeps avoid losing energy when a rendered frame takes longer than this bound.
            double timeRemaining = Time.deltaTime;
            while (timeRemaining > 0.0)
            {
                double dt = System.Math.Min(timeRemaining, 0.025);
                AdvanceThermalStep(dt);
                timeRemaining -= dt;
            }
            RebuildCellStatesAndTotals();
        }

        public CellState GetCellState(int x, int y)
        {
            if (!initialized || x < 0 || y < 0 || x >= gridWidth || y >= gridHeight)
                return default;

            WaterThermodynamicsSI.EquilibriumState s = cellThermo[x, y];
            return new CellState
            {
                containsSample = cellMassKg[x, y] > 0.0,
                massKg = cellMassKg[x, y],
                temperatureC = s.temperatureC,
                iceMassFraction = s.iceMassFraction,
                liquidMassFraction = s.liquidMassFraction,
                vapourMassFraction = s.vapourMassFraction
            };
        }

        [ContextMenu("Reset sample to initial ice temperature")]
        public void ResetSample()
        {
            gridWidth = Mathf.Clamp(gridWidth, 4, 64);
            gridHeight = Mathf.Clamp(gridHeight, 4, 96);
            cellMassKg = new double[gridWidth, gridHeight];
            cellEnthalpyJPerKg = new double[gridWidth, gridHeight];
            cellThermo = new WaterThermodynamicsSI.EquilibriumState[gridWidth, gridHeight];
            energyChangeJ = new double[gridWidth, gridHeight];

            double dx = beakerWidthM / gridWidth;
            double dy = beakerHeightM / gridHeight;
            double cellVolumeM3 = dx * dy * beakerDepthM;
            double iceMassPerFullCell = IceDensityKgPerM3 * cellVolumeM3;
            double maximumMass = iceMassPerFullCell * gridWidth * gridHeight;
            sampleMassKg = (float)System.Math.Min(System.Math.Max(0.001, sampleMassKg), maximumMass);

            double numberOfFullCellEquivalents = sampleMassKg / iceMassPerFullCell;
            int fullRows = Mathf.Min(gridHeight, (int)(numberOfFullCellEquivalents / gridWidth));
            double partialRowFraction = numberOfFullCellEquivalents / gridWidth - fullRows;
            double initialEnthalpy = WaterThermodynamicsSI.IceSpecificEnthalpyJPerKg(
                initialIceTemperatureC + 273.15);

            for (int y = 0; y < gridHeight; y++)
                for (int x = 0; x < gridWidth; x++)
                {
                    double rowFill = y < fullRows ? 1.0 : (y == fullRows ? partialRowFraction : 0.0);
                    if (rowFill <= 0.0) continue;
                    cellMassKg[x, y] = iceMassPerFullCell * rowFill;
                    cellEnthalpyJPerKg[x, y] = initialEnthalpy;
                }

            AmbientPressurePa = WaterThermodynamicsSI.AmbientPressureAtAltitudePa(altitudeM);
            initialized = true;
            RebuildCellStatesAndTotals();
        }

        /// <summary>Apply external energy, positive into the sample, distributed by cell mass.</summary>
        public void AddEnergyJ(double energyJ)
        {
            if (!initialized || TotalSampleMassKg <= 0.0) return;
            double deltaSpecificEnthalpy = energyJ / TotalSampleMassKg;
            for (int y = 0; y < gridHeight; y++)
                for (int x = 0; x < gridWidth; x++)
                    if (cellMassKg[x, y] > 0.0)
                        cellEnthalpyJPerKg[x, y] += deltaSpecificEnthalpy;
            RebuildCellStatesAndTotals();
        }

        private void AdvanceThermalStep(double dt)
        {
            RebuildCellStatesAndTotals();
            System.Array.Clear(energyChangeJ, 0, energyChangeJ.Length);
            double dx = beakerWidthM / gridWidth;
            double dy = beakerHeightM / gridHeight;
            double cellVolume = dx * dy * beakerDepthM;

            // Pairwise finite-volume Fourier fluxes; every internal face is counted once.
            for (int y = 0; y < gridHeight; y++)
                for (int x = 0; x < gridWidth; x++)
                {
                    if (cellMassKg[x, y] <= 0.0) continue;
                    if (x + 1 < gridWidth) TransferHeat(x, y, x + 1, y, dy * beakerDepthM, dx, dt, cellVolume);
                    if (y + 1 < gridHeight) TransferHeat(x, y, x, y + 1, dx * beakerDepthM, dy, dt, cellVolume);
                }

            // Heater power is treated as fully absorbed and spread evenly over the wet base.
            // A metal base plate and burner-to-glass losses are not included yet.
            if (heaterEnabled && heaterPowerW > 0f)
            {
                double wetBaseMass = 0.0;
                for (int x = 0; x < gridWidth; x++) wetBaseMass += cellMassKg[x, 0];
                if (wetBaseMass > 0.0)
                    for (int x = 0; x < gridWidth; x++)
                        if (cellMassKg[x, 0] > 0.0)
                            energyChangeJ[x, 0] += heaterPowerW * dt * cellMassKg[x, 0] / wetBaseMass;
            }

            for (int y = 0; y < gridHeight; y++)
                for (int x = 0; x < gridWidth; x++)
                    if (cellMassKg[x, y] > 0.0)
                        cellEnthalpyJPerKg[x, y] += energyChangeJ[x, y] / cellMassKg[x, y];
            RebuildCellStatesAndTotals();
        }

        private void TransferHeat(int ax, int ay, int bx, int by,
            double faceAreaM2, double distanceM, double dt, double cellVolumeM3)
        {
            if (cellMassKg[bx, by] <= 0.0) return;
            double kA = Conductivity(cellThermo[ax, ay]);
            double kB = Conductivity(cellThermo[bx, by]);
            double faceConductivity = 2.0 * kA * kB / (kA + kB);
            double fillA = System.Math.Min(1.0, cellMassKg[ax, ay] / (IceDensityKgPerM3 * cellVolumeM3));
            double fillB = System.Math.Min(1.0, cellMassKg[bx, by] / (IceDensityKgPerM3 * cellVolumeM3));
            double conductanceWPerK = faceConductivity * faceAreaM2 / distanceM * System.Math.Min(fillA, fillB);
            double energyToA = conductanceWPerK * (cellThermo[bx, by].temperatureC - cellThermo[ax, ay].temperatureC) * dt;
            energyChangeJ[ax, ay] += energyToA;
            energyChangeJ[bx, by] -= energyToA;
        }

        private static double Conductivity(WaterThermodynamicsSI.EquilibriumState s)
        {
            return s.iceMassFraction * IceConductivityWPerMK +
                   s.liquidMassFraction * LiquidConductivityWPerMK +
                   s.vapourMassFraction * VapourConductivityWPerMK;
        }

        private void RebuildCellStatesAndTotals()
        {
            if (!initialized || cellMassKg == null) return;
            double dx = beakerWidthM / gridWidth;
            double dy = beakerHeightM / gridHeight;
            double condensedVolume = 0.0;
            double estimatedLevel = WaterLevelM;
            // Two passes couple local saturation pressure to the hydrostatic head.
            for (int pass = 0; pass < 2; pass++)
            {
                condensedVolume = 0.0;
                for (int y = 0; y < gridHeight; y++)
                    for (int x = 0; x < gridWidth; x++)
                    {
                        double m = cellMassKg[x, y];
                        if (m <= 0.0) continue;
                        double cellCentreHeight = (y + 0.5) * dy;
                        double depthBelowSurface = System.Math.Max(0.0, estimatedLevel - cellCentreHeight);
                        double localPressure = WaterThermodynamicsSI.HydrostaticPressurePa(
                            AmbientPressurePa, depthBelowSurface, LiquidDensityKgPerM3);
                        var s = WaterThermodynamicsSI.StateFromSpecificEnthalpy(
                            cellEnthalpyJPerKg[x, y], localPressure);
                        cellThermo[x, y] = s;
                        condensedVolume += m * (s.iceMassFraction / IceDensityKgPerM3 +
                                                  s.liquidMassFraction / LiquidDensityKgPerM3);
                    }
                estimatedLevel = System.Math.Min(beakerHeightM,
                    condensedVolume / (beakerWidthM * beakerDepthM));
            }

            WaterLevelM = estimatedLevel;
            double totalMass = 0.0, totalEnthalpy = 0.0, totalTemperature = 0.0;
            double iceMass = 0.0, liquidMass = 0.0, vapourMass = 0.0;
            for (int y = 0; y < gridHeight; y++)
                for (int x = 0; x < gridWidth; x++)
                {
                    double m = cellMassKg[x, y];
                    if (m <= 0.0) continue;
                    var s = cellThermo[x, y];
                    totalMass += m;
                    totalEnthalpy += m * cellEnthalpyJPerKg[x, y];
                    totalTemperature += m * s.temperatureC;
                    iceMass += m * s.iceMassFraction;
                    liquidMass += m * s.liquidMassFraction;
                    vapourMass += m * s.vapourMassFraction;
                }
            TotalSampleMassKg = totalMass;
            if (totalMass <= 0.0) return;
            SpecificEnthalpyJPerKg = totalEnthalpy / totalMass;
            AverageTemperatureC = totalTemperature / totalMass;
            IceMassFraction = iceMass / totalMass;
            LiquidMassFraction = liquidMass / totalMass;
            VapourMassFraction = vapourMass / totalMass;
        }
    }
}
