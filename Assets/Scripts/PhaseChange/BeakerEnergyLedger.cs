using UnityEngine;

namespace PhaseTransition.Physics
{
    /// <summary>
    /// A spatial SI enthalpy grid for a small beaker sample. Each occupied cell has its own
    /// enthalpy and phase fractions, so locally melted water can warm while ice remains
    /// elsewhere, and local boiling can begin while most of the sample remains liquid.
    /// Liquid cells use a 2-D incompressible flow approximation; liquid mass and enthalpy move
    /// between cells. Vapour bubbles use a separate buoyant-rise closure and vent at the top.
    /// Ice remains stationary, and the bubble motion is not a gas Navier-Stokes solution.
    /// </summary>
    public sealed class BeakerEnergyLedger : MonoBehaviour
    {
        [Header("Initial sample")]
        [Min(0.001f)] public float sampleMassKg = 0.25f;
        [Range(0.5f, 1f)] public float initialIceWidthFraction = 0.65f;
        [Range(-100f, 0f)] public float initialIceTemperatureC = -50f;

        [Header("Beaker geometry (SI)")]
        [Min(0.01f)] public float beakerWidthM = 0.10f;
        [Min(0.005f)] public float beakerDepthM = 0.025f;
        [Min(0.02f)] public float beakerHeightM = 0.24f;
        [Range(4, 64)] public int gridWidth = 20;
        [Range(4, 96)] public int gridHeight = 24;

        [Header("Experimental input")]
        [Min(0f)] public float heaterPowerW = 500f;
        public bool heaterEnabled = true;
        [Min(0f)] public float altitudeM = 0f;
        [Min(0.005f)] public float heaterFootprintWidthM = 0.05f;

        [Header("Vapour bubble model (SI)")]
        [Min(0.0005f)] public float vapourBubbleDiameterM = 0.005f;
        [Min(0.1f)] public float vapourBubbleDragCoefficient = 1.0f;

        public struct CellState
        {
            public bool containsSample;
            public double massKg;
            public double temperatureC;
            public double iceMassFraction;
            public double liquidMassFraction;
            public double vapourMassFraction;
            public double velocityXMS;
            public double velocityYMS;
        }

        public double SpecificEnthalpyJPerKg { get; private set; }
        public double AverageTemperatureC { get; private set; }
        public double IceMassFraction { get; private set; }
        public double LiquidMassFraction { get; private set; }
        public double VapourMassFraction { get; private set; }
        public double AmbientPressurePa { get; private set; }
        public double WaterLevelM { get; private set; }
        public double TotalSampleMassKg { get; private set; }
        public double MaximumLiquidSpeedMS { get; private set; }
        public double MaximumBubbleRiseSpeedMS { get; private set; }
        public double VentedVapourMassKg { get; private set; }

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
        private double[,] massChangeKg;
        private double[,] buoyancyBaseMassKg;
        private double[,] buoyancyBaseEnergyJ;
        private int[] bottomContactRows;
        private LiquidConvection2D liquidFlow;
        private double[,] liquidFractionBuffer;
        private double[,] iceFractionBuffer;
        private double[,] temperatureBuffer;
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
                vapourMassFraction = s.vapourMassFraction,
                velocityXMS = liquidFlow != null ? liquidFlow.GetVelocityX(x, y) : 0.0,
                velocityYMS = liquidFlow != null ? liquidFlow.GetVelocityY(x, y) : 0.0
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
            massChangeKg = new double[gridWidth, gridHeight];
            buoyancyBaseMassKg = new double[gridWidth, gridHeight];
            buoyancyBaseEnergyJ = new double[gridWidth, gridHeight];
            bottomContactRows = new int[gridWidth];
            liquidFractionBuffer = new double[gridWidth, gridHeight];
            iceFractionBuffer = new double[gridWidth, gridHeight];
            temperatureBuffer = new double[gridWidth, gridHeight];
            liquidFlow = new LiquidConvection2D(gridWidth, gridHeight);

            double dx = beakerWidthM / gridWidth;
            double dy = beakerHeightM / gridHeight;
            double cellVolumeM3 = dx * dy * beakerDepthM;
            double iceMassPerFullCell = IceDensityKgPerM3 * cellVolumeM3;
            int occupiedColumns = Mathf.Clamp(Mathf.RoundToInt(gridWidth * initialIceWidthFraction), 1, gridWidth);
            double maximumMass = iceMassPerFullCell * occupiedColumns * gridHeight;
            sampleMassKg = (float)System.Math.Min(System.Math.Max(0.001, sampleMassKg), maximumMass);

            int firstOccupiedColumn = (gridWidth - occupiedColumns) / 2;
            double numberOfFullCellEquivalents = sampleMassKg / iceMassPerFullCell;
            int fullRows = Mathf.Min(gridHeight, (int)(numberOfFullCellEquivalents / occupiedColumns));
            double partialRowFraction = numberOfFullCellEquivalents / occupiedColumns - fullRows;
            double initialEnthalpy = WaterThermodynamicsSI.IceSpecificEnthalpyJPerKg(
                initialIceTemperatureC + 273.15);

            for (int y = 0; y < gridHeight; y++)
                for (int x = 0; x < gridWidth; x++)
                {
                    if (x < firstOccupiedColumn || x >= firstOccupiedColumn + occupiedColumns) continue;
                    double rowFill = y < fullRows ? 1.0 : (y == fullRows ? partialRowFraction : 0.0);
                    if (rowFill <= 0.0) continue;
                    cellMassKg[x, y] = iceMassPerFullCell * rowFill;
                    cellEnthalpyJPerKg[x, y] = initialEnthalpy;
                }

            AmbientPressurePa = WaterThermodynamicsSI.AmbientPressureAtAltitudePa(altitudeM);
            initialized = true;
            liquidFlow.Reset();
            VentedVapourMassKg = 0.0;
            MaximumLiquidSpeedMS = 0.0;
            MaximumBubbleRiseSpeedMS = 0.0;
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
            for (int y = 0; y < gridHeight; y++)
                for (int x = 0; x < gridWidth; x++)
                {
                    liquidFractionBuffer[x, y] = cellThermo[x, y].liquidMassFraction;
                    iceFractionBuffer[x, y] = cellThermo[x, y].iceMassFraction;
                    temperatureBuffer[x, y] = cellThermo[x, y].temperatureC;
                }

            double dx = beakerWidthM / gridWidth;
            double dy = beakerHeightM / gridHeight;
            liquidFlow.Advance(liquidFractionBuffer, iceFractionBuffer, temperatureBuffer, dx, dy, dt);
            MaximumLiquidSpeedMS = 0.0;
            for (int y = 0; y < gridHeight; y++)
                for (int x = 0; x < gridWidth; x++)
                {
                    double u = liquidFlow.GetVelocityX(x, y);
                    double v = liquidFlow.GetVelocityY(x, y);
                    double speed = System.Math.Sqrt(u * u + v * v);
                    if (speed > MaximumLiquidSpeedMS) MaximumLiquidSpeedMS = speed;
                }

            System.Array.Clear(energyChangeJ, 0, energyChangeJ.Length);
            System.Array.Clear(massChangeKg, 0, massChangeKg.Length);
            double cellVolume = dx * dy * beakerDepthM;

            // Pairwise finite-volume Fourier fluxes; every internal face is counted once.
            for (int y = 0; y < gridHeight; y++)
                for (int x = 0; x < gridWidth; x++)
                {
                    if (cellMassKg[x, y] <= 0.0) continue;
                    if (x + 1 < gridWidth) TransferHeat(x, y, x + 1, y, dy * beakerDepthM, dx, dt, cellVolume);
                    if (y + 1 < gridHeight) TransferHeat(x, y, x, y + 1, dx * beakerDepthM, dy, dt, cellVolume);
                }

            // Apply the selected input over a normalized Gaussian burner footprint at the
            // lowest occupied cells. Metal-base spreading and burner-to-beaker losses are absent.
            if (heaterEnabled && heaterPowerW > 0f)
            {
                double footprintSigma = System.Math.Max(0.001, heaterFootprintWidthM / 2.355);
                double weightSum = 0.0;
                for (int x = 0; x < gridWidth; x++)
                {
                    bottomContactRows[x] = -1;
                    for (int y = 0; y < gridHeight; y++)
                        if (cellMassKg[x, y] > 0.0)
                        {
                            bottomContactRows[x] = y;
                            double xCentre = (x + 0.5) * dx;
                            double offset = xCentre - 0.5 * beakerWidthM;
                            weightSum += System.Math.Exp(-0.5 * offset * offset /
                                (footprintSigma * footprintSigma));
                            break;
                        }
                }
                if (weightSum > 0.0)
                    for (int x = 0; x < gridWidth; x++)
                        if (bottomContactRows[x] >= 0)
                        {
                            int y = bottomContactRows[x];
                            double xCentre = (x + 0.5) * dx;
                            double offset = xCentre - 0.5 * beakerWidthM;
                            double weight = System.Math.Exp(-0.5 * offset * offset /
                                (footprintSigma * footprintSigma));
                            energyChangeJ[x, y] += heaterPowerW * dt * weight / weightSum;
                        }
            }

            AdvectLiquidMassAndEnthalpy(dt, dx, dy);

            for (int y = 0; y < gridHeight; y++)
                for (int x = 0; x < gridWidth; x++)
                {
                    double oldMass = cellMassKg[x, y];
                    double newMass = oldMass + massChangeKg[x, y];
                    if (newMass <= 1e-12)
                    {
                        cellMassKg[x, y] = 0.0;
                        cellEnthalpyJPerKg[x, y] = 0.0;
                        continue;
                    }

                    double totalEnergyJ = oldMass * cellEnthalpyJPerKg[x, y] + energyChangeJ[x, y];
                    cellMassKg[x, y] = newMass;
                    cellEnthalpyJPerKg[x, y] = totalEnergyJ / newMass;
                }
            RebuildCellStatesAndTotals();
            AdvectRisingVapour(dt, dy);
            RebuildCellStatesAndTotals();
            ApplyIceBuoyancy();
            RebuildCellStatesAndTotals();
        }

        private void AdvectLiquidMassAndEnthalpy(double dt, double dx, double dy)
        {
            double xFaceArea = dy * beakerDepthM;
            double yFaceArea = dx * beakerDepthM;
            double cellVolume = dx * dy * beakerDepthM;

            for (int y = 0; y < gridHeight; y++)
                for (int x = 0; x < gridWidth; x++)
                {
                    if (x + 1 < gridWidth)
                    {
                        double faceVelocity = 0.5 * (liquidFlow.GetVelocityX(x, y) +
                                                      liquidFlow.GetVelocityX(x + 1, y));
                        TransferLiquidAlongVelocity(x, y, x + 1, y, faceVelocity,
                            xFaceArea, cellVolume, dt);
                    }

                    if (y + 1 < gridHeight)
                    {
                        double faceVelocity = 0.5 * (liquidFlow.GetVelocityY(x, y) +
                                                      liquidFlow.GetVelocityY(x, y + 1));
                        TransferLiquidAlongVelocity(x, y, x, y + 1, faceVelocity,
                            yFaceArea, cellVolume, dt);
                    }
                }
        }

        private void TransferLiquidAlongVelocity(int ax, int ay, int bx, int by,
            double faceVelocityMS, double faceAreaM2, double cellVolumeM3, double dtS)
        {
            const double liquidDensity = 998.2;
            const double liquidHeatCapacity = 4181.3;
            if (System.Math.Abs(faceVelocityMS) < 1e-12) return;

            int donorX = faceVelocityMS >= 0.0 ? ax : bx;
            int donorY = faceVelocityMS >= 0.0 ? ay : by;
            int receiverX = faceVelocityMS >= 0.0 ? bx : ax;
            int receiverY = faceVelocityMS >= 0.0 ? by : ay;
            double donorFraction = liquidFractionBuffer[donorX, donorY];
            if (donorFraction <= 0.0) return;

            double donorLiquidMass = cellMassKg[donorX, donorY] * donorFraction;
            if (donorLiquidMass <= 0.0) return;

            // Permit the Navier-Stokes velocity to carry liquid across a free surface into an
            // empty cell. Do not let the advected liquid overfill a cell already occupied by ice.
            var receiverState = cellThermo[receiverX, receiverY];
            double occupiedVolume = cellMassKg[receiverX, receiverY] *
                (receiverState.iceMassFraction / IceDensityKgPerM3 +
                 receiverState.liquidMassFraction / LiquidDensityKgPerM3 +
                 receiverState.vapourMassFraction / VapourDensityKgPerM3);
            double receiverCapacityKg = System.Math.Max(0.0,
                (cellVolumeM3 - occupiedVolume) * liquidDensity);
            if (receiverCapacityKg <= 0.0) return;

            double signedMassFlux = liquidDensity * faceVelocityMS * faceAreaM2 * dtS * donorFraction;
            double movedMass = System.Math.Min(System.Math.Abs(signedMassFlux),
                System.Math.Min(0.05 * donorLiquidMass, receiverCapacityKg));
            if (movedMass <= 0.0) return;

            double energyFlux = movedMass * liquidHeatCapacity * temperatureBuffer[donorX, donorY];
            massChangeKg[donorX, donorY] -= movedMass;
            massChangeKg[receiverX, receiverY] += movedMass;
            energyChangeJ[donorX, donorY] -= energyFlux;
            energyChangeJ[receiverX, receiverY] += energyFlux;
        }

        private void AdvectRisingVapour(double dt, double dy)
        {
            // Coarse bubble terminal-rise closure from quadratic drag. It is separate from
            // the liquid Navier-Stokes field and is a model approximation.
            const double gasConstant = 8.314462618;
            const double molarMass = 0.01801528;
            System.Array.Clear(energyChangeJ, 0, energyChangeJ.Length);
            System.Array.Clear(massChangeKg, 0, massChangeKg.Length);

            double movedFraction = 0.0;
            MaximumBubbleRiseSpeedMS = 0.0;
            for (int y = 0; y < gridHeight; y++)
                for (int x = 0; x < gridWidth; x++)
                {
                    var state = cellThermo[x, y];
                    if (cellMassKg[x, y] <= 0.0 || state.vapourMassFraction <= 0.0) continue;

                    double gasTemperatureK = System.Math.Max(273.15, state.temperatureC + 273.15);
                    double gasDensity = state.systemPressurePa * molarMass / (gasConstant * gasTemperatureK);
                    double diameterM = System.Math.Max(0.0005, vapourBubbleDiameterM);
                    double dragCoefficient = System.Math.Max(0.1, vapourBubbleDragCoefficient);
                    double velocityMS = System.Math.Sqrt(System.Math.Max(0.0,
                        4.0 * WaterThermodynamicsSI.StandardGravity * diameterM *
                        (LiquidDensityKgPerM3 - gasDensity) /
                        (3.0 * dragCoefficient * LiquidDensityKgPerM3)));
                    if (velocityMS > MaximumBubbleRiseSpeedMS) MaximumBubbleRiseSpeedMS = velocityMS;
                    movedFraction = System.Math.Min(1.0, velocityMS * dt / dy);
                    if (movedFraction <= 0.0) continue;

                    double vapourMass = cellMassKg[x, y] * state.vapourMassFraction * movedFraction;
                    double vapourEnthalpy = VapourSpecificEnthalpy(x, y, state);
                    double vapourEnergyJ = vapourMass * vapourEnthalpy;
                    massChangeKg[x, y] -= vapourMass;
                    energyChangeJ[x, y] -= vapourEnergyJ;

                    if (y + 1 < gridHeight)
                    {
                        massChangeKg[x, y + 1] += vapourMass;
                        energyChangeJ[x, y + 1] += vapourEnergyJ;
                    }
                    else
                    {
                        VentedVapourMassKg += vapourMass;
                    }
                }

            for (int y = 0; y < gridHeight; y++)
                for (int x = 0; x < gridWidth; x++)
                {
                    double oldMass = cellMassKg[x, y];
                    double newMass = oldMass + massChangeKg[x, y];
                    if (newMass <= 1e-12)
                    {
                        cellMassKg[x, y] = 0.0;
                        cellEnthalpyJPerKg[x, y] = 0.0;
                        continue;
                    }

                    double totalEnergyJ = oldMass * cellEnthalpyJPerKg[x, y] + energyChangeJ[x, y];
                    cellMassKg[x, y] = newMass;
                    cellEnthalpyJPerKg[x, y] = totalEnergyJ / newMass;
                }
        }

        private double VapourSpecificEnthalpy(int x, int y,
            WaterThermodynamicsSI.EquilibriumState state)
        {
            double vapourFraction = state.vapourMassFraction;
            if (vapourFraction <= 1e-12) return 0.0;
            double knownPhaseEnthalpy = 0.0;
            if (state.iceMassFraction > 0.0)
            {
                double iceTemperatureK = System.Math.Max(170.0,
                    System.Math.Min(273.15, state.temperatureC + 273.15));
                knownPhaseEnthalpy += state.iceMassFraction *
                    WaterThermodynamicsSI.IceSpecificEnthalpyJPerKg(iceTemperatureK);
            }
            if (state.liquidMassFraction > 0.0)
                knownPhaseEnthalpy += state.liquidMassFraction * 4181.3 * state.temperatureC;
            return (cellEnthalpyJPerKg[x, y] - knownPhaseEnthalpy) / vapourFraction;
        }

        private void ApplyIceBuoyancy()
        {
            // Once the meltwater mass can support the remaining ice weight, reposition the
            // ice phase to its Archimedean flotation height in each occupied x-column.
            // This is a coarse grid-level solid-motion closure, not a rigid-body solver.
            double totalIceMass = 0.0;
            double totalLiquidMass = 0.0;
            for (int y = 0; y < gridHeight; y++)
                for (int x = 0; x < gridWidth; x++)
                {
                    totalIceMass += cellMassKg[x, y] * cellThermo[x, y].iceMassFraction;
                    totalLiquidMass += cellMassKg[x, y] * cellThermo[x, y].liquidMassFraction;
                }
            if (totalIceMass <= 1e-12) return;

            double dx = beakerWidthM / gridWidth;
            double dy = beakerHeightM / gridHeight;
            double columnArea = dx * beakerDepthM;
            double waterSurfaceM = (totalIceMass + totalLiquidMass) /
                (LiquidDensityKgPerM3 * beakerWidthM * beakerDepthM);
            bool floating = totalLiquidMass >= totalIceMass;

            for (int x = 0; x < gridWidth; x++)
            {
                double columnIceMass = 0.0;
                double columnIceEnergyJ = 0.0;
                for (int y = 0; y < gridHeight; y++)
                {
                    var state = cellThermo[x, y];
                    double mass = cellMassKg[x, y];
                    double iceMass = mass * state.iceMassFraction;
                    double iceTemperatureK = System.Math.Max(170.0,
                        System.Math.Min(273.15, state.temperatureC + 273.15));
                    double iceSpecificEnthalpy = WaterThermodynamicsSI.IceSpecificEnthalpyJPerKg(iceTemperatureK);
                    double iceEnergy = iceMass * iceSpecificEnthalpy;
                    columnIceMass += iceMass;
                    columnIceEnergyJ += iceEnergy;
                    buoyancyBaseMassKg[x, y] = mass - iceMass;
                    buoyancyBaseEnergyJ[x, y] = mass * cellEnthalpyJPerKg[x, y] - iceEnergy;
                }
                if (columnIceMass <= 1e-12) continue;

                double iceHeightM = columnIceMass / (IceDensityKgPerM3 * columnArea);
                double iceBottomM = floating
                    ? waterSurfaceM - columnIceMass / (LiquidDensityKgPerM3 * columnArea)
                    : 0.0;
                iceBottomM = System.Math.Max(0.0, System.Math.Min(
                    beakerHeightM - iceHeightM, iceBottomM));
                double iceTopM = iceBottomM + iceHeightM;
                double meanIceEnthalpy = columnIceEnergyJ / columnIceMass;

                for (int y = 0; y < gridHeight; y++)
                {
                    double cellBottomM = y * dy;
                    double cellTopM = cellBottomM + dy;
                    double overlapM = System.Math.Max(0.0,
                        System.Math.Min(cellTopM, iceTopM) - System.Math.Max(cellBottomM, iceBottomM));
                    double iceMass = overlapM * IceDensityKgPerM3 * columnArea;
                    double newMass = buoyancyBaseMassKg[x, y] + iceMass;
                    if (newMass <= 1e-12)
                    {
                        cellMassKg[x, y] = 0.0;
                        cellEnthalpyJPerKg[x, y] = 0.0;
                    }
                    else
                    {
                        cellMassKg[x, y] = newMass;
                        cellEnthalpyJPerKg[x, y] =
                            (buoyancyBaseEnergyJ[x, y] + iceMass * meanIceEnthalpy) / newMass;
                    }
                }
            }
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
                {
                    double cellCentreHeight = (y + 0.5) * dy;
                    double depthBelowSurface = System.Math.Max(0.0, estimatedLevel - cellCentreHeight);
                    double localPressure = WaterThermodynamicsSI.HydrostaticPressurePa(
                        AmbientPressurePa, depthBelowSurface, LiquidDensityKgPerM3);
                    double localSaturationTemperature = WaterThermodynamicsSI.SaturationTemperatureK(localPressure);
                    for (int x = 0; x < gridWidth; x++)
                    {
                        double m = cellMassKg[x, y];
                        if (m <= 0.0) continue;
                        var s = WaterThermodynamicsSI.StateFromSpecificEnthalpy(
                            cellEnthalpyJPerKg[x, y], localPressure, localSaturationTemperature);
                        cellThermo[x, y] = s;
                        condensedVolume += m * (s.iceMassFraction / IceDensityKgPerM3 +
                                                  s.liquidMassFraction / LiquidDensityKgPerM3);
                    }
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
