using System;
using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Materials;

namespace PhysicsSim.ThreePhase3D.Thermal
{
    // Axisymmetric glass and ring, with three individually discretised legs.
    // Properties and contact parameters are declared synthetic S08 inputs.
    public sealed class BeakerSolidMesh
    {
        public const int InnerRadialCells = 16, WallRadialCells = 3, BaseDepthCells = 6;
        public const int BaseRadialCells = InnerRadialCells + WallRadialCells;
        private const int WallHeightCells = 32, LegCells = 12, LegCount = 3;
        private const double GlassK = 1.2, GlassVolumetricCp = 1.8e6;
        private const double SteelK = 16, SteelVolumetricCp = 3.9e6;
        public const int WaterHeightCells = 8;
        private const double ContactAreaM2 = 1e-4, ContactResistanceM2KW = .001;

        public readonly double[] CapacityJPerK;
        public readonly double[] BurnerWeights;
        public readonly SolidConduction.Face[] Faces;
        public int Ring { get; }
        public int LowerWall { get; }
        public int Rim { get; }
        public int LegTop => Ring + 1;
        public int Foot => Ring + LegCells;

        public int WaterStart { get; }
        public double WaterDepthM { get; }
        public double WaterVolumeM3 { get; }

        public BeakerSolidMesh(GeometryInputs geometry, double waterVolumeM3 = 0)
        {
            int wallStart = BaseRadialCells * BaseDepthCells;
            Ring = wallStart + WallHeightCells * WallRadialCells;
            LowerWall = wallStart;
            Rim = Ring - WallRadialCells;
            if (!SiUnits.IsFinite(waterVolumeM3) || waterVolumeM3 < 0 ||
                waterVolumeM3 >= geometry.CavityVolumeM3)
                throw new ArgumentOutOfRangeException(nameof(waterVolumeM3), "Water must fit below the rim.");
            WaterVolumeM3 = waterVolumeM3;
            WaterDepthM = waterVolumeM3 / (Math.PI * geometry.innerRadiusM * geometry.innerRadiusM);
            WaterStart = Ring + 1 + LegCount * LegCells;
            int waterCount = waterVolumeM3 > 0 ? InnerRadialCells * WaterHeightCells : 0;
            CapacityJPerK = new double[WaterStart + waterCount];
            BurnerWeights = new double[CapacityJPerK.Length];
            var faces = new List<SolidConduction.Face>();
            double drWall = geometry.wallThicknessM / WallRadialCells;
            double dzBase = geometry.baseThicknessM / BaseDepthCells;
            double dzWall = geometry.cavityHeightM / WallHeightCells;
            var edges = new double[BaseRadialCells + 1];
            for (int r = 1; r <= InnerRadialCells; r++)
                edges[r] = geometry.innerRadiusM * r / InnerRadialCells;
            for (int r = 1; r <= WallRadialCells; r++)
                edges[InnerRadialCells + r] = geometry.innerRadiusM + r * drWall;
            var baseEdges = new double[BaseRadialCells];
            Array.Copy(edges, 1, baseEdges, 0, baseEdges.Length);
            var burner = RadialHeatFluxBoundary.GaussianFacePowerW(1, .015, baseEdges);

            for (int r = 0; r < BaseRadialCells; r++)
            {
                double area = Annulus(edges[r], edges[r + 1]);
                int bottom = r * BaseDepthCells;
                BurnerWeights[bottom] = burner[r];
                for (int z = 0; z < BaseDepthCells; z++)
                {
                    int cell = bottom + z;
                    CapacityJPerK[cell] = GlassVolumetricCp * area * dzBase;
                    if (z + 1 < BaseDepthCells)
                        faces.Add(new SolidConduction.Face(cell, cell + 1, GlassK * area / dzBase));
                    if (r + 1 < BaseRadialCells)
                        faces.Add(new SolidConduction.Face(cell, cell + BaseDepthCells,
                            GlassK * 2 * Math.PI * edges[r + 1] * dzBase / ((edges[r + 2] - edges[r]) / 2)));
                }
                if (r >= InnerRadialCells)
                    faces.Add(new SolidConduction.Face(bottom + BaseDepthCells - 1,
                        wallStart + r - InnerRadialCells, GlassK * area / ((dzBase + dzWall) / 2)));
            }

            for (int z = 0; z < WallHeightCells; z++)
                for (int r = 0; r < WallRadialCells; r++)
                {
                    int cell = wallStart + z * WallRadialCells + r;
                    double area = Annulus(edges[InnerRadialCells + r], edges[InnerRadialCells + r + 1]);
                    CapacityJPerK[cell] = GlassVolumetricCp * area * dzWall;
                    if (z + 1 < WallHeightCells)
                        faces.Add(new SolidConduction.Face(cell, cell + WallRadialCells, GlassK * area / dzWall));
                    if (r + 1 < WallRadialCells)
                        faces.Add(new SolidConduction.Face(cell, cell + 1,
                            GlassK * 2 * Math.PI * edges[InnerRadialCells + r + 1] * dzWall / drWall));
                }

            double tubeRadius = geometry.tripodTubeRadiusM;
            double section = Math.PI * tubeRadius * tubeRadius;
            CapacityJPerK[Ring] = SteelVolumetricCp * section * (2 * Math.PI * geometry.tripodRingRadiusM);
            // Distribute a synthetic finite contact patch across its overlap with base annuli.
            double patchInner = geometry.tripodRingRadiusM - tubeRadius;
            double patchOuter = geometry.tripodRingRadiusM + tubeRadius;
            double patchArea = Annulus(patchInner, patchOuter);
            for (int r = 0; r < BaseRadialCells; r++)
            {
                double inner = Math.Max(patchInner, edges[r]);
                double outer = Math.Min(patchOuter, edges[r + 1]);
                if (outer <= inner) continue;
                double area = ContactAreaM2 * Annulus(inner, outer) / patchArea;
                double conductance = CoupledHeatFaces.HeatRateW(area, 2, 1,
                    dzBase / 2, GlassK, ContactResistanceM2KW, tubeRadius, SteelK);
                faces.Add(new SolidConduction.Face(r * BaseDepthCells, Ring, conductance));
            }

            double legRise = geometry.externalBaseHeightM - 3 * tubeRadius;
            double legRun = geometry.tripodFootRadiusM - geometry.tripodRingRadiusM;
            double legSpacing = Math.Sqrt(legRise * legRise + legRun * legRun) / LegCells;
            for (int leg = 0; leg < LegCount; leg++)
            {
                int top = LegTop + leg * LegCells;
                faces.Add(new SolidConduction.Face(Ring, top, SteelK * section / (tubeRadius + legSpacing / 2)));
                for (int i = 0; i < LegCells; i++)
                {
                    int cell = top + i;
                    CapacityJPerK[cell] = SteelVolumetricCp * section * legSpacing;
                    if (i + 1 < LegCells)
                        faces.Add(new SolidConduction.Face(cell, cell + 1, SteelK * section / legSpacing));
                }
            }
            if (waterCount > 0)
                AddWater(geometry, edges, wallStart, dzBase, dzWall, drWall, faces);
            Faces = faces.ToArray();
        }

        private void AddWater(GeometryInputs geometry, double[] edges, int wallStart, double dzBase,
            double dzWall, double drWall, List<SolidConduction.Face> faces)
        {
            double dz = WaterDepthM / WaterHeightCells;
            double dr = geometry.innerRadiusM / InnerRadialCells;
            var water = new Water95();
            var state = water.Evaluate(293.15, 101325, WaterBranch.Liquid);
            double volumetricCapacity = state.DensityKgM3 * state.CpJkgK;
            double waterK = water.ConductivityWmK(state);
            for (int r = 0; r < InnerRadialCells; r++)
            {
                double area = Annulus(edges[r], edges[r + 1]);
                int bottom = WaterStart + r * WaterHeightCells;
                double floorG = CoupledHeatFaces.HeatRateW(area, 2, 1, dzBase / 2, GlassK, 0, dz / 2, waterK);
                faces.Add(new SolidConduction.Face(r * BaseDepthCells + BaseDepthCells - 1, bottom, floorG));
                for (int z = 0; z < WaterHeightCells; z++)
                {
                    int cell = bottom + z;
                    CapacityJPerK[cell] = volumetricCapacity * area * dz;
                    if (z + 1 < WaterHeightCells)
                        faces.Add(new SolidConduction.Face(cell, cell + 1, waterK * area / dz));
                    if (r + 1 < InnerRadialCells)
                        faces.Add(new SolidConduction.Face(cell, cell + WaterHeightCells,
                            waterK * 2 * Math.PI * edges[r + 1] * dz / dr));
                }
            }

            for (int z = 0; z < WaterHeightCells; z++)
            {
                int waterCell = WaterStart + (InnerRadialCells - 1) * WaterHeightCells + z;
                for (int wallZ = 0; wallZ < WallHeightCells; wallZ++)
                {
                    double low = Math.Max(z * dz, wallZ * dzWall);
                    double high = Math.Min((z + 1) * dz, (wallZ + 1) * dzWall);
                    if (high <= low) continue;
                    double area = 2 * Math.PI * geometry.innerRadiusM * (high - low);
                    double conductance = CoupledHeatFaces.HeatRateW(area, 2, 1, drWall / 2, GlassK, 0, dr / 2, waterK);
                    faces.Add(new SolidConduction.Face(wallStart + wallZ * WallRadialCells, waterCell, conductance));
                }
            }
        }

        private static double Annulus(double inner, double outer) => Math.PI * (outer - inner) * (outer + inner);
    }
}
