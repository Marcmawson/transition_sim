using System;
using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Core;

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
        private const double ContactAreaM2 = 1e-4, ContactResistanceM2KW = .001;

        public readonly double[] CapacityJPerK;
        public readonly double[] BurnerWeights;
        public readonly SolidConduction.Face[] Faces;
        public int Ring { get; }
        public int LowerWall { get; }
        public int Rim { get; }
        public int LegTop => Ring + 1;
        public int Foot => Ring + LegCells;

        public BeakerSolidMesh(GeometryInputs geometry)
        {
            int wallStart = BaseRadialCells * BaseDepthCells;
            Ring = wallStart + WallHeightCells * WallRadialCells;
            LowerWall = wallStart;
            Rim = Ring - WallRadialCells;
            CapacityJPerK = new double[Ring + 1 + LegCount * LegCells];
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
            Faces = faces.ToArray();
        }

        private static double Annulus(double inner, double outer) => Math.PI * (outer - inner) * (outer + inner);
    }
}
