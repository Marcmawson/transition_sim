using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Geometry;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // Physics-ID: FLOW-GROUPED-MAC-PROJECTION
    // Implements: R019
    // Model: Specification.md sections 6.3 and 9; integrated finite-volume pressure correction.
    // Sources: project Specification.md, Validation.md and Sources.md M02/M10.
    // Source-Locators: Specification.md section 6.3 pressure projection equation; S04-Single-Phase-Flow.md.
    // Inputs-SI: cut volumes [m3], face apertures [m2], density [kg/m3], gauge pressure [Pa].
    // Outputs-SI: face velocity [m/s], pressure [Pa], volume-constraint residual [1/s].
    // State-Ownership: immutable pressure topology and coefficients for a stationary geometry.
    // Conservation: one flux per face; internal merged-group faces cancel exactly.
    // Discretisation: face-area-weighted MAC divergence and two-point pressure gradient.
    // Validity: stationary geometry, zero expansion, prescribed positive density.
    // Validation: V07 V08 V35
    // Evidence: ValidationRuns/s04-flow-20261111/cpu-results.txt
    // Status: IMPLEMENTED_UNVERIFIED
    // Limitations: Unity device execution and expansion coupling remain open.
    public sealed class VariableCoefficientProjection
    {
        [StructLayout(LayoutKind.Sequential)]
        internal struct Face
        {
            public int Low, High, Axis, Active;
            public float Area, Mobility, BoundaryPressure, Viscosity;
        }

        internal readonly Grid3D Grid;
        internal readonly CutCellGeometry Geometry;
        internal readonly BoundaryConditions3D Boundaries;
        internal readonly Face[] Faces;
        internal readonly double[] Volumes;
        internal readonly float[] Diagonal;
        internal readonly int[] Rows, Incidence, GroupByCell, FaceOffsets;
        internal readonly bool HasEmbeddedSolid, HasVariableProperties;
        private readonly List<int[]> closedComponents = new List<int[]>();
        // Apertures at floating-point clipping roundoff cannot establish hydraulic
        // connectivity. Apply the same threshold when merging and assembling faces.
        private const double RoundoffFaceFraction = 64 * 2.2204460492503131e-16;
        public double RoundoffClosedAreaM2 { get; }
        public int FaceCount => Faces.Length;
        public int GroupCount => Volumes.Length;

        // The property callback supplies dynamic viscosity, not kinematic viscosity.
        public VariableCoefficientProjection(CutCellGeometry geometry, BoundaryConditions3D boundaries,
            Func<int, double> densityKgM3, Func<int, double> viscosityPaS)
            : this(geometry, boundaries, densityKgM3, viscosityPaS, false) { }

        internal VariableCoefficientProjection(CutCellGeometry geometry, BoundaryConditions3D boundaries,
            Func<int, double> densityKgM3, Func<int, double> viscosityPaS, bool periodic)
        {
            if (geometry == null || boundaries == null || densityKgM3 == null || viscosityPaS == null)
                throw new ArgumentNullException("Flow geometry, boundaries and material callbacks are required.");
            Grid = geometry.Grid;
            Geometry = geometry;
            Boundaries = boundaries;
            HasEmbeddedSolid = geometry.Patches.Count != 0;
            if (periodic && HasEmbeddedSolid) throw new NotSupportedException("Periodic pressure requires regular geometry.");
            var merge = new TinyCellMergePlan(geometry, .05, RoundoffFaceFraction);
            var roots = new Dictionary<int, int>();
            var volumes = new List<double>();
            GroupByCell = new int[Grid.Cells.Count];
            var density = new double[Grid.Cells.Count];
            var viscosity = new double[Grid.Cells.Count];
            double firstDensity = 0, firstViscosity = 0;
            for (int cell = 0; cell < GroupByCell.Length; cell++)
            {
                GroupByCell[cell] = -1;
                if (geometry.FluidVolumeM3[cell] <= 0) continue;
                int root = merge.Group[cell];
                if (!roots.TryGetValue(root, out int group))
                {
                    group = volumes.Count;
                    roots.Add(root, group);
                    volumes.Add(merge.GroupFluidVolumeM3[root]);
                }
                GroupByCell[cell] = group;
                density[cell] = densityKgM3(cell);
                viscosity[cell] = viscosityPaS(cell);
                PositiveFloat(density[cell]);
                if (!SiUnits.IsFinite(viscosity[cell]) || viscosity[cell] < 0 || viscosity[cell] > float.MaxValue)
                    throw new ArgumentException("Dynamic viscosity must be finite and nonnegative.");
                if (firstDensity == 0)
                {
                    firstDensity = density[cell];
                    firstViscosity = viscosity[cell];
                }
                else if (density[cell] != firstDensity || viscosity[cell] != firstViscosity)
                    HasVariableProperties = true;
            }
            if (volumes.Count == 0) throw new ArgumentException("The domain contains no fluid control volumes.");
            Volumes = volumes.ToArray();
            FaceOffsets = new[] { 0, Grid.XFaces.Count, checked(Grid.XFaces.Count + Grid.YFaces.Count) };
            Faces = new Face[checked(FaceOffsets[2] + Grid.ZFaces.Count)];
            var links = new List<int>[GroupCount];
            for (int group = 0; group < GroupCount; group++) links[group] = new List<int>();

            for (int axis = 0; axis < 3; axis++)
            {
                var shape = Grid.Shape((GridLocation)(axis + 1));
                var areas = geometry.Areas(axis);
                double roundoffArea = Grid.FaceArea(axis) * RoundoffFaceFraction;
                for (int index = 0; index < shape.Count; index++)
                {
                    shape.Coordinates(index, out int i, out int j, out int k);
                    if (!shape.Interior(i, j, k) || areas[index] <= 0) continue;
                    if (areas[index] <= roundoffArea)
                    {
                        RoundoffClosedAreaM2 += areas[index];
                        continue;
                    }
                    int lowCell = Cell(i - (axis == 0 ? 1 : 0), j - (axis == 1 ? 1 : 0), k - (axis == 2 ? 1 : 0));
                    int highCell = Cell(i, j, k);
                    if (periodic)
                    {
                        // The positive seam owns the face; its negative alias
                        // stays inactive in the padded S04 arrays.
                        if (lowCell < 0) continue;
                        if (highCell < 0)
                            highCell = Cell(axis == 0 ? 0 : i, axis == 1 ? 0 : j, axis == 2 ? 0 : k);
                    }
                    int low = lowCell < 0 ? -1 : GroupByCell[lowCell];
                    int high = highCell < 0 ? -1 : GroupByCell[highCell];
                    bool exterior = lowCell < 0 || highCell < 0;
                    if ((lowCell >= 0 && low < 0) || (highCell >= 0 && high < 0) || (!periodic && low == high)) continue;
                    var side = exterior ? boundaries.Side(axis, highCell < 0) : default;
                    if (exterior && side.Kind != FlowBoundaryKind.OpenPressure) continue;

                    double rho = exterior ? density[Math.Max(lowCell, highCell)] : .5 * (density[lowCell] + density[highCell]);
                    double mu = exterior ? viscosity[Math.Max(lowCell, highCell)] : .5 * (viscosity[lowCell] + viscosity[highCell]);
                    int face = FaceOffsets[axis] + index;
                    Faces[face] = new Face
                    {
                        Low = low, High = high, Axis = axis, Active = 1,
                        Area = PositiveFloat(areas[index]),
                        Mobility = PositiveFloat(1 / (rho * Grid.Spacing[axis] * (exterior ? .5 : 1))),
                        BoundaryPressure = (float)side.GaugePressurePa,
                        Viscosity = mu == 0 ? 0 : PositiveFloat(mu / rho)
                    };
                    if (float.IsInfinity(Faces[face].BoundaryPressure)) throw new ArgumentException("Boundary pressure exceeds FP32.");
                    if (low == high) continue;
                    if (low >= 0) links[low].Add(face + 1);
                    if (high >= 0) links[high].Add(-face - 1);
                }
            }

            Rows = new int[GroupCount + 1];
            Diagonal = new float[GroupCount];
            var incidence = new List<int>();
            for (int group = 0; group < GroupCount; group++)
            {
                Rows[group] = incidence.Count;
                foreach (int signed in links[group])
                {
                    var face = Faces[Math.Abs(signed) - 1];
                    Diagonal[group] += face.Area * face.Mobility;
                    incidence.Add(signed);
                }
                if (float.IsInfinity(Diagonal[group])) throw new ArgumentException("Pressure diagonal exceeds FP32.");
            }
            Rows[GroupCount] = incidence.Count;
            Incidence = incidence.ToArray();
            var visited = new bool[GroupCount];
            for (int start = 0; start < GroupCount; start++)
            {
                if (visited[start]) continue;
                var component = new List<int> { start };
                visited[start] = true;
                bool open = false;
                for (int cursor = 0; cursor < component.Count; cursor++)
                {
                    int group = component[cursor];
                    for (int entry = Rows[group]; entry < Rows[group + 1]; entry++)
                    {
                        int signed = Incidence[entry];
                        Face face = Faces[Math.Abs(signed) - 1];
                        int other = signed > 0 ? face.High : face.Low;
                        if (other < 0) { open = true; continue; }
                        if (visited[other]) continue;
                        visited[other] = true;
                        component.Add(other);
                    }
                }
                if (!open) closedComponents.Add(component.ToArray());
            }
        }

        public int FaceIndex(int axis, int i, int j, int k) =>
            Grid.Shape((GridLocation)(axis + 1)).Index(i, j, k) + FaceOffsets[axis];

        public int PressureGroup(int i, int j, int k) => GroupByCell[Grid.Cells.Index(i, j, k)];

        private int Cell(int i, int j, int k) => Grid.Cells.Interior(i, j, k) ? Grid.Cells.Index(i, j, k) : -1;

        private static float PositiveFloat(double value)
        {
            float result = (float)value;
            if (!(result > 0) || float.IsInfinity(result)) throw new ArgumentException("Flow coefficient is outside positive FP32 range.");
            return result;
        }

        // Closed components have exactly zero net imposed flux: all active faces
        // are shared, and all exterior walls have zero normal velocity. Jacobi
        // solves on the compatible subspace. Fix each reported gauge independently.
        internal void SetClosedGauges(double[] pressure)
        {
            foreach (int[] component in closedComponents)
            {
                double sum = 0, volume = 0;
                foreach (int group in component)
                {
                    sum += pressure[group] * Volumes[group];
                    volume += Volumes[group];
                }
                double mean = sum / volume;
                foreach (int group in component) pressure[group] -= mean;
            }
        }

        internal double OutwardFlux(int group, float[] velocity)
        {
            double flux = 0;
            for (int entry = Rows[group]; entry < Rows[group + 1]; entry++)
            {
                int signed = Incidence[entry], index = Math.Abs(signed) - 1;
                flux += Math.Sign(signed) * (double)Faces[index].Area * velocity[index];
            }
            return flux;
        }

        internal double Rhs(int group, float[] predicted, double dt)
        {
            double rhs = 0;
            for (int entry = Rows[group]; entry < Rows[group + 1]; entry++)
            {
                int signed = Incidence[entry], index = Math.Abs(signed) - 1;
                Face face = Faces[index];
                rhs -= Math.Sign(signed) * (double)face.Area * predicted[index] / dt;
                if (face.Low < 0 || face.High < 0)
                    rhs += (double)face.Area * face.Mobility * face.BoundaryPressure;
            }
            return rhs;
        }

        internal double Gradient(Face face, double[] pressure)
        {
            double low = face.Low < 0 ? face.BoundaryPressure : pressure[face.Low];
            double high = face.High < 0 ? face.BoundaryPressure : pressure[face.High];
            return face.Mobility * (high - low);
        }

        internal FlowDiagnostics Measure(float[] predicted, float[] corrected, double[] pressure, double dt, int iterations)
        {
            double residual2 = 0, rhs2 = 0, maxDivergence = 0;
            for (int group = 0; group < GroupCount; group++)
            {
                double flux = 0, residual = 0;
                for (int entry = Rows[group]; entry < Rows[group + 1]; entry++)
                {
                    int signed = Incidence[entry], index = Math.Abs(signed) - 1;
                    var face = Faces[index];
                    double area = Math.Sign(signed) * (double)face.Area;
                    flux += area * corrected[index];
                    residual += area * ((double)predicted[index] / dt - Gradient(face, pressure));
                }
                double rhs = Rhs(group, predicted, dt);
                rhs2 += rhs * rhs;
                residual2 += residual * residual;
                maxDivergence = Math.Max(maxDivergence, Math.Abs(flux) / Volumes[group]);
            }
            return new FlowDiagnostics(iterations, Math.Sqrt(residual2), Math.Sqrt(rhs2), maxDivergence, dt);
        }
    }

    public readonly struct FlowDiagnostics
    {
        public readonly int Iterations;
        public readonly double ResidualL2, RhsL2, MaxDivergencePerS, Dt;
        public FlowDiagnostics(int iterations, double residual, double rhs, double divergence, double dt)
        {
            Iterations = iterations;
            ResidualL2 = residual;
            RhsL2 = rhs;
            MaxDivergencePerS = divergence;
            Dt = dt;
        }
        public bool Converged => SiUnits.IsFinite(ResidualL2) && SiUnits.IsFinite(RhsL2) &&
            MaxDivergencePerS * Dt <= 1e-5 && (RhsL2 == 0 || ResidualL2 <= 1e-6 * RhsL2);
    }
}
