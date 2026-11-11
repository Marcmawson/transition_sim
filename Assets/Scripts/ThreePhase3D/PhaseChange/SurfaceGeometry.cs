using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;

namespace PhysicsSim.ThreePhase3D.PhaseChange
{
    public enum CurvatureSupport
    {
        PurePhase, HeightFunction, AlternateAxis, Unresolved
    }

    public readonly struct SurfaceCurvature
    {
        public readonly CurvatureSupport Support;
        public readonly double InverseMetres;

        internal SurfaceCurvature(CurvatureSupport support, double inverseMetres)
        {
            Support = support;
            InverseMetres = inverseMetres;
        }
    }

    // Regular periodic VOF height functions. Positive curvature denotes a
    // convex liquid surface. See Sources.md M01 and S06-SurfaceGeometry.md.
    public static class SurfaceGeometry
    {
        private const int Radius = 3;
        private const double FractionTolerance = 1e-12;

        public static SurfaceCurvature[] Evaluate(GeometricVof3D transport, double[] liquidFraction)
        {
            if (transport == null) throw new ArgumentNullException(nameof(transport));
            if (liquidFraction == null || liquidFraction.Length != transport.CellCount)
                throw new ArgumentException("Expected one liquid fraction per periodic cell.", nameof(liquidFraction));
            foreach (double fraction in liquidFraction)
                if (!SiUnits.IsFinite(fraction) || fraction < 0 || fraction > 1)
                    throw new ArgumentException("Liquid fractions must be finite and in [0, 1].", nameof(liquidFraction));

            var grid = transport.Grid;
            var shape = grid.Cells;
            var sizes = new[] { shape.X, shape.Y, shape.Z };
            var result = new SurfaceCurvature[transport.CellCount];
            var origin = new int[3];
            var slopes = new double[3];
            var heights = new double[9];

            double Sample(int axis, int u, int axial, int transverseU, int transverseV)
            {
                int i = origin[0], j = origin[1], k = origin[2];
                i += axis == 0 ? axial : u == 0 ? transverseU : transverseV;
                j += axis == 1 ? axial : u == 1 ? transverseU : transverseV;
                k += axis == 2 ? axial : u == 2 ? transverseU : transverseV;
                return liquidFraction[transport.CellIndex((i + sizes[0]) % sizes[0],
                    (j + sizes[1]) % sizes[1], (k + sizes[2]) % sizes[2])];
            }

            bool TryHeights(int axis, int u, int v, int direction)
            {
                if (sizes[axis] < 2 * Radius + 1 || sizes[u] < 3 || sizes[v] < 3) return false;
                for (int b = -1; b <= 1; b++)
                for (int a = -1; a <= 1; a++)
                {
                    double previous = Sample(axis, u, -Radius * direction, a, b);
                    if (previous < 1 - FractionTolerance) return false;
                    double sum = previous;
                    for (int s = -Radius + 1; s <= Radius; s++)
                    {
                        double fraction = Sample(axis, u, s * direction, a, b);
                        if (fraction > previous + FractionTolerance) return false;
                        sum += fraction;
                        previous = fraction;
                    }
                    if (previous > FractionTolerance) return false;
                    heights[(b + 1) * 3 + a + 1] = sum * grid.Spacing[axis];
                }
                return true;
            }

            for (int k = 0; k < shape.Z; k++)
            for (int j = 0; j < shape.Y; j++)
            for (int i = 0; i < shape.X; i++)
            {
                int cell = transport.CellIndex(i, j, k);
                double fraction = liquidFraction[cell];
                if (fraction == 0 || fraction == 1)
                {
                    result[cell] = new SurfaceCurvature(CurvatureSupport.PurePhase, double.NaN);
                    continue;
                }

                origin[0] = i;
                origin[1] = j;
                origin[2] = k;
                for (int axis = 0; axis < 3; axis++)
                {
                    int u = (axis + 1) % 3;
                    slopes[axis] = (Sample(axis, u, -1, 0, 0) -
                        Sample(axis, u, 1, 0, 0)) / grid.Spacing[axis];
                }

                result[cell] = new SurfaceCurvature(CurvatureSupport.Unresolved, double.NaN);
                for (int attempt = 0; attempt < 3; attempt++)
                {
                    int axis = 0;
                    for (int candidate = 1; candidate < 3; candidate++)
                        if (Math.Abs(slopes[candidate]) > Math.Abs(slopes[axis])) axis = candidate;
                    double slope = slopes[axis];
                    if (slope == 0) break;
                    slopes[axis] = 0;
                    int u = (axis + 1) % 3, v = (axis + 2) % 3;
                    if (!TryHeights(axis, u, v, Math.Sign(slope))) continue;

                    double du = grid.Spacing[u], dv = grid.Spacing[v];
                    double hu = (heights[5] - heights[3]) / (2 * du);
                    double hv = (heights[7] - heights[1]) / (2 * dv);
                    double huu = (heights[5] - 2 * heights[4] + heights[3]) / (du * du);
                    double hvv = (heights[7] - 2 * heights[4] + heights[1]) / (dv * dv);
                    double huv = (heights[8] - heights[6] - heights[2] + heights[0]) / (4 * du * dv);
                    double curvature = -(huu * (1 + hv * hv) + hvv * (1 + hu * hu) - 2 * hu * hv * huv) /
                        Math.Pow(1 + hu * hu + hv * hv, 1.5);
                    result[cell] = new SurfaceCurvature(attempt == 0 ? CurvatureSupport.HeightFunction :
                        CurvatureSupport.AlternateAxis, curvature);
                    break;
                }
            }
            return result;
        }
    }
}
