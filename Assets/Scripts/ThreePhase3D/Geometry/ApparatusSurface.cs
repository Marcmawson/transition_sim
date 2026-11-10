using System;
using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Geometry
{
    // Authoritative analytic dimensions and the preview tessellation are separate.
    // S03 must derive cut-cell geometry from this same surface, not from render colours.
    public sealed class TriangleSurface
    {
        public double[] positionsM;
        public int[] triangles;
        public double SignedVolumeM3()
        {
            double sum = 0, compensation = 0;
            for (int i = 0; i < triangles.Length; i += 3)
            {
                int a = 3 * triangles[i], b = 3 * triangles[i + 1], c = 3 * triangles[i + 2];
                double term = (positionsM[a] * (positionsM[b + 1] * positionsM[c + 2] - positionsM[b + 2] * positionsM[c + 1]) +
                    positionsM[a + 1] * (positionsM[b + 2] * positionsM[c] - positionsM[b] * positionsM[c + 2]) +
                    positionsM[a + 2] * (positionsM[b] * positionsM[c + 1] - positionsM[b + 1] * positionsM[c])) / 6;
                double adjusted = term - compensation, next = sum + adjusted;
                compensation = (next - sum) - adjusted; sum = next;
            }
            return sum;
        }
    }

    public static class ApparatusSurface
    {
        public const string Version = "cylindrical-apparatus-1";

        // Physics-ID: GEOMETRY-BEAKER-SURFACE
        // Implements: R004 R009 R050
        // Model: Specification.md sections 3.1 and 4.1; synthetic axisymmetric vessel.
        // Sources: project-declared geometry, not empirical glass property data.
        // Inputs-SI: radius, heights, base/wall thickness [m]; angular subdivisions [1].
        // Outputs-SI: outward-oriented closed glass triangle surface [m]; cavity is open above rim.
        // State-Ownership: newly allocated preview surface only.
        // Conservation: no phase inventory; base and walls form one closed glass volume.
        // Discretisation: polygonal revolution, finite angular resolution; no cut cells yet.
        // Validity: positive cylindrical dimensions; no pouring spout in this synthetic case.
        // Validation: V01 V36; V06 cut-cell verification remains future work.
        // Evidence: PENDING
        // Status: IMPLEMENTED_UNVERIFIED
        // Limitations: tessellated volume differs from analytic cylinder by O(dtheta^2).
        public static TriangleSurface Beaker(GeometryInputs g, int segments)
        {
            if (g == null || segments < 3) throw new ArgumentException("Geometry and at least three segments required.");
            return Revolve(new[] { 0.0, g.OuterRadiusM, g.OuterRadiusM, g.innerRadiusM, g.innerRadiusM, 0.0 },
                new[] { g.externalBaseHeightM, g.externalBaseHeightM, g.RimHeightM, g.RimHeightM,
                    g.externalBaseHeightM + g.baseThicknessM, g.externalBaseHeightM + g.baseThicknessM }, segments);
        }

        public static TriangleSurface SupportRing(GeometryInputs g, int segments)
        {
            const int tubeSegments = 16;
            var r = new double[tubeSegments + 1]; var y = new double[tubeSegments + 1];
            for (int i = 0; i <= tubeSegments; i++)
            {
                double angle = 2 * Math.PI * i / tubeSegments;
                r[i] = g.tripodRingRadiusM + g.tripodTubeRadiusM * Math.Cos(angle);
                y[i] = g.externalBaseHeightM - g.tripodTubeRadiusM + g.tripodTubeRadiusM * Math.Sin(angle);
            }
            return Revolve(r, y, segments);
        }

        static TriangleSurface Revolve(double[] radius, double[] height, int segments)
        {
            var vertices = new List<double>(); var indices = new List<int>();
            for (int k = 0; k + 1 < radius.Length; k++)
                for (int j = 0; j < segments; j++)
                {
                    double t0 = 2 * Math.PI * j / segments, t1 = 2 * Math.PI * (j + 1) / segments;
                    int a = Add(vertices, radius[k], height[k], t0);
                    int b = Add(vertices, radius[k + 1], height[k + 1], t0);
                    int c = Add(vertices, radius[k + 1], height[k + 1], t1);
                    int d = Add(vertices, radius[k], height[k], t1);
                    if (radius[k + 1] > 0) { indices.Add(a); indices.Add(b); indices.Add(c); }
                    if (radius[k] > 0) { indices.Add(a); indices.Add(c); indices.Add(d); }
                }
            return new TriangleSurface { positionsM = vertices.ToArray(), triangles = indices.ToArray() };
        }
        static int Add(List<double> vertices, double radius, double height, double angle)
        { int index = vertices.Count / 3; vertices.Add(radius * Math.Cos(angle)); vertices.Add(height); vertices.Add(radius * Math.Sin(angle)); return index; }
    }
}
