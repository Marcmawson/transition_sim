using System;
using System.Linq;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Thermal;

internal static class ThermalBoundaryMain
{
    private static void Main()
    {
        CheckBeaker(.033, .06, 9);
        CheckBeaker(.06, .16, 250);
        Near(RadialHeatFluxBoundary.GaussianFacePowerW(0, .015, new[] { .06 })[0], 0);
        var broad = RadialHeatFluxBoundary.GaussianFacePowerW(100, 1e10, new[] { .03, .06 });
        Near(broad[0], 25);
        Near(broad[1], 75);
        var narrow = RadialHeatFluxBoundary.GaussianFacePowerW(100, 1e-8, new[] { .03, .06 });
        Near(narrow[0], 100);
        Near(narrow[1], 0);
        double forward = CoupledHeatFaces.HeatRateW(.01, 330, 300, .003, 1.2, .001, .002, .6);
        double backward = CoupledHeatFaces.HeatRateW(.01, 300, 330, .002, .6, .001, .003, 1.2);
        Near(forward, .3 / (.003 / 1.2 + .001 + .002 / .6));
        Near(forward + backward, 0);
        Near(CoupledHeatFaces.HeatRateW(.01, 300, 300, .003, 1.2, 0, .002, .6), 0);
        Console.WriteLine("PASS S08 boundary/reference checks");
    }

    private static void CheckBeaker(double radiusM, double cavityHeightM, double waterMl)
    {
        var geometry = new GeometryInputs { innerRadiusM = radiusM, cavityHeightM = cavityHeightM };
        double capacityMl = geometry.CavityVolumeM3 * 1e6;
        if (waterMl >= capacityMl) throw new Exception("Test sample must leave headspace.");
        const double absorbedW = 100;
        foreach (int faceCount in new[] { 1, 4, 16, 64 })
        {
            var edges = Enumerable.Range(1, faceCount)
                .Select(i => geometry.OuterRadiusM * i / faceCount).ToArray();
            var power = RadialHeatFluxBoundary.GaussianFacePowerW(absorbedW, .015, edges);
            Near(power.Sum(), absorbedW);
            if (power.Any(value => value < 0)) throw new Exception("Negative face power.");
            if (faceCount > 1)
            {
                int mid = faceCount / 2;
                double expectedFraction = (1 - Math.Exp(-.5 * Math.Pow(edges[mid - 1] / .015, 2))) /
                    (1 - Math.Exp(-.5 * Math.Pow(geometry.OuterRadiusM / .015, 2)));
                Near(power.Take(mid).Sum(), absorbedW * expectedFraction);
            }
        }
        Console.WriteLine($"PASS {waterMl:G} mL case, capacity {capacityMl:F1} mL, {absorbedW:G} W on 1/4/16/64 annuli");
    }

    private static void Near(double actual, double expected)
    {
        if (Math.Abs(actual - expected) > 1e-9 || double.IsNaN(actual))
            throw new Exception($"Expected {expected:G17}, got {actual:G17}.");
    }
}
