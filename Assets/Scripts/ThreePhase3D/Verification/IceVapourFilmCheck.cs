using System;
using System.Linq;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.Materials;

namespace PhysicsSim.ThreePhase3D.Verification
{
    internal static class IceVapourFilmCheck
    {
        internal static void Run(Action<string> log)
        {
            const double pressure = 101325, diffusivity = 2e-5, film = 0.001, area = 1e-4, dt = 0.01;
            var humid = new HumidGas();
            foreach (double temperature in new[] { 253.15, 263.15 })
            {
                var surface = humid.EquilibriumWithIce(temperature, pressure);
                double saturated = surface.WaterMassFraction;
                // Prescribed dilute-vapour caloric model, shared reference with the S14 fixture.
                double vapourH = DropletGasCellExchange.VapourReferenceEnthalpy +
                    DropletGasCellExchange.VapourCp * (temperature - DropletGasCellExchange.ReferenceK);
                foreach (double ratio in new[] { 0.5, 1.0, 1.5 })
                {
                    double ambient = saturated * ratio;
                    double reservoirDensity = pressure /
                        (temperature * (8.31451 * (1 - ambient) / HumidGas.Ma + Water95.R * ambient));
                    const double dryAir = 1e-5, ice = 1e-4;
                    double vapour = dryAir * ambient / (1 - ambient);
                    var result = IceVapourFilm.Exchange(surface, ambient, diffusivity, film, area, dt, ice, vapour, vapourH);
                    double flux = result.VapourKg / (area * dt);
                    double initialThickness = ice / (surface.Ice.DensityKgM3 * area);
                    double finalThickness = initialThickness + result.PhaseFrontShiftM;
                    if (Math.Sign(result.PhaseFrontShiftM) != Math.Sign(ratio - 1) ||
                        Math.Abs(finalThickness * area * surface.Ice.DensityKgM3 + result.VapourKg - ice) > 1e-18)
                        throw new InvalidOperationException("V41 planar front did not match the ice mass balance.");
                    var grid = new Grid3D(4, 4, 4, new D3(0.005, 0.005, 0.005));
                    var lower = new D3(0.005, 0.005, 0.005);
                    CutCellGeometry BuildSlab(double thickness) => CutCellGeometry.Build(grid, iceSurface:
                        SurfacePrimitives.Box(lower, new D3(0.015, lower.Y + thickness, 0.015)));
                    var before = BuildSlab(initialThickness);
                    var after = BuildSlab(finalThickness);
                    double expectedIceChange = -result.VapourKg / surface.Ice.DensityKgM3;
                    double iceChange = after.IceVolumeM3.Sum() - before.IceVolumeM3.Sum();
                    double fluidChange = after.FluidVolumeM3.Sum() - before.FluidVolumeM3.Sum();
                    double volumeTolerance = Math.Max(1e-20, 1e-5 * Math.Abs(expectedIceChange));
                    if (Math.Abs(iceChange - expectedIceChange) > volumeTolerance ||
                        Math.Abs(fluidChange + expectedIceChange) > volumeTolerance)
                        throw new InvalidOperationException("V41 rebuilt geometry did not close the ice and gas volumes.");
                    var gasCells = before.FluidVolumeM3.Select(volume =>
                        (DryAirKg: volume * reservoirDensity * (1 - ambient), VapourKg: volume * reservoirDensity * ambient,
                            EnthalpyJ: volume * 1e4)).ToArray();
                    var remapped = IceVapourFilm.RemapPlanarGas(before, after, gasCells,
                        result.VapourKg, result.GasEnthalpyJ);
                    for (int cell = 0; cell < gasCells.Length; cell++)
                        if (remapped[cell].DryAirKg != gasCells[cell].DryAirKg ||
                            (before.FluidVolumeM3[cell] == after.FluidVolumeM3[cell] && remapped[cell] != gasCells[cell]))
                            throw new InvalidOperationException("V41 gas remap changed dry air or an untouched cell.");
                    if (Math.Abs(remapped.Sum(cell => cell.VapourKg) - gasCells.Sum(cell => cell.VapourKg) -
                            result.VapourKg) > 1e-20 ||
                        Math.Abs(remapped.Sum(cell => cell.EnthalpyJ) - gasCells.Sum(cell => cell.EnthalpyJ) -
                            result.GasEnthalpyJ) > 1e-15)
                        throw new InvalidOperationException("V41 gas remap did not conserve vapour or enthalpy.");
                    CheckRedistribution(after, before, remapped, temperature, dt, ambient, pressure,
                        reservoirDensity, result.VapourKg, log);
                    // Integrate the constant-flux species ODE across the film and
                    // compare the recovered ambient composition to its boundary value.
                    double recovered = 1 - (1 - saturated) *
                        Math.Exp(flux * film / (surface.Gas.DensityKgM3 * diffusivity));
                    double updatedVapour = vapour + result.VapourKg;
                    double updatedGasMass = dryAir + vapour + result.VapourKg;
                    if (Math.Sign(flux) != Math.Sign(1 - ratio) || Math.Abs(recovered - ambient) > 1e-15 ||
                        Math.Abs((ice - result.VapourKg) + updatedVapour - ice - vapour) > 1e-18 ||
                        Math.Abs(updatedGasMass - updatedVapour - dryAir) > 1e-18)
                        throw new InvalidOperationException("V41 species flux or water balance failed.");
                    if (Math.Sign(result.SurfaceHeatJ) != Math.Sign(flux) ||
                        Math.Abs(result.IceEnthalpyJ + result.GasEnthalpyJ - result.SurfaceHeatJ) > 1e-15)
                        throw new InvalidOperationException("V41 surface heat ledger failed.");
                    log(FormattableString.Invariant(
                        $"S15 V41 ice-vapour film passed: {temperature:F2} K, Y/Yice {ratio:F1}, flux {flux:G8} kg/m2/s, surface heat {result.SurfaceHeatJ:G8} J, front shift {result.PhaseFrontShiftM:G8} m."));
                }
            }
            const double initialK = 263.15, initialIce = 1e-7, dryAirKg = 1e-5;
            var start = humid.EquilibriumWithIce(initialK, pressure);
            var iceModel = new IceIh();
            foreach (double humidity in new[] { 0.5, 1.5 })
            {
                var coarse = RunTransient(8);
                var fine = RunTransient(16);
                var reference = RunTransient(32);
                if (Math.Sign(reference.Temperature - initialK) != Math.Sign(humidity - 1) ||
                    Math.Sign(reference.Vapour) != Math.Sign(1 - humidity) ||
                    Math.Abs(fine.Temperature - reference.Temperature) >= Math.Abs(coarse.Temperature - reference.Temperature))
                    throw new InvalidOperationException("V41 transient ice exchange did not warm, cool or refine as expected.");
                log(FormattableString.Invariant(
                    $"S15 V41 transient ice passed: Y/Yice {humidity:F1}, final {reference.Temperature:F6} K, vapour change {reference.Vapour:G8} kg."));

                (double Temperature, double Vapour) RunTransient(int steps)
                {
                    double initialFraction = humidity * start.WaterMassFraction;
                    double initialVapour = dryAirKg * initialFraction / (1 - initialFraction);
                    double vapour = initialVapour, ice = initialIce, kelvin = initialK, gasEnergy = 0, heat = 0;
                    double initialEnergy = ice * start.Ice.EnthalpyJkg;
                    for (int step = 0; step < steps; step++)
                    {
                        var surface = humid.EquilibriumWithIce(kelvin, pressure);
                        double vapourH = DropletGasCellExchange.VapourReferenceEnthalpy +
                            DropletGasCellExchange.VapourCp * (kelvin - DropletGasCellExchange.ReferenceK);
                        var next = IceVapourFilm.AdvanceLumped(surface, vapour / (dryAirKg + vapour),
                            diffusivity, film, area, 0.02 / steps, ice, vapour, vapourH, initialK, 25);
                        ice = next.IceKg;
                        vapour += next.VapourKg;
                        gasEnergy += next.GasEnthalpyJ;
                        heat += next.HeatIntoIceJ;
                        double target = next.IceEnthalpyJ / ice;
                        double low = 250, high = 270;
                        if (!(target >= iceModel.Evaluate(low, pressure).EnthalpyJkg &&
                            target <= iceModel.Evaluate(high, pressure).EnthalpyJkg))
                            throw new InvalidOperationException("V41 ice enthalpy left the reference bracket.");
                        for (int iteration = 0; iteration < 45; iteration++)
                        {
                            double middle = (low + high) * 0.5;
                            if (iceModel.Evaluate(middle, pressure).EnthalpyJkg < target) low = middle;
                            else high = middle;
                        }
                        kelvin = (low + high) * 0.5;
                        if (Math.Abs(ice + vapour - initialIce - initialVapour) > 1e-19 ||
                            Math.Abs(ice * iceModel.Evaluate(kelvin, pressure).EnthalpyJkg + gasEnergy - initialEnergy - heat) > 1e-13)
                            throw new InvalidOperationException("V41 transient water or enthalpy ledger failed.");
                    }
                    return (kelvin, vapour - initialVapour);
                }
            }
        }

        private static void CheckRedistribution(CutCellGeometry after, CutCellGeometry before,
            (double DryAirKg, double VapourKg, double EnthalpyJ)[] remapped, double temperature, double dt,
            double ambient, double pressure, double reservoirDensity, double transferredVapour, Action<string> log)
        {
            var sides = new FlowBoundary[6];
            sides[3] = new FlowBoundary(FlowBoundaryKind.OpenPressure);
            var plan = new VariableCoefficientProjection(after, new BoundaryConditions3D(sides), _ => reservoirDensity, _ => 0);
            var gas = new (double DryAirKg, double VapourKg, double EnthalpyJ, D3 Momentum)[plan.GroupCount];
            var volumes = new double[plan.GroupCount];
            // Prescribed temperature, ideal mixture. Use the geometric initial
            // volume and add the partial volume of the transferred water vapour.
            double specificVapourVolume = Water95.R * temperature / pressure;
            for (int cell = 0; cell < remapped.Length; cell++)
            {
                int group = plan.GroupByCell[cell];
                if (group < 0) continue;
                var local = remapped[cell];
                double originalVolume = before.FluidVolumeM3[cell];
                double addedVapour = local.VapourKg - originalVolume * reservoirDensity * ambient;
                volumes[group] += originalVolume + specificVapourVolume * addedVapour;
                gas[group].DryAirKg += local.DryAirKg;
                gas[group].VapourKg += local.VapourKg;
                gas[group].EnthalpyJ += local.EnthalpyJ;
            }
            var source = new double[plan.GroupCount];
            for (int group = 0; group < source.Length; group++)
                source[group] = (plan.Volumes[group] - volumes[group]) / dt;
            var flow = FlowCpuReference.Project(plan, new float[plan.FaceCount], dt, 32768, null, source);
            var moved = HumidGasAdvection.Advance(plan, flow, volumes, gas,
                (reservoirDensity * (1 - ambient), reservoirDensity * ambient, 1e4, default));
            double outwardVolume = 0;
            for (int face = 0; face < plan.FaceCount; face++)
            {
                var metric = plan.Faces[face];
                if (metric.Active != 0 && metric.High < 0)
                    outwardVolume += metric.Area * (double)flow.VelocityMPerS[face] * dt;
            }
            double expected = transferredVapour * specificVapourVolume -
                (after.FluidVolumeM3.Sum() - before.FluidVolumeM3.Sum());
            if (Math.Abs(outwardVolume - expected) > Math.Max(1e-19, Math.Abs(expected) * 1e-5) ||
                Math.Abs(moved.Gas.Sum(cell => cell.DryAirKg) + moved.Outflow.DryAirKg - gas.Sum(cell => cell.DryAirKg)) > 1e-19 ||
                Math.Abs(moved.Gas.Sum(cell => cell.VapourKg) + moved.Outflow.VapourKg - gas.Sum(cell => cell.VapourKg)) > 1e-19 ||
                Math.Abs(moved.Gas.Sum(cell => cell.EnthalpyJ) + moved.Outflow.EnthalpyJ - gas.Sum(cell => cell.EnthalpyJ)) > 1e-15)
                throw new InvalidOperationException("V41 pressure redistribution failed its boundary balance.");
            log(FormattableString.Invariant($"S15 V41 gas redistribution passed: outward volume {outwardVolume:G8} m3."));
        }
    }
}
