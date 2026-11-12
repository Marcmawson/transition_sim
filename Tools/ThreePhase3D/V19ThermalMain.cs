using System;
using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Thermal;

internal static class V19ThermalMain
{
    private const double ColdK = 299.5;
    private const double HotK = 300.5;
    private const double Conductivity = 0.026;
    private const double VolumetricHeatCapacity = 1200;

    private static void Main()
    {
        foreach (int resolution in new[] { 8, 16, 32 })
            CheckConduction(resolution);
    }

    private static void CheckConduction(int n)
    {
        // A unit square extruded through unit depth. The front/back and
        // top/bottom faces are insulated, so only in-plane faces carry heat.
        double h = 1.0 / n;
        double cellCapacity = VolumetricHeatCapacity * h * h;
        double wallConductance = 2 * Conductivity;
        var capacities = new double[n * n];
        var temperatures = new double[n * n];
        var power = new double[n * n];
        var faces = new List<SolidConduction.Face>();
        for (int y = 0; y < n; y++)
        for (int x = 0; x < n; x++)
        {
            int cell = y * n + x;
            capacities[cell] = cellCapacity;
            temperatures[cell] = ColdK + 0.25 * (HotK - ColdK);
            if (x + 1 < n) faces.Add(new SolidConduction.Face(cell, cell + 1, Conductivity));
            if (y + 1 < n) faces.Add(new SolidConduction.Face(cell, cell + n, Conductivity));
        }

        var conduction = new SolidConduction(capacities, temperatures, faces.ToArray());
        var state = conduction.TemperatureK;
        // A side-wall cell has at most three interior neighbours plus
        // the half-cell-distance wall conductance.
        double dt = cellCapacity / (5 * Conductivity);
        double duration = 2 * VolumetricHeatCapacity / Conductivity;
        int steps = (int)Math.Ceiling(duration / dt);
        double netInput = 0;
        double maxTemperatureError = 0;
        for (int step = 0; step < steps; step++)
        {
            for (int y = 0; y < n; y++)
            {
                int left = y * n;
                int right = left + n - 1;
                power[left] = wallConductance * (HotK - state[left]);
                power[right] = wallConductance * (ColdK - state[right]);
                netInput += dt * (power[left] + power[right]);
            }
            conduction.Step(dt, power);
        }

        double hotPower = 0;
        double coldPower = 0;
        double stored = 0;
        for (int y = 0; y < n; y++)
        {
            hotPower += wallConductance * (HotK - state[y * n]);
            coldPower += wallConductance * (state[y * n + n - 1] - ColdK);
            for (int x = 0; x < n; x++)
            {
                int cell = y * n + x;
                double expected = HotK - (HotK - ColdK) * (x + 0.5) * h;
                maxTemperatureError = Math.Max(maxTemperatureError, Math.Abs(state[cell] - expected));
                stored += cellCapacity * (state[cell] - temperatures[cell]);
            }
        }

        double conductivePower = Conductivity * (HotK - ColdK);
        double hotNu = hotPower / conductivePower;
        double coldNu = coldPower / conductivePower;
        double residual = stored - netInput;
        Console.WriteLine(FormattableString.Invariant(
            $"V19 conduction baseline {n}x{n}: hot Nu {hotNu:G10}, cold Nu {coldNu:G10}, maximum temperature error {maxTemperatureError:G5} K, energy residual {residual:G5} J"));
        if (Math.Abs(hotNu - 1) > 1e-6 || Math.Abs(coldNu - 1) > 1e-6 ||
            maxTemperatureError > 1e-6 || Math.Abs(residual) > 1e-6)
            throw new InvalidOperationException("The cavity thermal baseline disagrees with pure conduction.");
    }
}
