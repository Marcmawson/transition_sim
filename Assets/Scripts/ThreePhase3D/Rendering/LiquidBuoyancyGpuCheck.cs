using System;
using System.Linq;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.Materials;
using PhysicsSim.ThreePhase3D.Thermal;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    public static class LiquidBuoyancyGpuCheck
    {
        public static void Run()
        {
            const double transportDensity = 1000;
            const float dt = .05f, gravity = 9.80665f;
            if (!SystemInfo.supportsComputeShaders)
                throw new NotSupportedException("The S09 buoyancy check requires compute shader support.");
            var shader = Resources.Load<ComputeShader>("ThreePhase3D/LiquidBuoyancy");
            var pressure = Resources.Load<ComputeShader>("ThreePhase3D/SinglePhaseFlow");
            if (shader == null || pressure == null) throw new InvalidOperationException("S09 buoyancy or pressure shader is missing.");

            var grid = new Grid3D(4, 4, 2, new D3(.01, .01, .01));
            var plan = new VariableCoefficientProjection(CutCellGeometry.Build(grid),
                BoundaryConditions3D.Closed, _ => transportDensity, _ => 0);
            var table = new WaterPropertyTable(WaterPropertyTable.Generate(WaterBranch.Liquid));
            var enthalpy = new FluidEnthalpy(table, transportDensity);
            var packed = table.CopyValues();
            var density = Enumerable.Range(0, table.Count).Select(i => packed[i * WaterPropertyTable.Width]).ToArray();
            var vertical = new Vector2Int[plan.FaceCount];
            for (int f = 0; f < vertical.Length; f++)
            {
                var face = plan.Faces[f];
                vertical[f] = face.Active != 0 && face.Axis == 1 && face.Low >= 0 && face.High >= 0
                    ? new Vector2Int(face.Low, face.High) : new Vector2Int(-1, -1);
            }

            using (var transport = new CutCellGpuScalars(plan, Resources.Load<ComputeShader>("ThreePhase3D/CutCellVofScalars")))
            using (var recovery = new LiquidEnthalpyGpuRecovery(
                Resources.Load<ComputeShader>("ThreePhase3D/LiquidEnthalpyRecovery"), table, plan.GroupCount, transportDensity))
            using (var densities = new ComputeBuffer(density.Length, sizeof(float)))
            using (var neighbours = new ComputeBuffer(vertical.Length, 2 * sizeof(int)))
            using (var initialVelocity = new ComputeBuffer(vertical.Length, sizeof(float)))
            using (var tentative = new ComputeBuffer(vertical.Length, sizeof(float)))
            {
                int kernel = shader.FindKernel("PredictBuoyancy");
                if (!shader.IsSupported(kernel)) throw new NotSupportedException("S09 buoyancy kernel is unsupported.");
                densities.SetData(density);
                neighbours.SetData(vertical);
                initialVelocity.SetData(new float[vertical.Length]);
                shader.SetInt("FaceCount", plan.FaceCount);
                shader.SetInt("TableCount", table.Count);
                shader.SetFloat("MinimumK", (float)table.MinimumK);
                shader.SetFloat("StepK", (float)table.StepK);
                shader.SetFloat("GravityMagnitude", gravity);
                shader.SetFloat("Dt", dt);
                shader.SetBuffer(kernel, "DensityTable", densities);
                shader.SetBuffer(kernel, "VerticalNeighbours", neighbours);
                shader.SetBuffer(kernel, "VelocityRead", initialVelocity);
                shader.SetBuffer(kernel, "TentativeVelocity", tentative);
                shader.SetBuffer(kernel, "LiquidState", recovery.State);
                shader.SetBuffer(kernel, "RecoveryStatus", recovery.Status);
                var velocity = new float[vertical.Length];
                var zeroFlow = new double[vertical.Length];
                var zero = new double[plan.GroupCount];
                var status = new uint[plan.GroupCount];

                var water = new Water95();
                var pressureCorrection = transport.MeasurePressureCorrection(dt);
                double worstVelocityError = 0, worstEnthalpyDrift = 0;

                float Check(float ambientK, float columnK, int direction)
                {
                    var energy = plan.Volumes.Select(v => enthalpy.EnergyJ(v, ambientK)).ToArray();
                    var temperatures = Enumerable.Repeat((double)ambientK, plan.GroupCount).ToArray();
                    for (int k = 0; k < grid.Cells.Z; k++)
                        for (int j = 0; j < grid.Cells.Y; j++)
                        {
                            int group = plan.PressureGroup(1, j, k);
                            energy[group] = enthalpy.EnergyJ(plan.Volumes[group], columnK);
                            temperatures[group] = columnK;
                        }

                    var inventory = new CutCellInventories(plan.Volumes, zero, energy, zero, zero);
                    transport.AdvanceWithTemperature(inventory, zeroFlow, dt, recovery);
                    recovery.Status.GetData(status);
                    if (status.Any(value => value != 0)) throw new InvalidOperationException("S09 cavity contains invalid liquid state.");
                    float rho = table.EvaluateFloat(ambientK, 101325)[0];
                    shader.SetFloat("ReferenceDensity", rho);
                    shader.Dispatch(kernel, (plan.FaceCount + 63) / 64, 1, 1);
                    tentative.GetData(velocity);
                    var projected = FlowGpuSolver.Project(plan, pressure, velocity, dt, 4096, pressureCorrection);
                    int sample = plan.FaceIndex(1, 1, 2, 0);
                    float actual = projected.VelocityMPerS[sample];

                    // Evaluate the EOS directly in double precision, bypassing the GPU density table.
                    double referenceDensity = WaterPropertyTable.EvaluateReference(water, ambientK, table.PressurePa, WaterBranch.Liquid)[0];
                    double columnDensity = WaterPropertyTable.EvaluateReference(water, columnK, table.PressurePa, WaterBranch.Liquid)[0];
                    var referenceTentative = new float[vertical.Length];
                    for (int face = 0; face < vertical.Length; face++)
                    {
                        var pair = vertical[face];
                        if (pair.x < 0) continue;
                        double low = temperatures[pair.x] == columnK ? columnDensity : referenceDensity;
                        double high = temperatures[pair.y] == columnK ? columnDensity : referenceDensity;
                        referenceTentative[face] = (float)(dt * gravity * (referenceDensity - 0.5 * (low + high)) / referenceDensity);
                    }
                    var reference = FlowCpuReference.Project(plan, referenceTentative, dt, 4096);
                    double velocityError = Math.Abs(actual - reference.VelocityMPerS[sample]);
                    var moved = transport.AdvanceProjectedWithTemperature(inventory, projected, recovery);
                    double enthalpyDrift = Math.Abs(moved.Fields.LiquidEnergyJ.Sum() - energy.Sum());
                    recovery.Status.GetData(status);
                    if (velocityError > 2e-7 || enthalpyDrift > 1e-8 || status.Any(value => value != 0))
                        throw new InvalidOperationException($"V20 reference or energy mismatch at {ambientK:G6} to {columnK:G6} K: velocity error {velocityError:G6} m/s, enthalpy drift {enthalpyDrift:G6} J.");
                    worstVelocityError = Math.Max(worstVelocityError, velocityError);
                    worstEnthalpyDrift = Math.Max(worstEnthalpyDrift, enthalpyDrift);
                    if (!projected.Diagnostics.Converged || float.IsNaN(actual) ||
                        (direction == 0 ? Math.Abs(actual) > 1e-8 : direction * actual <= 1e-7))
                        throw new InvalidOperationException($"S09 unexpected projected velocity {actual:G6} m/s.");
                    return actual;
                }

                float uniform = Check(277.125f, 277.125f, 0);
                float sinking = Check(274f, 277.125f, -1);
                float rising = Check(277.125f, 283f, 1);
                Debug.Log($"S09 buoyancy check passed: uniform {uniform:G5} m/s, warmed below 4 °C {sinking:G5} m/s, warmed above 4 °C {rising:G5} m/s.");
                Debug.Log($"S09 V20 GPU density anomaly passed: maximum reference velocity error {worstVelocityError:G6} m/s, maximum enthalpy drift {worstEnthalpyDrift:G6} J.");
                CheckCirculation(false);
                CheckCirculation(true);

                void CheckCirculation(bool heatedFloor)
                {
                    const int steps = 40;
                    float ambientK = heatedFloor ? 293.15f : 277.125f;
                    const float warmK = 283f;
                    var energy = plan.Volumes.Select(v => enthalpy.EnergyJ(v, ambientK)).ToArray();
                    if (!heatedFloor)
                        for (int k = 0; k < grid.Cells.Z; k++)
                            for (int j = 0; j < grid.Cells.Y; j++)
                            {
                                int group = plan.PressureGroup(1, j, k);
                                energy[group] = enthalpy.EnergyJ(plan.Volumes[group], warmK);
                            }

                    double initialEnergy = energy.Sum(), initialVolume = plan.Volumes.Sum();
                    var inventory = new CutCellInventories(plan.Volumes, zero, energy, zero, zero);
                    var correction = transport.MeasurePressureCorrection(dt);
                    var flow = FlowGpuSolver.Project(plan, pressure, new float[plan.FaceCount], dt, 4096, correction);
                    int sampleY = heatedFloor ? 1 : 2;
                    int risingFace = plan.FaceIndex(1, 1, sampleY, 0);
                    int returnFace = plan.FaceIndex(1, 2, sampleY, 0);
                    double worstEnergy = 0, worstVolume = 0;
                    float minimumRise = float.PositiveInfinity, maximumReturn = float.NegativeInfinity;
                    shader.SetFloat("ReferenceDensity", table.EvaluateFloat(ambientK, 101325)[0]);

                    int width = grid.Cells.X, depth = grid.Cells.Z;
                    var floorGroups = new int[width * depth];
                    var gains = new float[floorGroups.Length];
                    var glassTemperatures = new Vector2[floorGroups.Length];
                    const float tileWatts = 2.5f, glassCapacity = .27f;
                    var floorShader = heatedFloor ? Resources.Load<ComputeShader>("ThreePhase3D/HeatedLiquidFloor") : null;
                    if (heatedFloor && floorShader == null)
                        throw new InvalidOperationException("The S09 heated-floor shader is missing.");

                    using var glassA = heatedFloor ? new ComputeBuffer(floorGroups.Length, 8) : null;
                    using var glassB = heatedFloor ? new ComputeBuffer(floorGroups.Length, 8) : null;
                    using var floorIndex = heatedFloor ? new ComputeBuffer(floorGroups.Length, 4) : null;
                    using var heatGain = heatedFloor ? new ComputeBuffer(floorGroups.Length, 4) : null;
                    ComputeBuffer glassRead = glassA, glassWrite = glassB;
                    int heatKernel = -1;
                    if (heatedFloor)
                    {
                        for (int z = 0; z < depth; z++)
                            for (int x = 0; x < width; x++)
                                floorGroups[z * width + x] = plan.PressureGroup(x, 0, z);
                        floorIndex.SetData(floorGroups);
                        glassRead.SetData(glassTemperatures);
                        heatKernel = floorShader.FindKernel("HeatFloor");
                        if (!floorShader.IsSupported(heatKernel)) throw new NotSupportedException("S09 heated-floor kernel is unsupported.");
                        floorShader.SetInt("Width", width);
                        floorShader.SetInt("Depth", depth);
                        floorShader.SetInt("HeatedColumn", 1);
                        floorShader.SetFloat("Dt", dt);
                        floorShader.SetFloat("InitialK", ambientK);
                        floorShader.SetFloat("TilePowerW", tileWatts);
                        // Synthetic 10 × 10 mm glass tiles, two 1.5 mm layers.
                        floorShader.SetFloat("CellCapacity", glassCapacity);
                        floorShader.SetFloat("AxialConductance", .08f);
                        floorShader.SetFloat("LateralConductance", .0018f);
                        double waterK = table.EvaluateFloat(ambientK, 101325)[10];
                        float contact = (float)CoupledHeatFaces.HeatRateW(.0001, 2, 1, .00075, 1.2, 0, .005, waterK);
                        floorShader.SetFloat("ContactConductance", contact);
                        floorShader.SetBuffer(heatKernel, "LiquidGroups", floorIndex);
                        floorShader.SetBuffer(heatKernel, "LiquidState", recovery.State);
                        floorShader.SetBuffer(heatKernel, "LiquidGainJ", heatGain);
                        transport.AdvanceWithTemperature(inventory, zeroFlow, dt, recovery);
                    }

                    using (var momentum = new CutCellGpuMomentum(plan,
                        Resources.Load<ComputeShader>("ThreePhase3D/CutCellVofMomentum")))
                        for (int i = 0; i < steps; i++)
                        {
                            if (heatedFloor)
                            {
                                floorShader.SetBuffer(heatKernel, "GlassRead", glassRead);
                                floorShader.SetBuffer(heatKernel, "GlassWrite", glassWrite);
                                floorShader.Dispatch(heatKernel, (floorGroups.Length + 63) / 64, 1, 1);
                                (glassRead, glassWrite) = (glassWrite, glassRead);
                                heatGain.GetData(gains);
                                var nextEnergy = (double[])inventory.LiquidEnergyJ.Clone();
                                for (int j = 0; j < gains.Length; j++) nextEnergy[floorGroups[j]] += gains[j];
                                inventory = new CutCellInventories(inventory.LiquidM3, inventory.GasM3, nextEnergy,
                                    inventory.GasEnergyJ, inventory.VapourKg);
                            }

                            var moved = transport.AdvanceProjectedWithTemperature(inventory, flow, recovery);
                            var advected = momentum.Advance(inventory, moved,
                                Array.ConvertAll(flow.VelocityMPerS, value => (double)value), transportDensity, 1.2);
                            inventory = moved.Fields;
                            recovery.Status.GetData(status);
                            if (status.Any(value => value != 0))
                                throw new InvalidOperationException($"S09 invalid recovered liquid at step {i + 1}.");

                            initialVelocity.SetData(Array.ConvertAll(advected.VelocityMPerS, value => (float)value));
                            shader.Dispatch(kernel, (plan.FaceCount + 63) / 64, 1, 1);
                            tentative.GetData(velocity);
                            flow = FlowGpuSolver.Project(plan, pressure, velocity, dt, 4096, correction, flow.GroupPressurePa);
                            float up = flow.VelocityMPerS[risingFace], down = flow.VelocityMPerS[returnFace];
                            if (!flow.Diagnostics.Converged || !SiUnits.IsFinite(up) || !SiUnits.IsFinite(down))
                                throw new InvalidOperationException($"S09 invalid projected flow at step {i + 1}.");
                            minimumRise = Math.Min(minimumRise, up);
                            maximumReturn = Math.Max(maximumReturn, down);
                            worstEnergy = Math.Max(worstEnergy, Math.Abs(inventory.LiquidEnergyJ.Sum() - initialEnergy));
                            worstVolume = Math.Max(worstVolume, Math.Abs(inventory.LiquidM3.Sum() - initialVolume));
                        }

                    if (heatedFloor)
                    {
                        glassRead.GetData(glassTemperatures);
                        double glassJ = glassTemperatures.Sum(value => glassCapacity * (double)(value.x + value.y));
                        double waterJ = inventory.LiquidEnergyJ.Sum() - initialEnergy;
                        double inputJ = steps * (double)dt * depth * tileWatts;
                        double residual = glassJ + waterJ - inputJ;
                        float up = flow.VelocityMPerS[risingFace], down = flow.VelocityMPerS[returnFace];
                        if (!(waterJ > 0 && glassJ > 0 && up > 1e-7 && down < -1e-7) ||
                            !SiUnits.IsFinite(residual) || Math.Abs(residual) > 1e-4 || worstVolume > 1e-12)
                            throw new InvalidOperationException($"S09 heated floor failed: glass {glassJ:G5} J, water {waterJ:G5} J, residual {residual:G5} J, up {up:G5}, return {down:G5} m/s.");
                        Debug.Log($"S09 heated floor check passed: {inputJ:F3} J supplied, glass {glassJ:F4} J, water {waterJ:F4} J, residual {residual:G5} J, rising {up:G5} m/s, return {down:G5} m/s.");
                        return;
                    }

                    double redistribution = inventory.LiquidEnergyJ.Zip(energy, (after, before) => Math.Abs(after - before)).Sum();
                    if (minimumRise <= 1e-7 || maximumReturn >= -1e-7 ||
                        worstEnergy > 1e-7 || worstVolume > 1e-12 || redistribution < 1e-5)
                        throw new InvalidOperationException($"S09 circulation failed: up {minimumRise:G5}, return {maximumReturn:G5}, energy {worstEnergy:G5}, volume {worstVolume:G5}, redistribution {redistribution:G5}.");
                    Debug.Log($"S09 circulation check passed: {steps} steps, {steps * dt:F2} s, rising {flow.VelocityMPerS[risingFace]:G5} m/s, return {flow.VelocityMPerS[returnFace]:G5} m/s, enthalpy drift {worstEnergy:G5} J, volume drift {worstVolume:G5} m³.");
                }
            }
        }
    }
}
