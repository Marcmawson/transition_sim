using System;
using System.Linq;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.PhaseChange;
using UnityEditor;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Editor
{
    public static class PlanarStefanVerificationMenu
    {
        [MenuItem("Physics Sim/Three Phase 3D/Run S10 GPU planar Stefan check")]
        public static void Run()
        {
            var shader = Resources.Load<ComputeShader>("ThreePhase3D/PlanarStefan");
            if (!SystemInfo.supportsComputeShaders || shader == null)
                throw new NotSupportedException("S10 requires the planar Stefan compute shader.");
            int kernel = shader.FindKernel("AdvancePlanarFront");
            if (!shader.IsSupported(kernel)) throw new NotSupportedException("The S10 planar Stefan kernel is unsupported.");
            Check(shader, kernel, "melting", 4200, 0.6f, 10);
            Check(shader, kernel, "freezing", 2100, 2.2f, -10);
        }

        private static void Check(ComputeShader shader, int kernel, string name,
            float heatCapacity, float conductivity, float wallOffset)
        {
            const int cells = 32, steps = 4096, batch = 64;
            const float density = 1000, latentHeat = 333427, seedFront = 0.002f, dt = 0.00025f;
            var seed = new float[cells];
            for (int cell = 0; cell < cells; cell++)
                seed[cell] = wallOffset * (1 - (cell + 0.5f) / cells);
            var cpu = new PlanarStefanFront(density, heatCapacity, conductivity, latentHeat,
                wallOffset, seedFront, Array.ConvertAll(seed, value => (double)value));
            double initialEnergy = cpu.EnergyJPerM2;
            for (int step = 0; step < steps; step++)
                if (cpu.Advance(dt) != dt) throw new InvalidOperationException("S10 GPU fixture exceeds the CPU timestep bound.");

            using var temperatures = new ComputeBuffer(cells, sizeof(float));
            using var frontAndHeat = new ComputeBuffer(1, 4 * sizeof(float));
            temperatures.SetData(seed);
            frontAndHeat.SetData(new[] { new Vector4(seedFront, 0, 0, 0) });
            shader.SetBuffer(kernel, "TemperatureOffset", temperatures);
            shader.SetBuffer(kernel, "FrontAndWallHeat", frontAndHeat);
            shader.SetFloat("Density", density);
            shader.SetFloat("HeatCapacity", heatCapacity);
            shader.SetFloat("Conductivity", conductivity);
            shader.SetFloat("LatentHeat", latentHeat);
            shader.SetFloat("WallOffset", wallOffset);
            shader.SetFloat("Dt", dt);
            shader.SetInt("Steps", batch);
            for (int step = 0; step < steps; step += batch)
                shader.Dispatch(kernel, 1, 1, 1);

            var offsets = new float[cells];
            var state = new Vector4[1];
            temperatures.GetData(offsets);
            frontAndHeat.GetData(state);
            double front = state[0].x, wallHeat = state[0].y;
            double frontError = Math.Abs(front - cpu.FrontM);
            double profileError = offsets.Select((value, cell) => Math.Abs(value - cpu.TemperatureOffsetK[cell])).Max();
            double energy = density * front * (Math.Sign(wallOffset) * latentHeat + heatCapacity * offsets.Average(v => (double)v));
            double residual = energy - initialEnergy - wallHeat;
            if (!SiUnits.IsFinite(front) || !SiUnits.IsFinite(wallHeat) ||
                offsets.Any(value => !SiUnits.IsFinite(value)) || front <= seedFront ||
                Math.Sign(wallHeat) != Math.Sign(wallOffset) || frontError > 5e-8 ||
                profileError > 0.002 || Math.Abs(wallHeat - cpu.HeatInputJPerM2) > 0.003 * Math.Abs(cpu.HeatInputJPerM2) ||
                Math.Abs(residual) > 0.003 * Math.Abs(wallHeat))
                throw new InvalidOperationException($"S10 GPU {name} failed: front error {frontError:G6} m, temperature error {profileError:G6} K, energy residual {residual:G6} J/m2.");
            Debug.Log($"S10 GPU planar {name} passed: front {front:G8} m, CPU error {frontError:G6} m, temperature error {profileError:G6} K, wall heat {wallHeat:G8} J/m2, energy residual {residual:G6} J/m2.");
        }
    }
}
