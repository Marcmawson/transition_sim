using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Materials;
using PhysicsSim.ThreePhase3D.Thermal;
using Seb.Fluid.Simulation;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    internal sealed class MovingBeakerHeat : IDisposable
    {
        [StructLayout(LayoutKind.Sequential)]
        private struct Link
        {
            public int Other;
            public float Conductance;
        }

        private const int Rings = BeakerSolidMesh.InnerRadialCells;
        private const int Depth = BeakerSolidMesh.BaseDepthCells;
        private readonly BeakerSolidMesh mesh;
        private readonly FluidSim fluid;
        private readonly ComputeShader shader;
        private readonly List<ComputeBuffer> owned = new List<ComputeBuffer>();
        private readonly int clear, find, exchange, reduce, conduct, particleGroups, solidGroups;
        private readonly double maxStep, particleCapacity;
        private readonly float[] solidSnapshot, particleSnapshot;
        private ComputeBuffer glassRead, glassWrite;
        private double inputJ;

        public MovingBeakerHeat(FluidSim fluid, GeometryInputs geometry, double volumeM3)
        {
            this.fluid = fluid;
            var asset = Resources.Load<ComputeShader>("ThreePhase3D/MovingBeakerHeat");
            if (asset == null) throw new InvalidOperationException("Moving beaker heat shader is unavailable.");
            mesh = new BeakerSolidMesh(geometry);
            int count = fluid.positionBuffer.count;
            particleGroups = (count + 63) / 64;
            solidGroups = (mesh.CapacityJPerK.Length + 63) / 64;
            solidSnapshot = new float[mesh.CapacityJPerK.Length];
            particleSnapshot = new float[count];
            double particleVolume = volumeM3 / count;
            double spacing = Math.Pow(particleVolume, 1.0 / 3);
            var water = new Water95();
            var state = water.Evaluate(293.15, 101325, WaterBranch.Liquid);
            particleCapacity = particleVolume * state.DensityKgM3 * state.CpJkgK;
            double perArea = CoupledHeatFaces.HeatRateW(1, 2, 1, geometry.baseThicknessM / (2 * Depth),
                1.2, 0, spacing / 2, water.ConductivityWmK(state));
            double width = geometry.innerRadiusM / Rings;
            maxStep = Math.Min(.01, .5 * new SolidConduction(mesh.CapacityJPerK,
                Enumerable.Repeat(293.15, solidSnapshot.Length).ToArray(), mesh.Faces).MaximumStepSeconds);
            maxStep = Math.Min(maxStep, .5 * particleCapacity / (perArea * spacing * spacing));
            for (int i = 0; i < Rings; i++)
                maxStep = Math.Min(maxStep, .5 * mesh.CapacityJPerK[(i + 1) * Depth - 1] /
                    (perArea * Math.PI * (2 * i + 1) * width * width));

            shader = UnityEngine.Object.Instantiate(asset);
            try
            {
                clear = Kernel("ClearContacts");
                find = Kernel("FindContacts");
                exchange = Kernel("ExchangeHeat");
                reduce = Kernel("ReduceRingHeat");
                conduct = Kernel("ConductGlass");
                glassRead = Allocate(solidSnapshot.Length, 4);
                glassWrite = Allocate(solidSnapshot.Length, 4);
                var capacity = Allocate(solidSnapshot.Length, 4);
                var burner = Allocate(solidSnapshot.Length, 4);
                var counts = Allocate(Rings, 4);
                var ring = Allocate(count, 4);
                var particleGain = Allocate(count, 4);
                var ringGain = Allocate(Rings, 4);
                capacity.SetData(Array.ConvertAll(mesh.CapacityJPerK, value => (float)value));
                burner.SetData(Array.ConvertAll(mesh.BurnerWeights, value => (float)value));

                var adjacency = new List<Link>[solidSnapshot.Length];
                for (int i = 0; i < adjacency.Length; i++) adjacency[i] = new List<Link>();
                foreach (var face in mesh.Faces)
                {
                    adjacency[face.A].Add(new Link { Other = face.B, Conductance = (float)face.ConductanceWPerK });
                    adjacency[face.B].Add(new Link { Other = face.A, Conductance = (float)face.ConductanceWPerK });
                }
                var starts = new int[adjacency.Length + 1];
                for (int i = 0; i < adjacency.Length; i++) starts[i + 1] = starts[i] + adjacency[i].Count;
                var rows = Allocate(starts.Length, 4);
                var links = Allocate(starts[starts.Length - 1], 8);
                rows.SetData(starts);
                links.SetData(adjacency.SelectMany(list => list).ToArray());

                shader.SetInt("ParticleCount", count);
                shader.SetInt("RingCount", Rings);
                shader.SetInt("BaseDepth", Depth);
                shader.SetInt("SolidCount", solidSnapshot.Length);
                shader.SetFloat("RingWidth", (float)width);
                shader.SetFloat("ContactRadius", fluid.beakerCollision.z);
                shader.SetFloat("ContactFloor", fluid.beakerCollision.w);
                shader.SetFloat("ContactDepth", (float)spacing);
                shader.SetFloat("ParticleArea", (float)(spacing * spacing));
                shader.SetFloat("ParticleCapacity", (float)particleCapacity);
                shader.SetFloat("ConductancePerArea", (float)perArea);
                shader.SetBuffer(clear, "ContactCounts", counts);
                shader.SetBuffer(find, "Positions", fluid.positionBuffer);
                shader.SetBuffer(find, "ContactCounts", counts);
                shader.SetBuffer(find, "ContactRing", ring);
                shader.SetBuffer(exchange, "ContactCounts", counts);
                shader.SetBuffer(exchange, "ContactRing", ring);
                shader.SetBuffer(exchange, "TemperatureRise", fluid.TemperatureRise);
                shader.SetBuffer(exchange, "ParticleGain", particleGain);
                shader.SetBuffer(reduce, "ContactRing", ring);
                shader.SetBuffer(reduce, "ParticleGain", particleGain);
                shader.SetBuffer(reduce, "RingGain", ringGain);
                shader.SetBuffer(conduct, "RingGain", ringGain);
                shader.SetBuffer(conduct, "GlassCapacity", capacity);
                shader.SetBuffer(conduct, "BurnerWeight", burner);
                shader.SetBuffer(conduct, "FaceStart", rows);
                shader.SetBuffer(conduct, "FaceLinks", links);
                Reset();
            }
            catch { Dispose(); throw; }
        }

        public void Reset()
        {
            Array.Clear(solidSnapshot, 0, solidSnapshot.Length);
            glassRead.SetData(solidSnapshot);
            inputJ = 0;
        }

        public void Advance(double seconds, float watts)
        {
            int steps = Math.Max(1, (int)Math.Ceiling(seconds / maxStep));
            float dt = (float)(seconds / steps);
            shader.SetFloat("Dt", dt);
            shader.SetFloat("BurnerWatts", watts);
            shader.Dispatch(clear, 1, 1, 1);
            shader.Dispatch(find, particleGroups, 1, 1);
            for (int i = 0; i < steps; i++)
            {
                shader.SetBuffer(exchange, "GlassRead", glassRead);
                shader.Dispatch(exchange, particleGroups, 1, 1);
                shader.Dispatch(reduce, Rings, 1, 1);
                shader.SetBuffer(conduct, "GlassRead", glassRead);
                shader.SetBuffer(conduct, "GlassWrite", glassWrite);
                shader.Dispatch(conduct, solidGroups, 1, 1);
                (glassRead, glassWrite) = (glassWrite, glassRead);
            }
            inputJ += (double)dt * steps * watts;
        }

        public string Measure()
        {
            glassRead.GetData(solidSnapshot);
            fluid.TemperatureRise.GetData(particleSnapshot);
            double solidJ = solidSnapshot.Select((value, i) => value * mesh.CapacityJPerK[i]).Sum();
            double liquidJ = particleSnapshot.Sum(value => (double)value) * particleCapacity;
            double residual = solidJ + liquidJ - inputJ;
            return FormattableString.Invariant($"burner {inputJ:F4} J · solids {solidJ:F4} J · water {liquidJ:F4} J · residual {residual:G4} J");
        }

        private int Kernel(string name)
        {
            int kernel = shader.FindKernel(name);
            if (!shader.IsSupported(kernel)) throw new NotSupportedException(name + " is unsupported.");
            return kernel;
        }

        private ComputeBuffer Allocate(int count, int stride)
        {
            var buffer = new ComputeBuffer(count, stride);
            owned.Add(buffer);
            return buffer;
        }

        public void Dispose()
        {
            foreach (var buffer in owned) buffer.Release();
            owned.Clear();
            if (shader != null) UnityEngine.Object.Destroy(shader);
        }
    }
}
