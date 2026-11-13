using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Materials;
using PhysicsSim.ThreePhase3D.Thermal;
using Seb.Fluid.Simulation;
using UnityEngine;
using UnityEngine.Rendering;

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

        private struct FlowSample
        {
            public double WarmCount, WarmHeight, WarmSpeed, CoolCount, CoolSpeed, LiftedCount, LiftedSpeed, PeakRiseK, FloorContacts;

            public void Add(Vector4 totals, Vector4 counts, float floorContacts)
            {
                WarmCount += totals.x;
                WarmHeight += totals.y;
                WarmSpeed += totals.z;
                CoolSpeed += totals.w;
                CoolCount += counts.x;
                LiftedCount += counts.y;
                LiftedSpeed += counts.z;
                FloorContacts += floorContacts;
                PeakRiseK = Math.Max(PeakRiseK, counts.w);
            }

            public string Describe()
            {
                string warm = WarmCount > 0
                    ? FormattableString.Invariant($"warm {WarmCount:F0} · height {1000 * WarmHeight / WarmCount:F2} mm · vertical {1000 * WarmSpeed / WarmCount:+0.000;-0.000;0.000} mm/s")
                    : "no particles at ≥21 °C";
                string cool = CoolCount > 0
                    ? FormattableString.Invariant($"cool vertical {1000 * CoolSpeed / CoolCount:+0.000;-0.000;0.000} mm/s")
                    : "no cooler particles";
                string lifted = LiftedCount > 0
                    ? FormattableString.Invariant($"warm above 0.5 mm {LiftedCount:F0} · vertical {1000 * LiftedSpeed / LiftedCount:+0.000;-0.000;0.000} mm/s")
                    : "warm above 0.5 mm: 0";
                return FormattableString.Invariant($"Peak water rise {PeakRiseK:F4} K · Floor contacts {FloorContacts:F0}") + " · " + warm + " · " + cool + " · " + lifted;
            }
        }

        private const double InitialTemperatureK = 293.15;
        private const int Rings = BeakerSolidMesh.InnerRadialCells;
        private const int Depth = BeakerSolidMesh.BaseDepthCells;
        private readonly BeakerSolidMesh mesh;
        private readonly FluidSim fluid;
        private readonly ComputeShader shader;
        private readonly List<ComputeBuffer> owned = new List<ComputeBuffer>();
        private readonly int clear, find, exchange, reduce, conduct, measureFlow, particleGroups, solidGroups;
        private readonly ComputeBuffer flowTotals;
        private readonly Vector4[] flowSnapshot;
        private FlowSample sampledFlow;
        public string FlowDescription => sampledFlow.Describe();
        private readonly double maxStep, particleCapacity;
        private readonly float[] solidSnapshot, particleSnapshot;
        private ComputeBuffer glassRead, glassWrite, burnerEntropy;
        private float[] burnerEntropySnapshot;
        public double SampledStoredEntropyJPerK { get; private set; }
        public double SampledProducedEntropyJPerK { get; private set; }
        private double inputJ, nextSampleTime;
        private int sampleRevision;
        private bool samplePending, sampleDirty;
        public double SampledInputJ { get; private set; }
        public double SampledSolidJ { get; private set; }
        public double SampledWaterJ { get; private set; }

        public MovingBeakerHeat(FluidSim fluid, GeometryInputs geometry, double volumeM3)
        {
            this.fluid = fluid;
            var asset = Resources.Load<ComputeShader>("ThreePhase3D/MovingBeakerHeat");
            if (asset == null) throw new InvalidOperationException("Moving beaker heat shader is unavailable.");
            mesh = new BeakerSolidMesh(geometry);
            int count = fluid.positionBuffer.count;
            particleGroups = (count + 63) / 64;
            flowSnapshot = new Vector4[3 * particleGroups];
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
                measureFlow = Kernel("MeasureVerticalFlow");
                flowTotals = Allocate(flowSnapshot.Length, 16);
                shader.SetFloat("BeakerRim", fluid.beakerRimY);
                shader.SetBuffer(measureFlow, "Positions", fluid.positionBuffer);
                shader.SetBuffer(measureFlow, "Velocities", fluid.velocityBuffer);
                shader.SetBuffer(measureFlow, "TemperatureRise", fluid.TemperatureRise);
                shader.SetBuffer(measureFlow, "FlowTotals", flowTotals);
                glassRead = Allocate(solidSnapshot.Length, 4);
                glassWrite = Allocate(solidSnapshot.Length, 4);
                burnerEntropy = Allocate(solidSnapshot.Length, 4);
                burnerEntropySnapshot = new float[solidSnapshot.Length];
                shader.SetBuffer(conduct, "BurnerEntropyJPerK", burnerEntropy);
                shader.SetFloat("InitialTemperatureK", (float)InitialTemperatureK);
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
            Array.Clear(burnerEntropySnapshot, 0, burnerEntropySnapshot.Length);
            burnerEntropy.SetData(burnerEntropySnapshot);
            SampledStoredEntropyJPerK = SampledProducedEntropyJPerK = 0;
            inputJ = SampledInputJ = SampledSolidJ = SampledWaterJ = 0;
            sampleRevision++;
            samplePending = sampleDirty = false;
            sampledFlow = default;
            nextSampleTime = 0;
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
            sampleDirty = true;
        }

        public void SampleIfDue(double now)
        {
            if (!sampleDirty || samplePending || now < nextSampleTime) return;
            nextSampleTime = now + .5;
            if (!SystemInfo.supportsAsyncGPUReadback)
            {
                Measure();
                return;
            }

            sampleDirty = false;
            samplePending = true;
            int revision = sampleRevision;
            double sampledInput = inputJ, solidJ = 0, waterJ = 0;
            double solidEntropy = 0, waterEntropy = 0, inputEntropy = 0;
            bool failed = false;
            int remaining = 4;
            FlowSample flow = default;
            shader.Dispatch(measureFlow, particleGroups, 1, 1);

            void Complete()
            {
                if (--remaining != 0 || revision != sampleRevision) return;
                samplePending = false;
                if (failed) { sampleDirty = true; return; }
                PublishSample(sampledInput, solidJ, waterJ, solidEntropy + waterEntropy, inputEntropy);
                sampledFlow = flow;
            }

            AsyncGPUReadback.Request(glassRead, result =>
            {
                if (revision != sampleRevision) return;
                failed |= result.hasError;
                if (!result.hasError)
                {
                    var data = result.GetData<float>();
                    for (int i = 0; i < data.Length; i++)
                    {
                        solidJ += data[i] * mesh.CapacityJPerK[i];
                        solidEntropy += mesh.CapacityJPerK[i] * SpecificEntropyFactor(data[i]);
                    }
                }
                Complete();
            });
            AsyncGPUReadback.Request(fluid.TemperatureRise, result =>
            {
                if (revision != sampleRevision) return;
                failed |= result.hasError;
                if (!result.hasError)
                {
                    var data = result.GetData<float>();
                    for (int i = 0; i < data.Length; i++)
                    {
                        waterJ += data[i];
                        waterEntropy += SpecificEntropyFactor(data[i]);
                    }
                    waterJ *= particleCapacity;
                    waterEntropy *= particleCapacity;
                }
                Complete();
            });
            AsyncGPUReadback.Request(burnerEntropy, result =>
            {
                if (revision != sampleRevision) return;
                failed |= result.hasError;
                if (!result.hasError)
                {
                    var data = result.GetData<float>();
                    for (int i = 0; i < data.Length; i++) inputEntropy += data[i];
                }
                Complete();
            });
            AsyncGPUReadback.Request(flowTotals, result =>
            {
                if (revision != sampleRevision) return;
                failed |= result.hasError;
                if (!result.hasError)
                {
                    var data = result.GetData<Vector4>();
                    for (int i = 0; i < data.Length; i += 3) flow.Add(data[i], data[i + 1], data[i + 2].x);
                }
                Complete();
            });
        }

        private static double SpecificEntropyFactor(float rise) => Math.Log(1 + rise / InitialTemperatureK);

        private void PublishSample(double absorbed, double solid, double water, double storedEntropy, double inputEntropy)
        {
            SampledInputJ = absorbed;
            SampledSolidJ = solid;
            SampledWaterJ = water;
            SampledStoredEntropyJPerK = storedEntropy;
            SampledProducedEntropyJPerK = storedEntropy - inputEntropy;
        }

        public string Measure()
        {
            shader.Dispatch(measureFlow, particleGroups, 1, 1);
            flowTotals.GetData(flowSnapshot);
            sampledFlow = default;
            for (int i = 0; i < flowSnapshot.Length; i += 3)
                sampledFlow.Add(flowSnapshot[i], flowSnapshot[i + 1], flowSnapshot[i + 2].x);
            glassRead.GetData(solidSnapshot);
            fluid.TemperatureRise.GetData(particleSnapshot);
            double solidJ = solidSnapshot.Select((value, i) => value * mesh.CapacityJPerK[i]).Sum();
            double liquidJ = particleSnapshot.Sum(value => (double)value) * particleCapacity;
            sampleRevision++;
            samplePending = sampleDirty = false;
            burnerEntropy.GetData(burnerEntropySnapshot);
            double storedEntropy = solidSnapshot.Select((rise, i) => mesh.CapacityJPerK[i] * SpecificEntropyFactor(rise)).Sum() +
                particleSnapshot.Sum(SpecificEntropyFactor) * particleCapacity;
            PublishSample(inputJ, solidJ, liquidJ, storedEntropy, burnerEntropySnapshot.Sum(value => (double)value));
            double residual = solidJ + liquidJ - inputJ;
            return FormattableString.Invariant($"burner {inputJ:F4} J · solids {solidJ:F4} J · water {liquidJ:F4} J · residual {residual:G4} J · entropy change {SampledStoredEntropyJPerK:G4} J/K · inferred production {SampledProducedEntropyJPerK:G4} J/K · {FlowDescription}");
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
            sampleRevision++;
            foreach (var buffer in owned) buffer.Release();
            owned.Clear();
            if (shader != null) UnityEngine.Object.Destroy(shader);
        }
    }
}
