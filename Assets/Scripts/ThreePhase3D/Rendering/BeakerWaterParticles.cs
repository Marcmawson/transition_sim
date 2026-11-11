using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    internal sealed class BeakerWaterParticles : IDisposable
    {
        private readonly BeakerVofTimeIntegration release;
        private readonly GeometryInputs glass;
        private readonly ParticleSystem display;
        private readonly Material material;
        private readonly Texture2D dot;
        private ParticleSystem.Particle[] particles = new ParticleSystem.Particle[2048];

        public BeakerWaterParticles(Transform parent, BeakerVofTimeIntegration release, GeometryInputs glass)
        {
            this.release = release;
            this.glass = glass;
            var shader = Shader.Find("Sprites/Default");
            if (shader == null) throw new InvalidOperationException("The water particle shader is unavailable.");

            dot = CreateDot();
            material = new Material(shader) { mainTexture = dot };
            var container = new GameObject("Water particles");
            container.transform.SetParent(parent, false);
            display = container.AddComponent<ParticleSystem>();
            display.Stop(false, ParticleSystemStopBehavior.StopEmittingAndClear);
            var main = display.main;
            main.playOnAwake = false;
            main.simulationSpace = ParticleSystemSimulationSpace.Local;
            main.maxParticles = particles.Length;
            var emission = display.emission;
            emission.enabled = false;
            var renderer = container.GetComponent<ParticleSystemRenderer>();
            renderer.sharedMaterial = material;
            renderer.minParticleSize = 0;
            display.Play(false);
            display.Pause(false);
        }

        private int lastSamplesPerAxis;
        private float lastDiameterM, lastJitter;

        public bool SettingsChanged(int samplesPerAxis, float diameterM, float jitter) =>
            samplesPerAxis != lastSamplesPerAxis || diameterM != lastDiameterM || jitter != lastJitter;

        public void Refresh(int samplesPerAxis, float diameterM, float jitter)
        {
            var grid = release.Geometry.Grid;
            var cells = grid.Cells;
            int samplesPerCell = samplesPerAxis * samplesPerAxis * samplesPerAxis;
            int count = 0;
            for (int k = 0; k < cells.Z; k++)
            for (int j = 0; j < cells.Y; j++)
            for (int i = 0; i < cells.X; i++)
            {
                double fraction = release.LiquidFractionAt(i, j, k);
                if (fraction <= 0) continue;
                int cell = cells.Index(i, j, k);
                var lower = grid.Frame.ToWorld(grid.Lower(i, j, k));
                for (int sample = 0; sample < samplesPerCell; sample++)
                {
                    uint seed = (uint)(cell * samplesPerCell + sample);
                    if (Unit(seed * 4) >= fraction) continue;
                    var point = new Vector3(
                        (float)(lower.X + grid.Spacing.X * Offset(sample % samplesPerAxis, seed * 4 + 1, samplesPerAxis, jitter)),
                        (float)(lower.Y + grid.Spacing.Y * Offset(sample / samplesPerAxis % samplesPerAxis, seed * 4 + 2, samplesPerAxis, jitter)),
                        (float)(lower.Z + grid.Spacing.Z * Offset(sample / (samplesPerAxis * samplesPerAxis), seed * 4 + 3, samplesPerAxis, jitter)));
                    if (TouchesGlass(point, diameterM * .5f)) continue;
                    if (count == particles.Length) Array.Resize(ref particles, count * 2);
                    particles[count++] = new ParticleSystem.Particle
                    {
                        position = point,
                        startSize = diameterM,
                        startColor = new Color32(62, 207, 248, 255),
                        startLifetime = 1,
                        remainingLifetime = 1
                    };
                }
            }
            var main = display.main;
            if (main.maxParticles < count) main.maxParticles = particles.Length;
            display.SetParticles(particles, count);
            lastSamplesPerAxis = samplesPerAxis;
            lastDiameterM = diameterM;
            lastJitter = jitter;
        }

        private bool TouchesGlass(Vector3 point, float padding)
        {
            float bottom = (float)glass.externalBaseHeightM;
            float rim = (float)glass.RimHeightM;
            if (point.y < bottom - padding || point.y > rim + padding) return false;
            float sector = 2 * Mathf.PI / BeakerVofTimeIntegration.Segments;
            float angle = Mathf.Atan2(point.z, point.x);
            float normal = (Mathf.Floor(angle / sector) + .5f) * sector;
            float side = point.x * Mathf.Cos(normal) + point.z * Mathf.Sin(normal);
            float scale = Mathf.Cos(sector / 2);
            if (side > (float)glass.OuterRadiusM * scale + padding) return false;
            float floor = bottom + (float)glass.baseThicknessM;
            return point.y < floor + padding || side > (float)glass.innerRadiusM * scale - padding;
        }

        private static float Offset(int index, uint seed, int samplesPerAxis, float jitter) =>
            (index + .5f + jitter * (Unit(seed) - .5f)) / samplesPerAxis;

        private static float Unit(uint value)
        {
            unchecked
            {
                value ^= value >> 16;
                value *= 0x7feb352du;
                value ^= value >> 15;
                value *= 0x846ca68bu;
                value ^= value >> 16;
                return (value >> 8) * (1f / 16777216f);
            }
        }

        internal static Texture2D CreateDot()
        {
            const int size = 16;
            var texture = new Texture2D(size, size, TextureFormat.RGBA32, false) { wrapMode = TextureWrapMode.Clamp };
            var pixels = new Color32[size * size];
            for (int y = 0; y < size; y++)
            for (int x = 0; x < size; x++)
            {
                var point = new Vector2((x + .5f) / size - .5f, (y + .5f) / size - .5f);
                byte alpha = (byte)Mathf.RoundToInt(255 * Mathf.Clamp01((.5f - point.magnitude) * size));
                pixels[x + y * size] = new Color32(255, 255, 255, alpha);
            }
            texture.SetPixels32(pixels);
            texture.Apply(false, true);
            return texture;
        }

        public void Dispose()
        {
            display.gameObject.SetActive(false);
            UnityEngine.Object.Destroy(display.gameObject);
            UnityEngine.Object.Destroy(material);
            UnityEngine.Object.Destroy(dot);
        }
    }
}
