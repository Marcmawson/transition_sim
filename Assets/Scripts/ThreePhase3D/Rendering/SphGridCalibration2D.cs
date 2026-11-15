using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    public sealed class SphGridCalibration2D : MonoBehaviour
    {
        private const float SpacingMm = 5f;
        private const float SupportMm = 2 * SpacingMm;
        private const int Columns = 120;
        private const int Rows = 72;
        private const int Trials = 5;
        private static readonly float[] CellRatios = { .5f, .75f, 1f, 1.25f, 1.5f, 1.75f, 2f, 2.25f, 2.5f };
        private static readonly Color Background = new Color(.035f, .049f, .068f);
        private static readonly Color ParticleColor = new Color(.37f, .70f, .91f);

        private readonly Vector2[] particles = new Vector2[Columns * Rows];
        private readonly SearchResult[] results = new SearchResult[CellRatios.Length];
        private int selected = 4;
        private bool measuring, showParticles = true;
        private int completedWidths, bestIndex = -1;
        private Vector2 scroll;

        private float CellWidth => CellRatios[selected] * SupportMm;

        private void Awake()
        {
            // A repeatable, near-uniform sampling with a little jitter to exercise grid boundaries.
            var random = new System.Random(601);
            for (int y = 0; y < Rows; y++)
            for (int x = 0; x < Columns; x++)
            {
                float dx = ((float)random.NextDouble() - .5f) * .3f;
                float dy = ((float)random.NextDouble() - .5f) * .3f;
                particles[x + y * Columns] = new Vector2(x + .5f + dx, y + .5f + dy) * SpacingMm;
            }
        }

        private static Vector2Int CellOf(Vector2 point, float width) =>
            new Vector2Int(Mathf.FloorToInt(point.x / width), Mathf.FloorToInt(point.y / width));

        private SearchResult MeasureWidth(float width)
        {
            var cells = new Dictionary<Vector2Int, List<int>>();
            for (int i = 0; i < particles.Length; i++)
            {
                Vector2Int cell = CellOf(particles[i], width);
                if (!cells.TryGetValue(cell, out List<int> occupants))
                {
                    occupants = new List<int>();
                    cells.Add(cell, occupants);
                }
                occupants.Add(i);
            }

            long candidates = 0, neighbours = 0;
            Vector2 reach = Vector2.one * SupportMm;
            for (int i = 0; i < particles.Length; i++)
            {
                Vector2 point = particles[i];
                Vector2Int first = CellOf(point - reach, width);
                Vector2Int last = CellOf(point + reach, width);
                for (int y = first.y; y <= last.y; y++)
                for (int x = first.x; x <= last.x; x++)
                {
                    if (!cells.TryGetValue(new Vector2Int(x, y), out List<int> occupants)) continue;
                    foreach (int j in occupants)
                    {
                        if (i == j) continue;
                        candidates++;
                        if ((particles[j] - point).sqrMagnitude <= SupportMm * SupportMm) neighbours++;
                    }
                }
            }

            return new SearchResult { Candidates = candidates, Neighbours = neighbours, OccupiedCells = cells.Count };
        }

        private IEnumerator MeasureAllWidths()
        {
            measuring = true;
            bestIndex = -1;
            completedWidths = 0;
            System.Array.Clear(results, 0, results.Length);
            yield return null;

            long expectedNeighbours = -1;
            bool sameNeighbours = true;
            for (int i = 0; i < CellRatios.Length; i++)
            {
                float width = CellRatios[i] * SupportMm;
                MeasureWidth(width);
                var times = new double[Trials];
                SearchResult result = default;
                for (int trial = 0; trial < Trials; trial++)
                {
                    long start = Stopwatch.GetTimestamp();
                    result = MeasureWidth(width);
                    times[trial] = (Stopwatch.GetTimestamp() - start) * 1000d / Stopwatch.Frequency;
                    yield return null;
                }

                System.Array.Sort(times);
                result.Milliseconds = times[Trials / 2];
                results[i] = result;
                if (expectedNeighbours < 0) expectedNeighbours = result.Neighbours;
                sameNeighbours &= expectedNeighbours == result.Neighbours;
                if (bestIndex < 0 || result.Milliseconds < results[bestIndex].Milliseconds) bestIndex = i;
                completedWidths++;
            }

            if (!sameNeighbours)
            {
                bestIndex = -1;
                UnityEngine.Debug.LogError("2D SPH grid search returned different neighbour counts across widths.", this);
            }
            else
            {
                UnityEngine.Debug.Log($"2D SPH grid calibration · {particles.Length:N0} particles · best {CellRatios[bestIndex]:0.##}× support · " +
                    $"{results[bestIndex].Milliseconds:F2} ms median build + search · {expectedNeighbours:N0} directed neighbours at every width", this);
            }
            measuring = false;
        }

        private void OnGUI()
        {
            float panelWidth = Mathf.Min(375, Screen.width * .46f);
            if (Event.current.type == EventType.Repaint)
            {
                DrawRect(new Rect(0, 0, Screen.width, Screen.height), Background);
                DrawGrid(new Rect(panelWidth + 24, 24, Screen.width - panelWidth - 48, Screen.height - 48));
            }

            GUILayout.BeginArea(new Rect(12, 12, panelWidth, Mathf.Max(0, Screen.height - 24)), GUI.skin.box);
            scroll = GUILayout.BeginScrollView(scroll);
            GUILayout.Label("2D SPH Grid Calibration");
            GUILayout.Label($"Particle spacing {SpacingMm:0} mm · support radius {SupportMm:0} mm");
            GUILayout.Label($"Grid {CellWidth:0.##} mm · {CellRatios[selected]:0.##}× support");
            float expectedOccupancy = Mathf.Pow(CellWidth / SpacingMm, 2);
            GUILayout.Label($"Estimated interior occupancy: {expectedOccupancy:0.##} particles");
            GUILayout.Label("Paper ratio: 1.5× support = 15 mm cells");
            GUILayout.Label("2D estimate: cell width = spacing × √target occupancy");
            showParticles = GUILayout.Toggle(showParticles, "Show particles");
            bool previousEnabled = GUI.enabled;
            GUI.enabled = previousEnabled && !measuring;
            if (GUILayout.Button("Measure all widths")) StartCoroutine(MeasureAllWidths());
            GUI.enabled = previousEnabled;
            GUILayout.Label(measuring ? $"Measured {completedWidths}/{CellRatios.Length} widths" :
                "CPU build + neighbour search · median of five runs");
            for (int i = 0; i < CellRatios.Length; i++)
            {
                SearchResult result = results[i];
                string measurements = i < completedWidths
                    ? $" · {result.Milliseconds:F2} ms · {result.Candidates:N0} checks · {(double)particles.Length / result.OccupiedCells:F1}/cell"
                    : "";
                string label = $"{CellRatios[i] * SupportMm:0.##} mm{(i == bestIndex && !measuring ? " (fastest)" : "")}{measurements}";
                if (GUILayout.Toggle(selected == i, label, GUI.skin.button)) selected = i;
            }
            GUILayout.EndScrollView();
            GUILayout.EndArea();
        }

        private void DrawGrid(Rect area)
        {
            if (area.width <= 0 || area.height <= 0) return;
            float domainWidth = Columns * SpacingMm;
            float domainHeight = Rows * SpacingMm;
            // Reserve space for full edge cells at every width without changing the drawing scale.
            float padding = CellRatios[CellRatios.Length - 1] * SupportMm;
            float pixelsPerMm = Mathf.Min(area.width / (domainWidth + padding),
                area.height / (domainHeight + padding));
            Vector2 origin = new Vector2(area.x, area.y + area.height);
            float width = CellWidth;
            int cellsX = Mathf.CeilToInt(domainWidth / width);
            int cellsY = Mathf.CeilToInt(domainHeight / width);
            float cellPixels = width * pixelsPerMm;

            for (int x = 0; x <= cellsX; x++)
                DrawRect(new Rect(origin.x + x * cellPixels - .5f, origin.y - cellsY * cellPixels,
                    1, cellsY * cellPixels), Color.white);
            for (int y = 0; y <= cellsY; y++)
                DrawRect(new Rect(origin.x, origin.y - y * cellPixels - .5f, cellsX * cellPixels, 1), Color.white);

            if (!showParticles) return;
            foreach (Vector2 particle in particles)
            {
                Vector2 screen = origin + new Vector2(particle.x, -particle.y) * pixelsPerMm;
                DrawRect(new Rect(screen.x - 1, screen.y - 1, 2, 2), ParticleColor);
            }
        }

        private static void DrawRect(Rect rect, Color color)
        {
            Color previous = GUI.color;
            GUI.color = color;
            GUI.DrawTexture(rect, Texture2D.whiteTexture);
            GUI.color = previous;
        }

        private struct SearchResult
        {
            public long Candidates;
            public long Neighbours;
            public int OccupiedCells;
            public double Milliseconds;
        }
    }
}
