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

        private static readonly Color CoarseParticleColor = new Color(1f, .61f, .25f);
        private SphDamBreak2D damBreak;
        private bool damBreakView = true, damRunning;
        private float damPendingSeconds;

        private readonly Vector2[] particles = new Vector2[Columns * Rows];
        private readonly SearchResult[] results = new SearchResult[CellRatios.Length];
        private int selected = 4;
        private bool measuring, showParticles = true;
        private int completedWidths, bestIndex = -1;
        private Vector2 scroll;

        private float CellWidth => CellRatios[selected] * SupportMm;

        private void Awake()
        {
            damBreak = new SphDamBreak2D();
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

        private void Update()
        {
            if (!damBreakView || !damRunning) return;
            damPendingSeconds = Mathf.Min(damPendingSeconds + Time.unscaledDeltaTime, .012f);
            int steps = Mathf.Min(24, Mathf.FloorToInt(damPendingSeconds / SphDamBreak2D.StepSeconds));
            if (steps == 0) return;
            damBreak.Advance(steps);
            damPendingSeconds -= steps * SphDamBreak2D.StepSeconds;
        }

        private void OnGUI()
        {
            float panelWidth = Mathf.Min(375, Screen.width * .46f);
            if (Event.current.type == EventType.Repaint)
            {
                DrawRect(new Rect(0, 0, Screen.width, Screen.height), Background);
                var area = new Rect(panelWidth + 24, 24, Screen.width - panelWidth - 48, Screen.height - 48);
                if (damBreakView) DrawDamBreak(area);
                else DrawGrid(area);
            }

            GUILayout.BeginArea(new Rect(12, 12, panelWidth, Mathf.Max(0, Screen.height - 24)), GUI.skin.box);
            scroll = GUILayout.BeginScrollView(scroll);
            damBreakView = GUILayout.Toolbar(damBreakView ? 0 : 1, new[] { "Dam break", "Grid calibration" }) == 0;
            if (damBreakView)
            {
                DrawDamBreakControls();
                GUILayout.EndScrollView();
                GUILayout.EndArea();
                return;
            }
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

        private void DrawDamBreakControls()
        {
            GUILayout.Label("2D SPH Dam Break");
            GUILayout.Label($"Simulated time {damBreak.StepCount * SphDamBreak2D.StepSeconds:F3} s");
            GUILayout.Label($"{damBreak.Count:N0} particles · timestep {SphDamBreak2D.StepSeconds * 1000:0.###} ms");
            DrawGridLegend(ParticleColor, $"Fine · {damBreak.FineCount:N0} particles · 15 mm cells");
            DrawGridLegend(CoarseParticleColor, $"Coarse · {damBreak.Count - damBreak.FineCount:N0} particles · 30 mm cells");
            GUILayout.Label("White borders show occupied search cells.");
            GUILayout.Label("Slow surface water splits below 0.40 m/s. Nearby siblings merge in the bulk or above 0.65 m/s.");
            showParticles = GUILayout.Toggle(showParticles, "Show particles");
            if (GUILayout.Button(damRunning ? "Pause" : damBreak.StepCount == 0 ? "Release water" : "Resume"))
            {
                damRunning = !damRunning;
                damPendingSeconds = 0;
            }
            if (GUILayout.Button("Reset dam break"))
            {
                damRunning = false;
                damPendingSeconds = 0;
                damBreak.Reset();
            }
        }

        private static void DrawGridLegend(Color color, string label)
        {
            Color previous = GUI.contentColor;
            GUI.contentColor = color;
            GUILayout.Label(label);
            GUI.contentColor = previous;
        }

        private void DrawDamBreak(Rect area)
        {
            if (area.width <= 0 || area.height <= 0) return;
            float scale = Mathf.Min(area.width / SphDamBreak2D.Width, area.height / SphDamBreak2D.Height);
            float width = SphDamBreak2D.Width * scale;
            float height = SphDamBreak2D.Height * scale;
            Vector2 origin = new Vector2(area.center.x - width * .5f, area.center.y + height * .5f);
            DrawOutline(new Rect(origin.x, origin.y - height, width, height), 2, Color.white);
            DrawOccupiedCells(damBreak.CoarseOccupiedCells, damBreak.CoarseCellsAcross,
                SphDamBreak2D.CoarseCellWidth * scale, origin, 1.5f, new Color(1, 1, 1, .55f));
            DrawOccupiedCells(damBreak.FineOccupiedCells, damBreak.FineCellsAcross,
                SphDamBreak2D.FineCellWidth * scale, origin, 1, Color.white);

            if (!showParticles) return;
            foreach (int i in damBreak.ActiveParticles)
            {
                Vector2 position = damBreak.Position(i);
                Vector2 screen = origin + new Vector2(position.x, -position.y) * scale;
                bool fine = damBreak.IsFine(i);
                float size = Mathf.Max(2, (fine ? 2.5f : 4f) * scale);
                DrawRect(new Rect(screen.x - size * .5f, screen.y - size * .5f, size, size),
                    fine ? ParticleColor : CoarseParticleColor);
            }
        }

        private static void DrawOccupiedCells(IReadOnlyList<int> occupied, int columns, float cellPixels,
            Vector2 origin, float thickness, Color color)
        {
            foreach (int cell in occupied)
            {
                int x = cell % columns;
                int y = cell / columns;
                DrawOutline(new Rect(origin.x + x * cellPixels, origin.y - (y + 1) * cellPixels,
                    cellPixels, cellPixels), thickness, color);
            }
        }

        private static void DrawOutline(Rect bounds, float thickness, Color color)
        {
            float half = thickness * .5f;
            DrawRect(new Rect(bounds.xMin - half, bounds.yMin - half, bounds.width + thickness, thickness), color);
            DrawRect(new Rect(bounds.xMin - half, bounds.yMax - half, bounds.width + thickness, thickness), color);
            DrawRect(new Rect(bounds.xMin - half, bounds.yMin - half, thickness, bounds.height + thickness), color);
            DrawRect(new Rect(bounds.xMax - half, bounds.yMin - half, thickness, bounds.height + thickness), color);
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
