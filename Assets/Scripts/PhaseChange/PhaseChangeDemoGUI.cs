using UnityEngine;

namespace PhaseTransition.Physics
{
    /// <summary>IMGUI presentation for the spatial beaker model and separate lattice-gas prototype.</summary>
    public sealed class PhaseChangeDemoGUI : MonoBehaviour
    {
        private enum View { Beaker, LatticeGas }
        private View view;
        private BeakerEnergyLedger ledger;
        private LatticeGas2D lattice;
        private float latticeTemperatureK = 300f;
        private GUIStyle titleStyle;
        private GUIStyle noteStyle;
        private Texture2D particleTexture;
        private bool showFlowVectors = true;

        private void Awake()
        {
            ledger = GetComponent<BeakerEnergyLedger>();
            if (ledger == null) ledger = gameObject.AddComponent<BeakerEnergyLedger>();
            const int width = 28, height = 18;
            lattice = new LatticeGas2D(width, height, width * height / 2, latticeTemperatureK,
                LatticeGas2D.PairEnergyForSquareLatticeCriticalTemperature(647.096));
            CreateParticleTexture();
        }

        private void OnDestroy()
        {
            if (particleTexture != null) Destroy(particleTexture);
        }

        private void Update()
        {
            if (view == View.LatticeGas && lattice != null)
            {
                lattice.TemperatureK = latticeTemperatureK;
                lattice.Sweep();
            }
        }

        private void OnGUI()
        {
            EnsureStyles();
            float panelWidth = Mathf.Min(430f, Screen.width - 24f);
            GUILayout.BeginArea(new Rect(12f, 12f, panelWidth, Screen.height - 24f), GUI.skin.box);
            GUILayout.Label("PHASE TRANSITION STUDY", titleStyle);
            GUILayout.BeginHorizontal();
            if (GUILayout.Toggle(view == View.Beaker, "SI beaker", GUI.skin.button)) view = View.Beaker;
            if (GUILayout.Toggle(view == View.LatticeGas, "Lattice gas", GUI.skin.button)) view = View.LatticeGas;
            GUILayout.EndHorizontal();
            if (view == View.Beaker) DrawBeakerControls();
            else DrawLatticeControls();
            GUILayout.EndArea();

            if (view == View.Beaker) DrawBeaker();
            else DrawLatticeGrid();
        }

        private void DrawBeakerControls()
        {
            if (ledger == null) return;
            GUILayout.Label("Spatial SI enthalpy grid: each cell has its own temperature and phase fractions.", noteStyle);
            GUILayout.Label("White = ice or vapour; blue = liquid. Lines show the computed liquid-flow direction.", noteStyle);
            GUILayout.Space(8f);
            ledger.heaterEnabled = GUILayout.Toggle(ledger.heaterEnabled, "Heater on");
            GUILayout.Label($"Heater power: {ledger.heaterPowerW:0} W");
            ledger.heaterPowerW = GUILayout.HorizontalSlider(ledger.heaterPowerW, 0f, 1500f);
            GUILayout.Label($"Sample mass: {ledger.sampleMassKg:0.00} kg  |  grid: {ledger.GridWidth} x {ledger.GridHeight}");
            GUILayout.Label($"Altitude: {ledger.altitudeM:0} m  |  ambient pressure: {ledger.AmbientPressurePa / 1000.0:0.00} kPa");
            ledger.altitudeM = GUILayout.HorizontalSlider(ledger.altitudeM, 0f, 3000f);
            GUILayout.Space(8f);
            GUILayout.Label($"Mass-weighted temperature: {ledger.AverageTemperatureC:0.00} deg C");
            GUILayout.Label($"Ice: {ledger.IceMassFraction:P1}   liquid: {ledger.LiquidMassFraction:P1}   vapour: {ledger.VapourMassFraction:P1}");
            GUILayout.Label($"Specific enthalpy: {ledger.SpecificEnthalpyJPerKg / 1000.0:0.0} kJ/kg");
            GUILayout.Label($"Peak liquid speed: {ledger.MaximumLiquidSpeedMS:0.000} m/s");
            showFlowVectors = GUILayout.Toggle(showFlowVectors, "Show liquid flow direction");
            if (GUILayout.Button("Reset to ice at -50 deg C")) ledger.ResetSample();
            GUILayout.Space(5f);
            GUILayout.Label("Liquid uses a 2-D incompressible flow approximation with buoyancy and temperature-dependent viscosity. Flow carries liquid sensible heat. Solid movement, vapour flow, beaker heat capacity and 16 C ambient heat exchange are not included yet.", noteStyle);
        }

        private void DrawLatticeControls()
        {
            GUILayout.Label("Fixed-N 2-D lattice gas; one occupied particle at most per site.", noteStyle);
            GUILayout.Label($"Temperature: {latticeTemperatureK:0} K");
            latticeTemperatureK = GUILayout.HorizontalSlider(latticeTemperatureK, 100f, 900f);
            GUILayout.Label($"Occupied sites: {lattice.ParticleCount} / {lattice.Width * lattice.Height}");
            GUILayout.Label($"Occupied neighbour pairs: {lattice.CountOccupiedNeighbourPairs()}");
            GUILayout.Label($"Configurational energy: {lattice.ConfigurationalEnergyJ:E3} J");
            GUILayout.Space(5f);
            GUILayout.Label("Nearest-neighbour attraction, Metropolis updates; particle count is conserved.", noteStyle);
            GUILayout.Label("Pair energy is calibrated only to the 647.096 K critical-point coordinate. This is not quantitative water physics.", noteStyle);
        }

        private void DrawBeaker()
        {
            if (ledger == null || particleTexture == null) return;
            float x = Mathf.Min(Screen.width - 260f, 500f);
            float y = Mathf.Max(85f, Screen.height * 0.20f);
            float w = Mathf.Min(250f, Screen.width * 0.30f);
            float h = Mathf.Min(330f, Screen.height * 0.48f);
            const float wall = 8f;
            GUI.color = new Color(0.72f, 0.87f, 0.91f, 0.8f);
            GUI.Box(new Rect(x, y, wall, h), GUIContent.none);
            GUI.Box(new Rect(x + w - wall, y, wall, h), GUIContent.none);
            GUI.Box(new Rect(x, y + h - wall, w, wall), GUIContent.none);

            Rect interior = new Rect(x + wall + 4f, y + wall + 4f,
                w - 2f * (wall + 4f), h - 2f * (wall + 4f));
            float levelFraction = Mathf.Clamp01((float)(ledger.WaterLevelM / ledger.beakerHeightM));
            float fillHeight = interior.height * levelFraction;
            if (fillHeight > 0.5f)
            {
                GUI.color = new Color(0.02f, 0.15f, 0.30f, 0.88f);
                GUI.Box(new Rect(interior.x, interior.y + interior.height - fillHeight,
                    interior.width, fillHeight), GUIContent.none);
            }

            DrawLocalPhaseParticles(interior);
            if (showFlowVectors) DrawLiquidFlowVectors(interior);
            if (ledger.heaterEnabled && ledger.heaterPowerW > 0f)
            {
                GUI.color = new Color(1f, 0.28f, 0.06f, 0.95f);
                GUI.Box(new Rect(x + w * 0.30f, y + h + 15f, w * 0.40f, 26f), "HEATER");
                GUI.color = new Color(1f, 0.55f, 0.08f, 0.65f);
                GUI.Box(new Rect(x + w * 0.40f, y + h - 2f, w * 0.20f, 17f), GUIContent.none);
            }
            GUI.color = Color.white;
            GUI.Label(new Rect(x, y - 24f, w + 80f, 22f), "OPEN GLASS BEAKER  |  SEA-LEVEL REFERENCE");
            GUI.Label(new Rect(x, y + h + 44f, w + 120f, 44f), "Cells transition locally: mass-average temperature can rise while ice remains elsewhere.", noteStyle);
        }

        private void DrawLiquidFlowVectors(Rect interior)
        {
            float cellWidth = interior.width / ledger.GridWidth;
            float cellHeight = interior.height / ledger.GridHeight;
            for (int iy = 1; iy < ledger.GridHeight - 1; iy += 2)
                for (int ix = 1; ix < ledger.GridWidth - 1; ix += 2)
                {
                    BeakerEnergyLedger.CellState cell = ledger.GetCellState(ix, iy);
                    if (cell.liquidMassFraction < 0.2) continue;
                    float u = (float)cell.velocityXMS;
                    float v = -(float)cell.velocityYMS;
                    float speed = Mathf.Sqrt(u * u + v * v);
                    if (speed < 0.00005f) continue;

                    float centreX = interior.x + (ix + 0.5f) * cellWidth;
                    float centreY = interior.y + interior.height - (iy + 0.5f) * cellHeight;
                    float angle = Mathf.Atan2(v, u) * Mathf.Rad2Deg;
                    Matrix4x4 oldMatrix = GUI.matrix;
                    GUIUtility.RotateAroundPivot(angle, new Vector2(centreX, centreY));
                    GUI.color = new Color(0.92f, 0.96f, 1f, 0.8f);
                    GUI.DrawTexture(new Rect(centreX - cellWidth * 0.32f, centreY - 1f,
                        cellWidth * 0.64f, 2f), Texture2D.whiteTexture);
                    GUI.matrix = oldMatrix;
                    GUI.color = Color.white;
                }
        }

        private void DrawLocalPhaseParticles(Rect interior)
        {
            float cellWidth = interior.width / ledger.GridWidth;
            float cellHeight = interior.height / ledger.GridHeight;
            double cellVolume = (ledger.beakerWidthM / ledger.GridWidth) *
                                (ledger.beakerHeightM / ledger.GridHeight) * ledger.beakerDepthM;

            for (int iy = 0; iy < ledger.GridHeight; iy++)
                for (int ix = 0; ix < ledger.GridWidth; ix++)
                {
                    BeakerEnergyLedger.CellState cell = ledger.GetCellState(ix, iy);
                    if (!cell.containsSample) continue;

                    double condensedVolumeFraction = cell.massKg *
                        (cell.iceMassFraction / 917.0 + cell.liquidMassFraction / 998.2) / cellVolume;
                    float localVolumeFraction = Mathf.Clamp01((float)condensedVolumeFraction);
                    float centreX = interior.x + (ix + 0.5f) * cellWidth;
                    float centreY = interior.y + interior.height - (iy + localVolumeFraction * 0.5f) * cellHeight;
                    DrawParticle(new Rect(centreX - cellWidth * 0.18f - 3f, centreY - 3f, 6f, 6f),
                        Color.white, (float)cell.iceMassFraction);
                    DrawParticle(new Rect(centreX + cellWidth * 0.18f - 3f,
                        centreY - 3f, 6f, 6f),
                        new Color(0.12f, 0.52f, 1f, 1f), (float)cell.liquidMassFraction);

                    if (cell.vapourMassFraction > 0.001)
                    {
                        DrawParticle(new Rect(centreX - 2.5f, centreY - 2.5f, 5f, 5f),
                            Color.white, (float)cell.vapourMassFraction);
                    }
                }
        }

        private void DrawParticle(Rect rect, Color colour, float phaseFraction)
        {
            if (phaseFraction <= 0f) return;
            GUI.color = new Color(colour.r, colour.g, colour.b,
                colour.a * Mathf.Clamp01(phaseFraction));
            GUI.DrawTexture(rect, particleTexture, ScaleMode.StretchToFill, true);
            GUI.color = Color.white;
        }

        private void CreateParticleTexture()
        {
            const int textureSize = 16;
            particleTexture = new Texture2D(textureSize, textureSize, TextureFormat.RGBA32, false);
            particleTexture.name = "Phase particle display marker";
            particleTexture.filterMode = FilterMode.Bilinear;
            for (int py = 0; py < textureSize; py++)
                for (int px = 0; px < textureSize; px++)
                {
                    float dx = (px + 0.5f) / textureSize - 0.5f;
                    float dy = (py + 0.5f) / textureSize - 0.5f;
                    float alpha = Mathf.Clamp01((0.48f - Mathf.Sqrt(dx * dx + dy * dy)) * 18f);
                    particleTexture.SetPixel(px, py, new Color(1f, 1f, 1f, alpha));
                }
            particleTexture.Apply();
        }

        private void DrawLatticeGrid()
        {
            float cell = Mathf.Clamp(Mathf.Min((Screen.height - 50f) / lattice.Height,
                (Screen.width - 500f) / lattice.Width), 10f, 25f);
            float startX = Mathf.Min(Screen.width - lattice.Width * cell - 20f, 500f);
            float startY = Mathf.Max(65f, (Screen.height - lattice.Height * cell) * 0.5f);
            for (int y = 0; y < lattice.Height; y++)
                for (int x = 0; x < lattice.Width; x++)
                {
                    GUI.color = lattice.IsOccupied(x, y) ? new Color(0.09f, 0.66f, 0.72f) : new Color(0.92f, 0.89f, 0.82f);
                    GUI.Box(new Rect(startX + x * cell, startY + y * cell, cell - 1f, cell - 1f), GUIContent.none);
                }
            GUI.color = Color.white;
        }

        private void EnsureStyles()
        {
            if (titleStyle != null) return;
            titleStyle = new GUIStyle(GUI.skin.label) { fontSize = 19, fontStyle = FontStyle.Bold };
            noteStyle = new GUIStyle(GUI.skin.label) { wordWrap = true, fontSize = 12 };
        }
    }
}
