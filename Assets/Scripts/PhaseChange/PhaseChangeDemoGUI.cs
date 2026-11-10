using UnityEngine;

namespace PhaseTransition.Physics
{
    /// <summary>Simple IMGUI presentation for the initial SI beaker and lattice-gas prototypes.</summary>
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
        private Vector2[] iceSites;
        private Vector2[] liquidSites;
        private Vector2[] vapourSites;

        private void Awake()
        {
            ledger = GetComponent<BeakerEnergyLedger>();
            if (ledger == null) ledger = gameObject.AddComponent<BeakerEnergyLedger>();
            const int width = 28, height = 18;
            lattice = new LatticeGas2D(width, height, width * height / 2, latticeTemperatureK,
                LatticeGas2D.PairEnergyForSquareLatticeCriticalTemperature(647.096));
            CreateParticleDisplay();
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
            GUILayout.BeginArea(new Rect(12, 12, panelWidth, Screen.height - 24), GUI.skin.box);
            GUILayout.Label("PHASE TRANSITION STUDY", titleStyle);
            GUILayout.BeginHorizontal();
            if (GUILayout.Toggle(view == View.Beaker, "SI beaker", GUI.skin.button)) view = View.Beaker;
            if (GUILayout.Toggle(view == View.LatticeGas, "Lattice gas", GUI.skin.button)) view = View.LatticeGas;
            GUILayout.EndHorizontal();
            if (view == View.Beaker) DrawBeakerControls();
            else DrawLatticeControls();
            GUILayout.EndArea();

            if (view == View.Beaker) DrawSchematic();
            else DrawLatticeGrid();
        }

        private void DrawBeakerControls()
        {
            if (ledger == null) return;
            GUILayout.Label("Uniform-sample SI enthalpy ledger; schematic is not to scale.", noteStyle);
            GUILayout.Label("Display markers: white ice, blue liquid, white vapour. They are representative, not individual molecules.", noteStyle);
            GUILayout.Space(8);
            ledger.heaterEnabled = GUILayout.Toggle(ledger.heaterEnabled, "Heater on");
            GUILayout.Label($"Heater power: {ledger.heaterPowerW:0} W");
            ledger.heaterPowerW = GUILayout.HorizontalSlider(ledger.heaterPowerW, 0f, 1500f);
            GUILayout.Label($"Sample mass: {ledger.sampleMassKg:0.00} kg");
            ledger.sampleMassKg = GUILayout.HorizontalSlider(ledger.sampleMassKg, 0.05f, 1.00f);
            GUILayout.Label($"Altitude: {ledger.altitudeM:0} m  |  pressure: {ledger.AmbientPressurePa / 1000.0:0.00} kPa");
            ledger.altitudeM = GUILayout.HorizontalSlider(ledger.altitudeM, 0f, 3000f);
            GUILayout.Space(8);
            var s = ledger.CurrentState;
            GUILayout.Label($"Phase: {s.phase}");
            GUILayout.Label($"Temperature: {s.temperatureC:0.00} deg C");
            GUILayout.Label($"Ice: {s.iceMassFraction:P1}   liquid: {s.liquidMassFraction:P1}   vapour: {s.vapourMassFraction:P1}");
            GUILayout.Label($"Specific enthalpy: {ledger.SpecificEnthalpyJPerKg / 1000.0:0.0} kJ/kg");
            if (GUILayout.Button("Reset to ice at -50 deg C")) ledger.ResetSample();
            GUILayout.Space(5);
            GUILayout.Label("Currently omitted: spatial conduction/convection, beaker heat capacity, heat loss, nucleation and vapour transport.", noteStyle);
        }

        private void DrawLatticeControls()
        {
            GUILayout.Label("Fixed-N 2-D lattice gas; one occupied particle at most per site.", noteStyle);
            GUILayout.Label($"Temperature: {latticeTemperatureK:0} K");
            latticeTemperatureK = GUILayout.HorizontalSlider(latticeTemperatureK, 100f, 900f);
            GUILayout.Label($"Occupied sites: {lattice.ParticleCount} / {lattice.Width * lattice.Height}");
            GUILayout.Label($"Occupied neighbour pairs: {lattice.CountOccupiedNeighbourPairs()}");
            GUILayout.Label($"Configurational energy: {lattice.ConfigurationalEnergyJ:E3} J");
            GUILayout.Space(5);
            GUILayout.Label("Nearest-neighbour attraction, Metropolis updates; particle count is conserved.", noteStyle);
            GUILayout.Label("Pair energy is calibrated only to the 647.096 K critical-point coordinate. This is not quantitative water physics.", noteStyle);
        }

        private void DrawSchematic()
        {
            if (ledger == null) return;
            float x = Mathf.Min(Screen.width - 260f, 500f);
            float y = Mathf.Max(85f, Screen.height * 0.20f);
            float w = Mathf.Min(250f, Screen.width * 0.30f);
            float h = Mathf.Min(330f, Screen.height * 0.48f);
            float wall = 8f;
            GUI.color = new Color(0.72f, 0.87f, 0.91f, 0.8f);
            GUI.Box(new Rect(x, y, wall, h), GUIContent.none);
            GUI.Box(new Rect(x + w - wall, y, wall, h), GUIContent.none);
            GUI.Box(new Rect(x, y + h - wall, w, wall), GUIContent.none);

            var s = ledger.CurrentState;
            float sampleFraction = Mathf.Clamp01((float)(s.iceMassFraction + s.liquidMassFraction));
            float fillHeight = (h - wall * 2) * sampleFraction * 0.70f;
            if (fillHeight > 0.5f)
            {
                Color sampleColor = s.phase == WaterThermodynamicsSI.Phase.SolidIce ||
                                    s.phase == WaterThermodynamicsSI.Phase.MeltingMixture
                    ? new Color(0.22f, 0.42f, 0.58f, 0.90f)
                    : new Color(0.02f, 0.15f, 0.42f, 0.88f);
                GUI.color = sampleColor;
                GUI.Box(new Rect(x + wall + 4f, y + h - wall - fillHeight, w - 2f * (wall + 4f), fillHeight), GUIContent.none);
            }

            Rect interior = new Rect(x + wall + 4f, y + wall + 4f,
                w - 2f * (wall + 4f), h - 2f * (wall + 4f));
            float condensedFraction = Mathf.Clamp01((float)(s.iceMassFraction + s.liquidMassFraction));
            float condensedHeight = interior.height * condensedFraction * 0.70f;
            Rect condensedRegion = new Rect(interior.x, interior.y + interior.height - condensedHeight,
                interior.width, condensedHeight);
            Rect gasRegion = new Rect(interior.x, interior.y, interior.width,
                Mathf.Max(1f, interior.height - condensedHeight));

            float iceCount = 90f * (float)s.iceMassFraction;
            float liquidCount = 110f * (float)s.liquidMassFraction;
            // Gas occupies much more volume per unit mass than liquid, so use a display
            // scaling rather than drawing equal marker counts for equal mass fractions.
            float vapourCount = s.vapourMassFraction <= 0.0001 ? 0f :
                Mathf.Clamp(140f * Mathf.Sqrt((float)s.vapourMassFraction), 1f, 140f);

            if (s.vapourMassFraction > 0.001)
            {
                GUI.color = new Color(0.72f, 0.81f, 0.88f, 0.20f);
                float vapourHeight = Mathf.Max(16f, (h - wall * 2) * (float)s.vapourMassFraction * 0.35f);
                GUI.Box(new Rect(x + wall + 4f, y + wall + 4f, w - 2f * (wall + 4f), vapourHeight), GUIContent.none);
            }
            DrawPhaseParticles(iceSites, iceCount, condensedRegion, Color.white, 0f, false, 7f);
            DrawPhaseParticles(liquidSites, liquidCount, condensedRegion,
                new Color(0.10f, 0.48f, 1f, 1f), 0.012f, false, 6f);
            DrawPhaseParticles(vapourSites, vapourCount, gasRegion, Color.white, 0.08f, true, 5f);

            if (ledger.heaterEnabled && ledger.heaterPowerW > 0f)
            {
                GUI.color = new Color(1f, 0.28f, 0.06f, 0.95f);
                GUI.Box(new Rect(x + w * 0.30f, y + h + 15f, w * 0.40f, 26f), "HEATER");
                GUI.color = new Color(1f, 0.55f, 0.08f, 0.65f);
                GUI.Box(new Rect(x + w * 0.40f, y + h - 2f, w * 0.20f, 17f), GUIContent.none);
            }
            GUI.color = Color.white;
            GUI.Label(new Rect(x, y - 24f, w + 80f, 22f), "OPEN GLASS BEAKER  |  SEA-LEVEL REFERENCE");
            GUI.Label(new Rect(x, y + h + 44f, w + 100f, 42f), "Ice markers are ordered; liquid markers move locally; vapour markers spread through the headspace.", noteStyle);
        }

        private void CreateParticleDisplay()
        {
            const int displayCount = 140;
            iceSites = new Vector2[displayCount];
            liquidSites = new Vector2[displayCount];
            vapourSites = new Vector2[displayCount];
            var random = new System.Random(7319);
            for (int i = 0; i < displayCount; i++)
            {
                // Ordered points communicate a solid lattice; this is not a water crystal model.
                iceSites[i] = new Vector2((i % 14 + 0.5f) / 14f, (i / 14 + 0.5f) / 10f);
                liquidSites[i] = new Vector2((float)random.NextDouble(), (float)random.NextDouble());
                vapourSites[i] = new Vector2((float)random.NextDouble(), (float)random.NextDouble());
            }

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

        private void DrawPhaseParticles(Vector2[] sites, float count, Rect region, Color colour,
            float motion, bool rising, float diameter)
        {
            if (sites == null || count <= 0 || region.width <= 0f || region.height <= 1f) return;
            float time = Time.time;
            for (int i = 0; i < Mathf.CeilToInt(count); i++)
            {
                Vector2 p = sites[i];
                float px = p.x;
                float py = p.y;
                if (rising)
                {
                    py = Mathf.Repeat(py + time * motion, 1f);
                    px = Mathf.Repeat(px + Mathf.Sin(time * 1.7f + i * 1.91f) * 0.035f, 1f);
                }
                else if (motion > 0f)
                {
                    px = Mathf.Clamp01(px + Mathf.Sin(time * 1.1f + i * 2.17f) * motion);
                    py = Mathf.Clamp01(py + Mathf.Sin(time * 0.8f + i * 1.37f) * motion);
                }
                Rect marker = new Rect(region.x + px * region.width - diameter * 0.5f,
                    region.y + py * region.height - diameter * 0.5f, diameter, diameter);
                float markerAlpha = Mathf.Clamp01(count - i);
                GUI.color = new Color(colour.r, colour.g, colour.b, colour.a * markerAlpha);
                GUI.DrawTexture(marker, particleTexture, ScaleMode.StretchToFill, true);
            }
            GUI.color = Color.white;
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
