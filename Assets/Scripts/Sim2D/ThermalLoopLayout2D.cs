using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace Seb.Fluid2D.Simulation
{
    /// <summary>
    /// Draws the conceptual thermal-loop apparatus as both Scene-view gizmos and
    /// runtime line renderers visible from the Game camera.
    /// </summary>
    public sealed class ThermalLoopLayout2D : MonoBehaviour
    {
        [Header("Wireframe vessels")]
        public Vector2 primaryTankCentre = new Vector2(-10f, 24f);
        public Vector2 primaryTankSize = new Vector2(9f, 7f);
        public Vector2 heatedTankCentre = new Vector2(0f, 8f);
        public Vector2 heatedTankSize = new Vector2(65f, 36f);
        public Vector2 reclamationTankCentre = new Vector2(15f, -8f);
        public Vector2 reclamationTankSize = new Vector2(10f, 7f);

        [Header("Spout and pipe settings")]
        [Range(-70f, 70f)] public float spoutAngleDegrees = -25f;
        [Range(0.25f, 3f)] public float spoutOpening = 1f;
        public bool inflowEnabled = true;
        public bool drainEnabled = true;
        [Min(0)] public float gravityFeedRate = 260f;
        [Min(0.1f)] public float sourceTankHead = 8f;
        [Range(0.1f, 1f)] public float sourceTankFill = 0.75f;
        [Range(-20f, 20f)] public float inletOffsetX = -10f;
        [Range(0.25f, 5f)] public float drainOpening = 2.5f;
        [Range(-20f, 20f)] public float drainLocalX = 15f;

        [Header("Reclamation pump")]
        [Range(0f, 1f)] public float reclamationFill;
        [Range(0f, 1f)] public float pumpStartLevel = 0.65f;
        public bool impellerEnabled = true;
        [Range(0.01f, 0.5f)] public float pumpRate = 0.12f;

        [Header("Burner")]
        [Range(0f, 1f)] public float airCollarOpening = 0.55f;
        public bool burnerEnabled = true;
        [Range(0f, 100f)] public float burnerPower = 75f;
        [Range(0.5f, 8f)] public float burnerDepth = 4f;
        [Range(0f, 30f)] public float burnerLocalX;
        [Range(0f, 1f)] public float thermalCooling = 0.08f;
        [Tooltip("Reference temperature for water and vessel cooling, in degrees Celsius.")]
        public float ambientTemperatureC = 20f;
        [Header("Thermal transport")]
        [Tooltip("Metres represented by one Unity simulation unit. Default: 1 unit = 1 cm.")]
        [Min(0.0001f)] public float metresPerSimulationUnit = 0.01f;
        [Tooltip("AISI 304 stainless steel thermal diffusivity, approximately 4.2e-6 m^2/s near room temperature.")]
        [Min(0f)] public float wallThermalDiffusivity = 4.2e-6f;
        [Tooltip("Relative thermal mass of the metal base; larger values make it warm more slowly.")]
        [Min(0.1f)] public float wallHeatCapacityScale = 64f;
        [Tooltip("Wall-to-water contact transfer rate in inverse simulation seconds.")]
        [Min(0f)] public float wallWaterTransferRate = 4f;
        [Tooltip("Passive heat loss rate from the vessel base to ambient air, per second.")]
        [Min(0f)] public float wallAmbientCoolingRate = 0.02f;
        [Tooltip("Liquid-water thermal diffusivity near room temperature, in m^2/s (IAPWS-based).")]
        [Min(0f)] public float liquidThermalDiffusivity = 1.43e-7f;
        [Tooltip("Water-vapour thermal diffusivity near 100 C, approximate, in m^2/s.")]
        [Min(0f)] public float vapourThermalDiffusivity = 2.1e-5f;
        [Tooltip("Dimensionless multiplier on the calibrated conduction rate.")]
        [Min(0f)] public float thermalDiffusionRate = 1f;
        [Tooltip("Approximate boiling temperature at atmospheric pressure, in degrees Celsius.")]
        [Min(0.1f)] public float evaporationThreshold = 100f;
        [Tooltip("Fallback condensation threshold, in degrees Celsius; immersed vapour also compares with surrounding liquid temperature.")]
        [Min(0f)] public float condensationThreshold = 98f;
        [Tooltip("Spacing between simplified active boiling sites along the heated base, in Unity units.")]
        [Min(0.1f)] public float nucleationSiteSpacing = 2.5f;
        [Tooltip("Simplified latent-heat temperature debit in Celsius-equivalent simulation units; not a joule-per-kilogram value.")]
        [Min(0f)] public float latentHeat = 2f;
        [Tooltip("Multiplier on vapour thermal diffusion; it exchanges heat with nearby phases faster than liquid water.")]
        [Min(1f)] public float vapourCoolingMultiplier = 3f;
        [Range(0.1f, 1f)] public float surfaceDensityFraction = 0.78f;
        [Range(0.01f, 0.5f)] public float nearSurfaceDensityFraction = 0.02f;
        [Tooltip("Boussinesq thermal expansion coefficient for water (1/K), used to drive natural convection.")]
        [Min(0f)] public float liquidThermalBuoyancy = 0.00021f;
        [Min(0f)] public float vapourBuoyancy = 36f;
        [Tooltip("Coarse-grained vapour/liquid density ratio. Kept far above the physical steam ratio because this particle resolution cannot represent steam's full expansion.")]
        [Range(0.01f, 0.5f)] public float vapourDensityRatio = 0.1f;
        [Range(0f, 1f)] public float vapourPressureScale = 0.15f;
        [Range(0f, 1f)] public float vapourViscosityScale = 0.03f;
        [Tooltip("Cross-phase repulsion strength; prevents vapour particles passing through liquid particles.")]
        [Min(0f)] public float interphaseRepulsion = 0.2f;
        [Tooltip("Cross-phase drag strength; couples bubble velocity to surrounding water.")]
        [Min(0f)] public float interphaseDrag = 8f;
        public bool pressureVentEnabled = true;
        [Range(-20f, 20f)] public float pressureVentLocalX;
        [Range(0.5f, 10f)] public float pressureVentWidth = 3f;
        [Min(0f)] public float pressureVentDensityThreshold = 100f;

        private float hudScroll;
        private FluidSim2D fluidSim;

        private void Start()
        {
            fluidSim = GetComponent<FluidSim2D>();
            FlamePlume2D plume = GetComponent<FlamePlume2D>();
            if (plume == null) plume = gameObject.AddComponent<FlamePlume2D>();
            plume.thermalLoop = this;
            plume.fluidSim = GetComponent<FluidSim2D>();
        }

        public float GetParticleRate()
        {
            if (!inflowEnabled || sourceTankFill <= 0f) return 0f;
            return gravityFeedRate * spoutOpening * Mathf.Sqrt(sourceTankFill);
        }

        public Vector2 GetInletPosition(Vector2 boundsCentre, Vector2 boundsSize)
        {
            return new Vector2(boundsCentre.x + inletOffsetX, boundsCentre.y + boundsSize.y * 0.5f - 0.3f);
        }

        public Vector2 GetInletVelocity(float gravity)
        {
            float speed = Mathf.Sqrt(2f * Mathf.Max(0.1f, Mathf.Abs(gravity)) * sourceTankHead);
            Vector2 direction = Quaternion.Euler(0f, 0f, spoutAngleDegrees) * Vector2.right;
            return direction * speed;
        }

        public void RegisterDrainedParticles(uint count)
        {
            if (count == 0) return;
            reclamationFill = Mathf.Clamp01(reclamationFill + count / 50000f);
        }

        private void Update()
        {
            if (!Application.isPlaying) return;
            if (Input.GetKeyDown(KeyCode.B)) burnerEnabled = !burnerEnabled;
            if (Input.GetKeyDown(KeyCode.LeftBracket)) airCollarOpening = Mathf.Clamp01(airCollarOpening - 0.05f);
            if (Input.GetKeyDown(KeyCode.RightBracket)) airCollarOpening = Mathf.Clamp01(airCollarOpening + 0.05f);
        }

        private void OnGUI()
        {
            if (!Application.isPlaying) return;

            if (fluidSim == null) fluidSim = GetComponent<FluidSim2D>();
            GUILayout.BeginArea(new Rect(12, 44, 320, 390), GUI.skin.box);
            GUILayout.Label("THERMAL LOOP CONTROLS");
            burnerEnabled = GUILayout.Toggle(burnerEnabled, "Burner ON (B)");
            GUILayout.Label($"Heat power: {burnerPower:0}");
            burnerPower = GUILayout.HorizontalSlider(burnerPower, 0, 100);
            GUILayout.Label($"Air collar [ ]: {airCollarOpening:0.00}");
            airCollarOpening = GUILayout.HorizontalSlider(airCollarOpening, 0, 1);
            GUILayout.Label("Focused collar narrows the flame and increases energy input.");
            if (fluidSim != null)
            {
                fluidSim.obstacleEnabled = GUILayout.Toggle(fluidSim.obstacleEnabled, "Internal black bar (O)");
                pressureVentEnabled = GUILayout.Toggle(pressureVentEnabled, "Automatic pressure relief valve");
                GUILayout.Label($"Water temperature: �C | boil at {evaporationThreshold:0.0} | condense below {condensationThreshold:0.0}");
                GUILayout.Label("Liquid thermal buoyancy drives convection currents.");

                GUILayout.BeginHorizontal();
                GUILayout.Label($"Wall rebound retention: {fluidSim.collisionDamping:0.000}", GUILayout.Width(215));
                if (GUILayout.Button("-", GUILayout.Width(28))) fluidSim.collisionDamping = Mathf.Clamp01(fluidSim.collisionDamping - 0.001f);
                if (GUILayout.Button("+", GUILayout.Width(28))) fluidSim.collisionDamping = Mathf.Clamp01(fluidSim.collisionDamping + 0.001f);
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GUILayout.Label($"Heat decay: {thermalCooling:0.000}/s", GUILayout.Width(215));
                if (GUILayout.Button("-", GUILayout.Width(28))) thermalCooling = Mathf.Clamp(thermalCooling - 0.001f, 0f, 1f);
                if (GUILayout.Button("+", GUILayout.Width(28))) thermalCooling = Mathf.Clamp(thermalCooling + 0.001f, 0f, 1f);
                GUILayout.EndHorizontal();
            }
            GUILayout.EndArea();
        }

        private readonly List<LineRenderer> runtimeLines = new List<LineRenderer>();
        private Material runtimeMaterial;
        private int lineCursor;
        private const float RenderZ = -0.25f;
        private const float LineWidth = 0.09f;

        private void OnEnable()
        {
            if (Application.isPlaying)
                CreateRuntimeMaterial();
        }

        private void LateUpdate()
        {
            if (!Application.isPlaying)
                return;

            if (runtimeMaterial == null)
                CreateRuntimeMaterial();
            if (runtimeMaterial == null)
                return;

            lineCursor = 0;
            DrawRuntimeTankWithVent(heatedTankCentre, heatedTankSize, new Color(0.25f, 0.75f, 1f));

            Vector2 burner = new Vector2(heatedTankCentre.x + burnerLocalX, heatedTankCentre.y - heatedTankSize.y * 0.5f - 4f);
            Color metal = new Color(0.75f, 0.78f, 0.82f);
            DrawRectangle(burner + Vector2.down * 1.4f, new Vector2(3f, 1.2f), metal);
            AddLine(burner + Vector2.left * 1.5f, burner + Vector2.left * 1.5f + Vector2.down * 2f, metal);
            AddLine(burner + Vector2.right * 1.5f, burner + Vector2.right * 1.5f + Vector2.down * 2f, metal);

            if (pressureVentEnabled)
            {
                float top = heatedTankCentre.y + heatedTankSize.y * 0.5f;
                float ventX = heatedTankCentre.x + pressureVentLocalX;
                AddLine(new Vector2(ventX, top), new Vector2(ventX, top + 1.4f), metal);
                DrawRectangle(new Vector2(ventX, top + 1.4f), new Vector2(1.3f, 0.35f), metal);
                AddLine(new Vector2(ventX, top + 1.4f), new Vector2(ventX + 0.9f, top + 2.1f), metal);
            }

            for (int i = lineCursor; i < runtimeLines.Count; i++)
                runtimeLines[i].enabled = false;
        }

        private void CreateRuntimeMaterial()
        {
            Shader shader = Shader.Find("Sprites/Default");
            if (shader == null)
                shader = Shader.Find("Unlit/Color");
            if (shader != null)
                runtimeMaterial = new Material(shader) { name = "Thermal Loop Wireframe (Runtime)" };
        }

        private LineRenderer GetLine(Color colour)
        {
            LineRenderer line;
            if (lineCursor < runtimeLines.Count)
            {
                line = runtimeLines[lineCursor];
            }
            else
            {
                GameObject go = new GameObject("Thermal Loop Wire");
                go.transform.SetParent(transform, false);
                line = go.AddComponent<LineRenderer>();
                line.useWorldSpace = true;
                line.alignment = LineAlignment.View;
                line.shadowCastingMode = ShadowCastingMode.Off;
                line.receiveShadows = false;
                line.numCapVertices = 2;
                line.numCornerVertices = 2;
                line.positionCount = 2;
                runtimeLines.Add(line);
            }
            lineCursor++;
            line.enabled = true;
            line.sharedMaterial = runtimeMaterial;
            line.startColor = colour;
            line.endColor = colour;
            line.startWidth = LineWidth;
            line.endWidth = LineWidth;
            line.sortingOrder = 1000;
            return line;
        }

        private void AddLine(Vector2 a, Vector2 b, Color colour)
        {
            LineRenderer line = GetLine(colour);
            line.SetPosition(0, new Vector3(a.x, a.y, RenderZ));
            line.SetPosition(1, new Vector3(b.x, b.y, RenderZ));
        }

        private void DrawRuntimeTank(Vector2 centre, Vector2 size, Color colour)
        {
            DrawRectangle(centre, size, colour);
        }

        private void DrawRuntimeTankWithVent(Vector2 centre, Vector2 size, Color colour)
        {
            if (!pressureVentEnabled)
            {
                DrawRuntimeTank(centre, size, colour);
                return;
            }

            Vector2 half = size * 0.5f;
            float left = centre.x - half.x;
            float right = centre.x + half.x;
            float bottom = centre.y - half.y;
            float top = centre.y + half.y;
            float ventLeft = Mathf.Clamp(centre.x + pressureVentLocalX - pressureVentWidth * 0.5f, left, right);
            float ventRight = Mathf.Clamp(centre.x + pressureVentLocalX + pressureVentWidth * 0.5f, left, right);
            AddLine(new Vector2(left, bottom), new Vector2(right, bottom), colour);
            AddLine(new Vector2(left, bottom), new Vector2(left, top), colour);
            AddLine(new Vector2(right, bottom), new Vector2(right, top), colour);
            AddLine(new Vector2(left, top), new Vector2(ventLeft, top), colour);
            AddLine(new Vector2(ventRight, top), new Vector2(right, top), colour);
        }

        private void DrawRectangle(Vector2 centre, Vector2 size, Color colour)
        {
            Vector2 half = size * 0.5f;
            Vector2 a = centre + new Vector2(-half.x, -half.y);
            Vector2 b = centre + new Vector2(half.x, -half.y);
            Vector2 c = centre + new Vector2(half.x, half.y);
            Vector2 d = centre + new Vector2(-half.x, half.y);
            AddLine(a, b, colour); AddLine(b, c, colour);
            AddLine(c, d, colour); AddLine(d, a, colour);
        }

        private void AddCircle(Vector2 centre, float radius, Color colour)
        {
            const int segments = 16;
            Vector2 previous = centre + Vector2.right * radius;
            for (int i = 1; i <= segments; i++)
            {
                float angle = i * (2f * Mathf.PI / segments);
                Vector2 next = centre + new Vector2(Mathf.Cos(angle), Mathf.Sin(angle)) * radius;
                AddLine(previous, next, colour);
                previous = next;
            }
        }

        private void OnDrawGizmos()
        {
            if (pressureVentEnabled)
                DrawGizmoTankWithVent(heatedTankCentre, heatedTankSize, new Color(0.25f, 0.75f, 1f));
            else
                DrawGizmoTank(heatedTankCentre, heatedTankSize, new Color(0.25f, 0.75f, 1f));
            Gizmos.color = new Color(0.8f, 0.8f, 0.82f);
            Vector2 burner = new Vector2(heatedTankCentre.x + burnerLocalX, heatedTankCentre.y - heatedTankSize.y * 0.5f - 4f);
            Gizmos.DrawWireCube(burner + Vector2.down * 1.4f, new Vector3(3f, 1.2f, 0f));
        }

        private void DrawGizmoTankWithVent(Vector2 centre, Vector2 size, Color colour)
        {
            Vector2 half = size * 0.5f;
            float left = centre.x - half.x;
            float right = centre.x + half.x;
            float bottom = centre.y - half.y;
            float top = centre.y + half.y;
            float ventLeft = Mathf.Clamp(centre.x + pressureVentLocalX - pressureVentWidth * 0.5f, left, right);
            float ventRight = Mathf.Clamp(centre.x + pressureVentLocalX + pressureVentWidth * 0.5f, left, right);
            Gizmos.color = colour;
            Gizmos.DrawLine(new Vector2(left, bottom), new Vector2(right, bottom));
            Gizmos.DrawLine(new Vector2(left, bottom), new Vector2(left, top));
            Gizmos.DrawLine(new Vector2(right, bottom), new Vector2(right, top));
            Gizmos.DrawLine(new Vector2(left, top), new Vector2(ventLeft, top));
            Gizmos.DrawLine(new Vector2(ventRight, top), new Vector2(right, top));
            Gizmos.color = new Color(0.8f, 0.8f, 0.82f);
            float ventX = centre.x + pressureVentLocalX;
            Gizmos.DrawLine(new Vector2(ventX, top), new Vector2(ventX, top + 1.4f));
            Gizmos.DrawWireCube(new Vector2(ventX, top + 1.4f), new Vector3(1.3f, 0.35f, 0f));
            Gizmos.DrawLine(new Vector2(ventX, top + 1.4f), new Vector2(ventX + 0.9f, top + 2.1f));
        }

        private static void DrawGizmoTank(Vector2 centre, Vector2 size, Color colour)
        {
            Gizmos.color = colour;
            Gizmos.DrawWireCube(centre, size);
        }

        private void OnDestroy()
        {
            if (runtimeMaterial != null)
            {
                if (Application.isPlaying) Destroy(runtimeMaterial);
                else DestroyImmediate(runtimeMaterial);
            }
        }
    }
}
