using UnityEngine;

namespace PhaseTransition.FluidFlow
{
    /// <summary>
    /// Scene-only setup and display for the first liquid-flow validation cases.
    /// This deliberately contains no fluid physics; it gives the standalone solver
    /// a clean, inspectable scene without reusing the phase-change prototype.
    /// </summary>
    public sealed class LiquidFlowValidationSetup : MonoBehaviour
    {
        public enum Case
        {
            HydrostaticRest,
            DamBreak
        }

        [Header("Physical domain (SI)")]
        [Min(0.01f)] public float domainWidthM = 0.10f;
        [Min(0.01f)] public float domainHeightM = 0.24f;
        [Min(0.001f)] public float sliceDepthM = 0.025f;
        [Min(1)] public int gridWidth = 64;
        [Min(1)] public int gridHeight = 96;

        [Header("Validation case initial condition")]
        public Case validationCase = Case.HydrostaticRest;
        [Range(0.05f, 0.95f)] public float initialFillHeightFraction = 0.5f;
        [Range(0.1f, 1f)] public float damInitialWidthFraction = 0.35f;
        [Range(0.1f, 1f)] public float damInitialHeightFraction = 0.7f;

        private void OnGUI()
        {
            float scale = Mathf.Clamp(Screen.height / 720f, 0.65f, 1.35f);
            float pad = 16f * scale;
            float panelWidth = 330f * scale;
            float panelHeight = 250f * scale;

            GUI.color = new Color(0.035f, 0.045f, 0.06f, 0.96f);
            GUI.Box(new Rect(pad, pad, panelWidth, panelHeight), GUIContent.none);
            GUI.color = Color.white;

            GUILayout.BeginArea(new Rect(pad + 12f * scale, pad + 10f * scale,
                panelWidth - 24f * scale, panelHeight - 20f * scale));
            GUI.skin.label.fontSize = Mathf.RoundToInt(18f * scale);
            GUILayout.Label("LIQUID FLOW VALIDATION");
            GUI.skin.label.fontSize = Mathf.RoundToInt(12f * scale);
            GUILayout.Label("Step 1 setup | SI domain | no thermal or phase model");
            GUILayout.Space(8f * scale);

            GUILayout.BeginHorizontal();
            if (GUILayout.Toggle(validationCase == Case.HydrostaticRest, "Hydrostatic rest", GUI.skin.button))
                validationCase = Case.HydrostaticRest;
            if (GUILayout.Toggle(validationCase == Case.DamBreak, "Dam break", GUI.skin.button))
                validationCase = Case.DamBreak;
            GUILayout.EndHorizontal();

            GUILayout.Space(6f * scale);
            GUILayout.Label(validationCase == Case.HydrostaticRest
                ? "Initial condition: still water at rest. The solver must hold a level free surface under gravity."
                : "Initial condition: a stationary water column is released. The solver must spread and settle under gravity.");
            GUILayout.Space(6f * scale);
            GUILayout.Label($"Domain: {domainWidthM:0.000} x {domainHeightM:0.000} m   depth: {sliceDepthM:0.000} m");
            GUILayout.Label($"Grid: {gridWidth} x {gridHeight}   cell: {domainWidthM / gridWidth * 1000f:0.00} x {domainHeightM / gridHeight * 1000f:0.00} mm");
            GUI.color = new Color(1f, 0.75f, 0.35f);
            GUILayout.Label("Solver: not connected yet — this scene is the clean test rig.");
            GUI.color = Color.white;
            GUILayout.EndArea();

            DrawDomain(scale);
        }

        private void DrawDomain(float scale)
        {
            float availableWidth = Screen.width * 0.40f;
            float availableHeight = Screen.height * 0.72f;
            float aspect = domainWidthM / domainHeightM;
            float height = Mathf.Min(availableHeight, availableWidth / aspect);
            float width = height * aspect;
            float x = Screen.width - width - 50f * scale;
            float y = (Screen.height - height) * 0.5f;
            Rect vessel = new Rect(x, y, width, height);

            GUI.color = Color.white;
            GUI.Label(new Rect(x, y - 28f * scale, width + 50f, 24f * scale),
                "2D LIQUID DOMAIN  |  OPEN TOP");

            float fillWidth = validationCase == Case.HydrostaticRest
                ? 1f : damInitialWidthFraction;
            float fillHeight = validationCase == Case.HydrostaticRest
                ? initialFillHeightFraction : damInitialHeightFraction;
            Rect fluid = new Rect(vessel.x + 2f, vessel.yMax - vessel.height * fillHeight,
                (vessel.width - 4f) * fillWidth, vessel.height * fillHeight - 2f);

            DrawSolidRect(vessel, new Color(0.025f, 0.035f, 0.05f, 1f));
            DrawSolidRect(fluid, new Color(0.08f, 0.40f, 0.95f, 1f));
            Color wallColour = new Color(0.65f, 0.88f, 1f, 1f);
            float wall = Mathf.Max(3f, 4f * scale);
            DrawSolidRect(new Rect(vessel.x - wall, vessel.y, wall, vessel.height), wallColour);
            DrawSolidRect(new Rect(vessel.xMax, vessel.y, wall, vessel.height), wallColour);
            DrawSolidRect(new Rect(vessel.x - wall, vessel.yMax, vessel.width + 2f * wall, wall), wallColour);
            GUI.color = Color.white;
        }

        private static void DrawSolidRect(Rect rect, Color colour)
        {
            GUI.color = colour;
            GUI.DrawTexture(rect, Texture2D.whiteTexture, ScaleMode.StretchToFill, false);
        }
    }
}
