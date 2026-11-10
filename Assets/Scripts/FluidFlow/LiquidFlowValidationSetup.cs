using UnityEngine;

namespace PhaseTransition.FluidFlow
{
    /// <summary>
    /// Runs and displays the liquid-only MAC-grid validation scene. Fluid state is
    /// advanced by GPU compute kernels; the CPU handles controls and diagnostics.
    /// </summary>
    public sealed class LiquidFlowValidationSetup : MonoBehaviour
    {
        public enum Case { HydrostaticRest, DamBreak }

        [Header("GPU solver")]
        public ComputeShader fluidSolver;

        [Header("Physical domain (SI)")]
        [Min(0.01f)] public float domainWidthM = 0.10f;
        [Min(0.01f)] public float domainHeightM = 0.24f;
        [Min(0.001f)] public float sliceDepthM = 0.025f;
        [Min(8)] public int gridWidth = 64;
        [Min(8)] public int gridHeight = 96;
        [Min(0.01f)] public float liquidDensityKgPerM3 = 998.2f;
        [Min(0f)] public float kinematicViscosityM2PerS = 1.004e-6f;

        [Header("Validation case")]
        public Case validationCase = Case.DamBreak;
        [Range(0.05f, 0.95f)] public float initialFillHeightFraction = 0.5f;
        [Range(0.1f, 1f)] public float damInitialWidthFraction = 0.35f;
        [Range(0.1f, 1f)] public float damInitialHeightFraction = 0.7f;
        [Range(0.25f, 8f)] public float simulationRate = 1f;
        [Range(1, 16)] public int maxSubstepsPerFrame = 8;
        [Range(10, 100)] public int pressureIterations = 40;

        private const float Gravity = 9.80665f;
        private const int Threads = 8;
        private ComputeBuffer liquid, liquidNext;
        private ComputeBuffer u, v, uAdvected, vAdvected, uStar, vStar;
        private ComputeBuffer pressureA, pressureB, fluxX, fluxY;
        private RenderTexture display;
        private int kAdvectU, kAdvectV, kApplyForces, kPressure, kProjectU, kProjectV;
        private int kFluxX, kFluxY, kUpdateLiquid, kRender;
        private bool initialized;
        private bool isPaused;
        private double simulatedTimeS;
        private float maximumU, maximumV, maximumDivergence;
        private float liquidVolumeM3;
        private Case lastCase;

        private int CellCount => gridWidth * gridHeight;
        private int UCount => (gridWidth + 1) * gridHeight;
        private int VCount => gridWidth * (gridHeight + 1);
        private float Dx => domainWidthM / gridWidth;
        private float Dy => domainHeightM / gridHeight;

        private void Start()
        {
            if (!SystemInfo.supportsComputeShaders || fluidSolver == null)
            {
                Debug.LogError("Liquid Flow Validation requires a supported GPU and an assigned compute shader.", this);
                enabled = false;
                return;
            }
            InitializeSolver();
        }

        private void Update()
        {
            if (!initialized) return;
            if (lastCase != validationCase) ResetSolver();
            if (!isPaused) AdvanceFrame(Time.unscaledDeltaTime * simulationRate);
            MeasureDiagnostics();
            RenderLiquid();
        }

        private void InitializeSolver()
        {
            gridWidth = Mathf.Clamp(gridWidth, 8, 256);
            gridHeight = Mathf.Clamp(gridHeight, 8, 256);
            liquid = Buffer(CellCount);
            liquidNext = Buffer(CellCount);
            u = Buffer(UCount);
            v = Buffer(VCount);
            uAdvected = Buffer(UCount);
            vAdvected = Buffer(VCount);
            uStar = Buffer(UCount);
            vStar = Buffer(VCount);
            pressureA = Buffer(CellCount);
            pressureB = Buffer(CellCount);
            fluxX = Buffer((gridWidth + 1) * gridHeight);
            fluxY = Buffer(gridWidth * (gridHeight + 1));

            display = new RenderTexture(gridWidth, gridHeight, 0, RenderTextureFormat.ARGB32)
            {
                enableRandomWrite = true,
                filterMode = FilterMode.Point,
                wrapMode = TextureWrapMode.Clamp
            };
            display.Create();

            kAdvectU = fluidSolver.FindKernel("AdvectU");
            kAdvectV = fluidSolver.FindKernel("AdvectV");
            kApplyForces = fluidSolver.FindKernel("ApplyForces");
            kPressure = fluidSolver.FindKernel("PressureSOR");
            kProjectU = fluidSolver.FindKernel("ProjectU");
            kProjectV = fluidSolver.FindKernel("ProjectV");
            kFluxX = fluidSolver.FindKernel("ComputeFluxX");
            kFluxY = fluidSolver.FindKernel("ComputeFluxY");
            kUpdateLiquid = fluidSolver.FindKernel("UpdateLiquid");
            kRender = fluidSolver.FindKernel("RenderLiquid");

            fluidSolver.SetInt("Width", gridWidth);
            fluidSolver.SetInt("Height", gridHeight);
            fluidSolver.SetFloat("Dx", Dx);
            fluidSolver.SetFloat("Dy", Dy);
            fluidSolver.SetFloat("Density", liquidDensityKgPerM3);
            fluidSolver.SetFloat("Viscosity", kinematicViscosityM2PerS);
            fluidSolver.SetFloat("Gravity", Gravity);
            fluidSolver.SetBuffer(kAdvectU, "LiquidRead", liquid);
            fluidSolver.SetBuffer(kAdvectU, "URead", u);
            fluidSolver.SetBuffer(kAdvectU, "VRead", v);
            fluidSolver.SetBuffer(kAdvectU, "UWrite", uAdvected);
            fluidSolver.SetBuffer(kAdvectV, "LiquidRead", liquid);
            fluidSolver.SetBuffer(kAdvectV, "URead", u);
            fluidSolver.SetBuffer(kAdvectV, "VRead", v);
            fluidSolver.SetBuffer(kAdvectV, "VWrite", vAdvected);
            fluidSolver.SetBuffer(kApplyForces, "LiquidRead", liquid);
            fluidSolver.SetBuffer(kApplyForces, "UAdvected", uAdvected);
            fluidSolver.SetBuffer(kApplyForces, "VAdvected", vAdvected);
            fluidSolver.SetBuffer(kApplyForces, "UStar", uStar);
            fluidSolver.SetBuffer(kApplyForces, "VStar", vStar);
            fluidSolver.SetBuffer(kPressure, "LiquidRead", liquid);
            fluidSolver.SetBuffer(kPressure, "UStar", uStar);
            fluidSolver.SetBuffer(kPressure, "VStar", vStar);
            fluidSolver.SetBuffer(kProjectU, "LiquidRead", liquid);
            fluidSolver.SetBuffer(kProjectU, "UStar", uStar);
            fluidSolver.SetBuffer(kProjectU, "UWrite", u);
            fluidSolver.SetBuffer(kProjectV, "LiquidRead", liquid);
            fluidSolver.SetBuffer(kProjectV, "VStar", vStar);
            fluidSolver.SetBuffer(kProjectV, "VWrite", v);
            fluidSolver.SetBuffer(kFluxX, "LiquidRead", liquid);
            fluidSolver.SetBuffer(kFluxX, "URead", u);
            fluidSolver.SetBuffer(kFluxX, "FluxX", fluxX);
            fluidSolver.SetBuffer(kFluxY, "LiquidRead", liquid);
            fluidSolver.SetBuffer(kFluxY, "VRead", v);
            fluidSolver.SetBuffer(kFluxY, "FluxY", fluxY);
            fluidSolver.SetBuffer(kUpdateLiquid, "LiquidRead", liquid);
            fluidSolver.SetBuffer(kUpdateLiquid, "LiquidWrite", liquidNext);
            fluidSolver.SetBuffer(kUpdateLiquid, "FluxX", fluxX);
            fluidSolver.SetBuffer(kUpdateLiquid, "FluxY", fluxY);
            fluidSolver.SetBuffer(kRender, "LiquidRead", liquid);
            fluidSolver.SetBuffer(kRender, "URead", u);
            fluidSolver.SetBuffer(kRender, "VRead", v);
            fluidSolver.SetTexture(kRender, "Output", display);

            initialized = true;
            ResetSolver();
        }

        private static ComputeBuffer Buffer(int count)
        {
            var buffer = new ComputeBuffer(count, sizeof(float), ComputeBufferType.Structured);
            var zeros = new float[count];
            buffer.SetData(zeros);
            return buffer;
        }

        private void ResetSolver()
        {
            var fractions = new float[CellCount];
            for (int y = 0; y < gridHeight; y++)
                for (int x = 0; x < gridWidth; x++)
                {
                    float xCentre = (x + 0.5f) / gridWidth;
                    float yCentre = (y + 0.5f) / gridHeight;
                    bool occupied = validationCase == Case.HydrostaticRest
                        ? yCentre < initialFillHeightFraction
                        : xCentre < damInitialWidthFraction && yCentre < damInitialHeightFraction;
                    fractions[x + y * gridWidth] = occupied ? 1f : 0f;
                }
            liquid.SetData(fractions);
            liquidNext.SetData(new float[CellCount]);
            u.SetData(new float[UCount]);
            v.SetData(new float[VCount]);
            uAdvected.SetData(new float[UCount]);
            vAdvected.SetData(new float[VCount]);
            uStar.SetData(new float[UCount]);
            vStar.SetData(new float[VCount]);
            pressureA.SetData(new float[CellCount]);
            pressureB.SetData(new float[CellCount]);
            simulatedTimeS = 0;
            maximumU = maximumV = maximumDivergence = 0;
            lastCase = validationCase;
        }

        private void AdvanceFrame(float requestedTimeS)
        {
            double remaining = Mathf.Min(requestedTimeS, 0.1f);
            int steps = 0;
            while (remaining > 1e-7 && steps < maxSubstepsPerFrame)
            {
                float rate = maximumU / Dx + maximumV / Dy;
                float dt = Mathf.Min((float)remaining, 0.0015f);
                if (rate > 1e-5f) dt = Mathf.Min(dt, 0.40f / rate);
                dt = Mathf.Min(dt, Mathf.Sqrt(0.25f * Mathf.Min(Dx, Dy) / Gravity));
                if (dt <= 1e-8f) break;
                AdvanceStep(dt);
                remaining -= dt;
                simulatedTimeS += dt;
                steps++;
            }
        }

        private void AdvanceStep(float dt)
        {
            fluidSolver.SetFloat("Dt", dt);
            Dispatch(kAdvectU, gridWidth + 1, gridHeight);
            Dispatch(kAdvectV, gridWidth, gridHeight + 1);
            Dispatch(kApplyForces, gridWidth + 1, gridHeight + 1);

            ComputeBuffer p = pressureA;
            for (int i = 0; i < pressureIterations; i++)
            {
                // Red/black SOR updates one parity per dispatch. Each dispatch is
                // a GPU-wide synchronization point, so in-place updates are safe.
                fluidSolver.SetBuffer(kPressure, "PressureInOut", p);
                fluidSolver.SetInt("Colour", 0);
                Dispatch(kPressure, gridWidth, gridHeight);
                fluidSolver.SetInt("Colour", 1);
                Dispatch(kPressure, gridWidth, gridHeight);
            }

            fluidSolver.SetBuffer(kProjectU, "PressureRead", p);
            fluidSolver.SetBuffer(kProjectV, "PressureRead", p);
            Dispatch(kProjectU, gridWidth + 1, gridHeight);
            Dispatch(kProjectV, gridWidth, gridHeight + 1);
            Dispatch(kFluxX, gridWidth + 1, gridHeight);
            Dispatch(kFluxY, gridWidth, gridHeight + 1);
            Dispatch(kUpdateLiquid, gridWidth, gridHeight);

            var liquidSwap = liquid; liquid = liquidNext; liquidNext = liquidSwap;
            fluidSolver.SetBuffer(kAdvectU, "LiquidRead", liquid);
            fluidSolver.SetBuffer(kAdvectV, "LiquidRead", liquid);
            fluidSolver.SetBuffer(kApplyForces, "LiquidRead", liquid);
            fluidSolver.SetBuffer(kPressure, "LiquidRead", liquid);
            fluidSolver.SetBuffer(kProjectU, "LiquidRead", liquid);
            fluidSolver.SetBuffer(kProjectV, "LiquidRead", liquid);
            fluidSolver.SetBuffer(kFluxX, "LiquidRead", liquid);
            fluidSolver.SetBuffer(kFluxY, "LiquidRead", liquid);
            fluidSolver.SetBuffer(kUpdateLiquid, "LiquidRead", liquid);
            fluidSolver.SetBuffer(kUpdateLiquid, "LiquidWrite", liquidNext);
            fluidSolver.SetBuffer(kRender, "LiquidRead", liquid);
        }

        private void Dispatch(int kernel, int width, int height) =>
            fluidSolver.Dispatch(kernel, Mathf.CeilToInt(width / (float)Threads),
                Mathf.CeilToInt(height / (float)Threads), 1);

        private void MeasureDiagnostics()
        {
            var liquidData = new float[CellCount];
            var uData = new float[UCount];
            var vData = new float[VCount];
            liquid.GetData(liquidData);
            u.GetData(uData);
            v.GetData(vData);
            maximumU = maximumV = maximumDivergence = liquidVolumeM3 = 0f;
            for (int i = 0; i < uData.Length; i++) maximumU = Mathf.Max(maximumU, Mathf.Abs(uData[i]));
            for (int i = 0; i < vData.Length; i++) maximumV = Mathf.Max(maximumV, Mathf.Abs(vData[i]));
            for (int y = 0; y < gridHeight; y++)
                for (int x = 0; x < gridWidth; x++)
                {
                    int cell = x + y * gridWidth;
                    liquidVolumeM3 += liquidData[cell] * Dx * Dy * sliceDepthM;
                    if (liquidData[cell] <= 0.01f) continue;
                    float div = (uData[x + 1 + y * (gridWidth + 1)] - uData[x + y * (gridWidth + 1)]) / Dx +
                                (vData[x + (y + 1) * gridWidth] - vData[x + y * gridWidth]) / Dy;
                    maximumDivergence = Mathf.Max(maximumDivergence, Mathf.Abs(div));
                }
        }

        private void RenderLiquid()
        {
            fluidSolver.SetBuffer(kRender, "LiquidRead", liquid);
            fluidSolver.SetBuffer(kRender, "URead", u);
            fluidSolver.SetBuffer(kRender, "VRead", v);
            fluidSolver.SetTexture(kRender, "Output", display);
            Dispatch(kRender, gridWidth, gridHeight);
        }

        private void OnGUI()
        {
            float scale = Mathf.Clamp(Screen.height / 720f, 0.65f, 1.35f);
            float pad = 16f * scale;
            float panelWidth = 350f * scale;
            float panelHeight = 320f * scale;
            GUI.color = new Color(0.035f, 0.045f, 0.06f, 0.96f);
            GUI.Box(new Rect(pad, pad, panelWidth, panelHeight), GUIContent.none);
            GUI.color = Color.white;
            GUILayout.BeginArea(new Rect(pad + 12f * scale, pad + 10f * scale,
                panelWidth - 24f * scale, panelHeight - 20f * scale));
            GUI.skin.label.fontSize = Mathf.RoundToInt(18f * scale);
            GUILayout.Label("LIQUID FLOW | STEP 1");
            GUI.skin.label.fontSize = Mathf.RoundToInt(12f * scale);
            GUILayout.Label("GPU MAC grid | incompressible liquid | no phase or heat model");
            GUILayout.Space(8f * scale);
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Hydrostatic rest")) { validationCase = Case.HydrostaticRest; ResetSolver(); }
            if (GUILayout.Button("Dam break")) { validationCase = Case.DamBreak; ResetSolver(); }
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            if (GUILayout.Button(isPaused ? "Resume" : "Pause")) isPaused = !isPaused;
            if (GUILayout.Button("Reset case")) ResetSolver();
            GUILayout.EndHorizontal();
            GUILayout.Space(6f * scale);
            GUILayout.Label(validationCase == Case.HydrostaticRest
                ? "Rest test: the level pool should remain still under gravity."
                : "Dam-break test: the water column spreads under gravity and settles.");
            GUILayout.Label($"Domain: {domainWidthM:0.000} x {domainHeightM:0.000} m; depth {sliceDepthM:0.000} m");
            GUILayout.Label($"Grid: {gridWidth} x {gridHeight}; cell {Dx * 1000f:0.00} x {Dy * 1000f:0.00} mm");
            GUILayout.Label($"Simulated time: {simulatedTimeS:0.000} s");
            GUILayout.Label($"Peak speed: {Mathf.Sqrt(maximumU * maximumU + maximumV * maximumV):0.000} m/s");
            GUILayout.Label($"Maximum divergence: {maximumDivergence:0.000} 1/s");
            GUILayout.Label($"Water volume: {liquidVolumeM3:0.000000} m3");
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
            GUI.Label(new Rect(x, y - 28f * scale, width + 50f, 24f * scale), "2D LIQUID DOMAIN | OPEN TOP");
            DrawSolidRect(vessel, new Color(0.025f, 0.035f, 0.05f, 1f));
            if (display != null)
                GUI.DrawTextureWithTexCoords(vessel, display, new Rect(0f, 1f, 1f, -1f), false);
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

        private void OnDestroy()
        {
            Release(liquid); Release(liquidNext); Release(u); Release(v); Release(uAdvected); Release(vAdvected);
            Release(uStar); Release(vStar); Release(pressureA); Release(pressureB); Release(fluxX); Release(fluxY);
            if (display != null) { display.Release(); Destroy(display); }
        }

        private static void Release(ComputeBuffer buffer)
        {
            if (buffer != null) buffer.Release();
        }
    }
}
