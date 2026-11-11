using System;
using System.Linq;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Verification;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    public sealed class VofSlicePreview : MonoBehaviour
    {
        private const int Size = 28;
        private const double Dt = .025;
        private GeometricVof3D vof;
        private VofGpuGeometry gpu;
        private VofFields state;
        private double[][] transport;
        private Texture2D slice;
        private Color32[] pixels;
        private double initialVolume, elapsed;
        private float nextStep;
        private string error;

        private void OnEnable()
        {
            if (!Application.isPlaying) return;
            try
            {
                var grid = new Grid3D(Size, Size, Size, new D3(1.0 / Size, 1.0 / Size, 1.0 / Size));
                vof = new GeometricVof3D(grid, 997, 1.2);
                gpu = new VofGpuGeometry(vof, Resources.Load<ComputeShader>("ThreePhase3D/VofGeometry"));
                transport = VofChecks.Uniform(vof.CellCount, .28, -.13, 0);
                state = VofChecks.Fields(grid, VofChecks.Sphere(vof, new D3(.38, .59, .5), .19), transport);
                initialVolume = state.LiquidFraction.Sum() * grid.CellVolume;
                elapsed = 0;
                nextStep = Time.unscaledTime;
                error = null;
                pixels = new Color32[Size * Size];
                slice = new Texture2D(Size, Size, TextureFormat.RGBA32, false) { filterMode = FilterMode.Point };
                DrawSlice();
            }
            catch (Exception e) { RecordFailure(e); }
        }

        private void Update()
        {
            if (gpu == null || error != null || Time.unscaledTime < nextStep) return;
            try
            {
                state = gpu.Advance(state, transport, Dt).Fields;
                elapsed += Dt;
                nextStep = Time.unscaledTime + .075f;
                DrawSlice();
            }
            catch (Exception e) { RecordFailure(e); }
        }

        private void DrawSlice()
        {
            var gas = new Color32(13, 24, 42, 255);
            var water = new Color32(42, 205, 241, 255);
            for (int y = 0; y < Size; y++)
            for (int x = 0; x < Size; x++)
                pixels[y * Size + x] = Color32.Lerp(gas, water, (float)state.LiquidFraction[vof.CellIndex(x, y, Size / 2)]);
            slice.SetPixels32(pixels);
            slice.Apply(false);
        }

        private void OnGUI()
        {
            GUI.Label(new Rect(16, 10, Screen.width - 32, 24), "S05  •  Translating droplet  •  XY slice through the 3D periodic grid");
            if (error != null) GUI.Label(new Rect(16, 34, Screen.width - 32, 48), error);
            if (slice == null) return;
            double volume = state.LiquidFraction.Sum() * vof.Grid.CellVolume;
            GUI.Label(new Rect(16, 34, Screen.width - 32, 24),
                $"t = {elapsed:F2} s   |   liquid volume = {volume:F6} m³   |   drift = {volume - initialVolume:E2} m³");
            float size = Mathf.Max(1, Mathf.Min(Screen.width - 32, Screen.height - 78));
            GUI.DrawTexture(new Rect((Screen.width - size) * .5f, 64, size, size), slice);
        }

        private void RecordFailure(Exception exception)
        {
            error = exception.Message;
            Debug.LogException(exception, this);
        }

        private void OnDisable()
        {
            gpu?.Dispose();
            gpu = null;
            if (slice != null) Destroy(slice);
            slice = null;
        }
    }
}
