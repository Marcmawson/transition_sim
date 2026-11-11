using System;
using PhysicsSim.ThreePhase3D.Core;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    public sealed class BeakerVofPreview : MonoBehaviour
    {
        private BeakerVofGpuSession session;
        private Texture2D slice;
        private Color32[] pixels;
        private bool running;
        private string error;
        private float nextStep;

        private void OnEnable()
        {
            if (!Application.isPlaying) return;
            try
            {
                session = new BeakerVofGpuSession();
                var grid = session.Simulation.Geometry.Grid;
                slice = new Texture2D(grid.Cells.X,grid.Cells.Z,TextureFormat.RGBA32,false) { filterMode = FilterMode.Point };
                pixels = new Color32[grid.Cells.X*grid.Cells.Z];
                error = null;
                running = true;
                nextStep = Time.unscaledTime;
                DrawSlice();
            }
            catch (Exception e) { Fail(e); }
        }

        private void Update()
        {
            if (!running || error != null || session == null || Time.unscaledTime < nextStep) return;
            Advance();
        }

        private void Advance()
        {
            try
            {
                session.Step();
                DrawSlice();
                nextStep = Time.unscaledTime + .15f;
            }
            catch (Exception e) { Fail(e); }
        }

        private void DrawSlice()
        {
            var sim = session.Simulation;
            var geometry = sim.Geometry;
            var grid = geometry.Grid;
            var air = new Color32(14,24,40,255);
            var liquid = new Color32(30,204,238,255);
            var solid = new Color32(146,167,183,255);
            for (int k = 0; k < grid.Cells.Z; k++)
            for (int i = 0; i < grid.Cells.X; i++)
            {
                int cell = grid.Cells.Index(i,sim.SliceJ,k);
                float glass = (float)(geometry.FixedVolumeM3[cell]/grid.CellVolume);
                var fluid = Color32.Lerp(air,liquid,(float)sim.LiquidFractionAt(i,sim.SliceJ,k));
                pixels[k*grid.Cells.X+i] = Color32.Lerp(fluid,solid,glass);
            }
            slice.SetPixels32(pixels);
            slice.Apply(false);
        }

        private void OnGUI()
        {
            GUI.Label(new Rect(16,10,600,25),"S05 coupled beaker transport");
            if (error != null) GUI.Label(new Rect(16,90,Screen.width-32,48),error);
            if (session == null || slice == null) return;
            var sim = session.Simulation;
            GUI.Label(new Rect(16,36,630,22),$"Step {sim.Steps}   |   time {sim.TimeS:F3} s   |   liquid drift {sim.LiquidDriftM3:E2} m³");
            GUI.Label(new Rect(16,58,630,22),$"Max momentum remap error {sim.MaxMomentumLedgerErrorKgMPerS:E2} kg m/s");
            if (error == null)
            {
                if (GUI.Button(new Rect(16,86,80,26),running ? "Pause" : "Run")) running = !running;
                if (!running && GUI.Button(new Rect(104,86,80,26),"Step")) Advance();
            }
            float size = Mathf.Max(1,Mathf.Min(Screen.width*.34f,Screen.height-150));
            float x = Screen.width-size-16;
            GUI.Label(new Rect(x,104,size,22),"Horizontal slice · cyan liquid · grey glass");
            GUI.DrawTexture(new Rect(x,128,size,size),slice);
        }

        private void Fail(Exception exception)
        {
            error = exception.Message;
            running = false;
            Debug.LogException(exception,this);
        }

        private void OnDisable()
        {
            running = false;
            session?.Dispose();
            session = null;
            if (slice != null) Destroy(slice);
            slice = null;
        }
    }
}
