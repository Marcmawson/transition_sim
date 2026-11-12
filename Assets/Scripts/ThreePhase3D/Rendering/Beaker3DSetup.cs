using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Geometry;
using UnityEngine;
using UnityEngine.Rendering;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    [ExecuteAlways, DisallowMultipleComponent]
    public sealed class Beaker3DSetup : MonoBehaviour
    {
        public ScenarioConfigSI scenario = new ScenarioConfigSI();
        [SerializeField] bool useDielectricGlassPreview;
        readonly List<Object> ownedResources = new List<Object>();
        GameObject generated;
        string previewHash;
        ConfigurationReport report;
        public string HardwareDescription => SystemInfo.graphicsDeviceName + " | " + SystemInfo.graphicsDeviceVersion +
            " | VRAM reported ~" + SystemInfo.graphicsMemorySize + " MB | " + SystemInfo.graphicsDeviceType +
            " | compute=" + SystemInfo.supportsComputeShaders + " | async readback=" + SystemInfo.supportsAsyncGPUReadback;

        void OnEnable() => Refresh();
        void Update()
        {
            if (scenario == null) return;
            string hash = PreviewHash();
            if (hash != previewHash) Refresh();
        }
        void OnDisable() => Release();

        public void Refresh()
        {
            Release();
            if (scenario == null) { report = new ConfigurationReport(); report.Errors.Add("Scenario is missing."); return; }
            report = scenario.Validate();
            previewHash = PreviewHash();
            if (transform.position != Vector3.zero || transform.rotation != Quaternion.identity || transform.lossyScale != Vector3.one)
                report.Errors.Add("Apparatus transform must be identity: one Unity unit is one metre.");
            if (!report.CanPreview) return;
            generated = new GameObject("Generated apparatus (SI preview)") { hideFlags = HideFlags.HideAndDontSave };
            generated.transform.SetParent(transform, false);
            var glass = useDielectricGlassPreview ? NewDielectricGlass() :
                NewMaterial("Glass preview (optics unvalidated)", new Color(0.55f, 0.82f, 0.92f, scenario.display.glassOpacity), true);
            var metal = NewMaterial("Tripod preview", new Color(0.2f, 0.23f, 0.27f, 1), false);
            var g = scenario.physical.geometry;
            AddSurface("Glass beaker", ApparatusSurface.Beaker(g, scenario.display.cylinderSegments), glass);
            AddSurface("Tripod contact ring", ApparatusSurface.SupportRing(g, scenario.display.cylinderSegments), metal);
            for (int i = 0; i < 3; i++)
            {
                float angle = (float)(2 * System.Math.PI * i / 3);
                Vector3 direction = new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle));
                Vector3 top = direction * (float)g.tripodRingRadiusM + Vector3.up * (float)(g.externalBaseHeightM - 2 * g.tripodTubeRadiusM);
                Vector3 bottom = direction * (float)g.tripodFootRadiusM + Vector3.up * (float)g.tripodTubeRadiusM;
                GameObject leg = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                leg.name = "Tripod leg " + (i + 1); leg.hideFlags = HideFlags.HideAndDontSave;
                leg.transform.SetParent(generated.transform, false);
                leg.transform.localPosition = (top + bottom) / 2;
                leg.transform.localRotation = Quaternion.FromToRotation(Vector3.up, top - bottom);
                leg.transform.localScale = new Vector3((float)(2 * g.tripodTubeRadiusM), (top - bottom).magnitude / 2, (float)(2 * g.tripodTubeRadiusM));
                leg.GetComponent<Renderer>().sharedMaterial = metal;
            }
        }

        string PreviewHash() => RecordJson.Hash(RecordJson.Encode(scenario)) + (useDielectricGlassPreview ? ":dielectric" : ":tinted");

        Material NewDielectricGlass()
        {
            var shader = Resources.Load<Shader>("ThreePhase3D/BeakerDielectric");
            if (shader == null) throw new System.InvalidOperationException("Beaker dielectric shader unavailable.");
            const float duranIndex = 1.473f; // MAT015, 587.6 nm, air approximated as n = 1.
            float amplitude = (duranIndex - 1) / (duranIndex + 1);
            var material = new Material(shader) { name = "DURAN dielectric preview", hideFlags = HideFlags.HideAndDontSave };
            material.SetFloat("_NormalReflectance", amplitude * amplitude);
            ownedResources.Add(material);
            return material;
        }

        Material NewMaterial(string label, Color colour, bool transparent)
        {
            Shader shader = Shader.Find("Standard");
            if (shader == null) throw new System.InvalidOperationException("Built-in Standard shader unavailable.");
            var material = new Material(shader) { name = label, hideFlags = HideFlags.HideAndDontSave, color = colour };
            material.SetFloat("_Metallic", transparent ? 0 : 0.75f); material.SetFloat("_Glossiness", 0.7f);
            if (transparent)
            {
                material.SetFloat("_Mode", 3); material.SetInt("_SrcBlend", (int)BlendMode.One);
                material.SetInt("_DstBlend", (int)BlendMode.OneMinusSrcAlpha); material.SetInt("_ZWrite", 0);
                material.EnableKeyword("_ALPHAPREMULTIPLY_ON"); material.renderQueue = 3000;
            }
            ownedResources.Add(material); return material;
        }
        void AddSurface(string label, TriangleSurface surface, Material material)
        {
            var vertices = new Vector3[surface.positionsM.Length / 3];
            for (int i = 0; i < vertices.Length; i++) vertices[i] = new Vector3((float)surface.positionsM[3 * i], (float)surface.positionsM[3 * i + 1], (float)surface.positionsM[3 * i + 2]);
            var mesh = new Mesh { name = label + " surface", hideFlags = HideFlags.HideAndDontSave };
            mesh.vertices = vertices; mesh.triangles = surface.triangles; mesh.RecalculateNormals(); mesh.RecalculateBounds(); ownedResources.Add(mesh);
            var part = new GameObject(label) { hideFlags = HideFlags.HideAndDontSave }; part.transform.SetParent(generated.transform, false);
            part.AddComponent<MeshFilter>().sharedMesh = mesh; part.AddComponent<MeshRenderer>().sharedMaterial = material;
            part.AddComponent<MeshCollider>().sharedMesh = mesh;
        }
        void Release()
        {
            if (generated != null) DestroyOwned(generated);
            generated = null;
            foreach (var resource in ownedResources) if (resource != null) DestroyOwned(resource);
            ownedResources.Clear();
        }
        static void DestroyOwned(Object value)
        { if (Application.isPlaying) Destroy(value); else DestroyImmediate(value); }

        void OnGUI()
        {
            if (!Application.isPlaying || scenario == null || !scenario.display.showSetupReport) return;
            GUILayout.BeginArea(new Rect(12, 12, 440, Screen.height - 24), GUI.skin.box);
            GUILayout.Label("THREE PHASE 3D | S00 APPARATUS PREVIEW");
            GUILayout.Label("1 Unity unit = 1 m | y up | empty beaker");
            GUILayout.Label("Fluid/thermal/phase solvers await stages S02 onwards.");
            GUILayout.Space(8);
            GUILayout.Label(HardwareDescription);
            GUILayout.Label("Unity " + Application.unityVersion);
            if (report != null)
            {
                foreach (var error in report.Errors) GUILayout.Label("ERROR: " + error);
                GUILayout.Label("Missing data (physics run blocked):");
                foreach (var missing in report.MissingData) GUILayout.Label(missing);
            }
            GUILayout.Label("Run checks: Physics Sim > Three Phase 3D > Run foundation checks");
            GUILayout.EndArea();
        }
    }
}
