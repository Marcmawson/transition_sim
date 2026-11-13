using System.Collections.Generic;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    [RequireComponent(typeof(Beaker3DSetup))]
    public sealed class BeakerRenderMockup : MonoBehaviour
    {
        internal const int MaxMarkedVolumeMl = 250;

        private readonly List<Material> materials = new List<Material>();
        private GameObject stage;
        private Mesh detailedGlass;
        private Mesh backdrop;
        private Mesh cylinder;
        private readonly List<Texture2D> metalNormals = new List<Texture2D>();
        private Transform flameCore, flameSheath;
        private Light burnerGlow;

        private void Start()
        {
            var setup = GetComponent<Beaker3DSetup>();
            var vessel = setup.scenario.physical.geometry;
            vessel.innerRadiusM = .032;
            vessel.wallThicknessM = .0025;
            vessel.baseThicknessM = .005;
            vessel.cavityHeightM = .09;
            vessel.tripodRingRadiusM = .028;
            vessel.tripodTubeRadiusM = .003;
            setup.scenario.display.showSetupReport = false;
            setup.scenario.display.cylinderSegments = 96;
            setup.Refresh();

            stage = new GameObject("Daylight stage");
            stage.transform.SetParent(transform, false);
            DetailBeaker(setup);
            cylinder = BuildCylinder();
            var iron = CreateMaterial(new Color(.19f, .185f, .18f), .8f, .24f);
            var steel = CreateMaterial(new Color(.72f, .74f, .77f), 1f, .82f);
            SetMetalGrain(iron, false);
            SetMetalGrain(steel, true);
            foreach (var renderer in setup.GetComponentsInChildren<MeshRenderer>())
            {
                if (renderer.name == "Tripod contact ring") renderer.sharedMaterial = iron;
                if (!renderer.name.StartsWith("Tripod leg ", System.StringComparison.Ordinal)) continue;
                renderer.sharedMaterial = iron;
                renderer.GetComponent<MeshFilter>().sharedMesh = cylinder;
            }

            BuildDaylightStage();
            Part("Burner base", PrimitiveType.Cylinder, new Vector3(0, .007f, 0),
                new Vector3(.052f, .007f, .052f), steel);
            Part("Burner barrel", PrimitiveType.Cylinder, new Vector3(0, .042f, 0),
                new Vector3(.015f, .034f, .015f), steel);
            Part("Air collar", PrimitiveType.Cylinder, new Vector3(0, .029f, 0),
                new Vector3(.021f, .007f, .021f), steel);
            for (int ring = 0; ring < 5; ring++)
                Part("Collar ridge", PrimitiveType.Cylinder, new Vector3(0, .024f + ring * .002f, 0),
                    new Vector3(.022f, .00045f, .022f), steel);
            Part("Burner nozzle", PrimitiveType.Cylinder, new Vector3(0, .075f, 0),
                new Vector3(.018f, .004f, .018f), steel);
            Part("Nozzle opening", PrimitiveType.Cylinder, new Vector3(0, .0791f, 0),
                new Vector3(.013f, .00015f, .013f), iron);

            flameSheath = Flame("Blue flame envelope", new Color(.04f, .24f, 1f, .32f), .006f, .034f);
            flameCore = Flame("Cyan inner cone", new Color(.16f, .64f, 1f, .78f), .0036f, .019f);
            burnerGlow = new GameObject("Burner glow").AddComponent<Light>();
            burnerGlow.transform.SetParent(stage.transform, false);
            burnerGlow.transform.localPosition = new Vector3(0, .092f, 0);
            burnerGlow.type = LightType.Point;
            burnerGlow.color = new Color(.2f, .48f, 1);
            burnerGlow.range = .12f;

            var probe = new GameObject("Burner reflection probe").AddComponent<ReflectionProbe>();
            probe.transform.SetParent(stage.transform, false);
            probe.transform.localPosition = new Vector3(0, .14f, 0);
            probe.mode = UnityEngine.Rendering.ReflectionProbeMode.Realtime;
            probe.refreshMode = UnityEngine.Rendering.ReflectionProbeRefreshMode.ViaScripting;
            probe.resolution = 128;
            probe.size = new Vector3(2, 2, 2);
            probe.RenderProbe();
        }

        private void Update()
        {
            if (flameCore == null) return;
            float phase = Mathf.Repeat(Time.time, 2f) * Mathf.PI;
            float flicker = .65f * Mathf.Sin(phase * 5) + .35f * Mathf.Sin(phase * 11 + .6f);
            flameSheath.localScale = new Vector3(1 + .09f * flicker, 1 + .13f * flicker, 1 + .09f * flicker);
            flameSheath.localRotation = Quaternion.Euler(2 * Mathf.Sin(phase * 3), 0, 2 * Mathf.Cos(phase * 4));
            flameCore.localScale = new Vector3(1, 1 + .06f * flicker, 1);
            burnerGlow.intensity = .75f + .1f * flicker;
        }

        private Transform Flame(string label, Color colour, float radius, float height)
        {
            var material = new Material(Shader.Find("Particles/Standard Unlit"));
            materials.Add(material);
            material.SetColor("_Color", colour);
            material.SetColor("_EmissionColor", new Color(colour.r, colour.g, colour.b) * .6f);
            material.EnableKeyword("_EMISSION");
            material.SetFloat("_Mode", 2);
            material.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
            material.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.One);
            material.SetInt("_ZWrite", 0);
            material.renderQueue = 3000;
            var profile = new[]
            {
                new Vector2(radius * .75f, 0), new Vector2(radius, height * .18f),
                new Vector2(radius * .78f, height * .5f), new Vector2(radius * .42f, height * .78f),
                new Vector2(radius * .08f, height * .97f), new Vector2(0, height)
            };
            var flame = new GameObject(label);
            flame.transform.SetParent(stage.transform, false);
            flame.transform.localPosition = Vector3.up * .0794f;
            var mesh = Revolve(profile, 48, label);
            flameMeshes.Add(mesh);
            flame.AddComponent<MeshFilter>().sharedMesh = mesh;
            var renderer = flame.AddComponent<MeshRenderer>();
            renderer.sharedMaterial = material;
            renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            return flame.transform;
        }

        private readonly List<Mesh> flameMeshes = new List<Mesh>();

        private static Mesh BuildCylinder() => Revolve(new[]
        {
            new Vector2(0, -1), new Vector2(.48f, -1), new Vector2(.5f, -.98f),
            new Vector2(.5f, .98f), new Vector2(.48f, 1), new Vector2(0, 1)
        }, 96, "Bevelled metal cylinder");

        private static Mesh Revolve(Vector2[] profile, int segments, string label)
        {
            var vertices = new List<Vector3>();
            var uv = new List<Vector2>();
            var triangles = new List<int>();
            int stride = segments + 1;
            for (int row = 0; row < profile.Length; row++)
            for (int side = 0; side <= segments; side++)
            {
                float angle = 2 * Mathf.PI * (side % segments) / segments;
                vertices.Add(new Vector3(profile[row].x * Mathf.Cos(angle), profile[row].y,
                    profile[row].x * Mathf.Sin(angle)));
                uv.Add(new Vector2((float)side / segments, (float)row / (profile.Length - 1)));
                if (row == 0 || side == segments) continue;
                int a = (row - 1) * stride + side;
                int b = a + stride;
                if (profile[row].x > 0) triangles.AddRange(new[] { a, b, b + 1 });
                if (profile[row - 1].x > 0) triangles.AddRange(new[] { a, b + 1, a + 1 });
            }
            var mesh = new Mesh { name = label };
            mesh.SetVertices(vertices);
            mesh.SetUVs(0, uv);
            mesh.SetTriangles(triangles, 0);
            mesh.RecalculateNormals();
            var normals = mesh.normals;
            for (int row = 0; row < profile.Length; row++)
            {
                int start = row * stride;
                if (profile[row].x == 0)
                {
                    Vector3 pole = row == 0 ? Vector3.down : Vector3.up;
                    for (int side = 0; side <= segments; side++) normals[start + side] = pole;
                }
                else normals[start] = normals[start + segments] = (normals[start] + normals[start + segments]).normalized;
            }
            mesh.normals = normals;
            mesh.RecalculateTangents();
            mesh.RecalculateBounds();
            return mesh;
        }

        private void SetMetalGrain(Material material, bool brushed)
        {
            const int size = 256;
            var texture = new Texture2D(size, size, TextureFormat.RGBA32, true, true)
                { name = brushed ? "Brushed stainless grain" : "Cast iron grain", wrapMode = TextureWrapMode.Repeat };
            var pixels = new Color32[size * size];
            for (int y = 0; y < size; y++)
            for (int x = 0; x < size; x++)
            {
                float dx = Height(x + 1, y) - Height(x - 1, y);
                float dy = Height(x, y + 1) - Height(x, y - 1);
                Vector3 n = new Vector3(-dx, -dy, 1).normalized;
                pixels[y * size + x] = new Color(.5f + n.x * .5f, .5f + n.y * .5f,
                    .5f + n.z * .5f, 1);
            }
            texture.SetPixels32(pixels);
            texture.Apply(true, true);
            metalNormals.Add(texture);
            material.SetTexture("_BumpMap", texture);
            material.SetTextureScale("_BumpMap", brushed ? new Vector2(1, 3) : new Vector2(2, 4));
            material.EnableKeyword("_NORMALMAP");

            float Height(int x, int y)
            {
                float u = (x & 255) / 256f, v = (y & 255) / 256f;
                if (brushed) return .085f * Mathf.Sin(2 * Mathf.PI * (73 * v + .13f * Mathf.Sin(2 * Mathf.PI * u)));
                return .46f * Mathf.PerlinNoise(u * 45, v * 45) + .27f * Mathf.PerlinNoise(u * 110 + 7, v * 110);
            }
        }

        private void BuildDaylightStage()
        {
            var surface = CreateMaterial(new Color(.35f, .41f, .39f), 0, .2f);
            // The worktop bends into the rear wall, so the camera sees a
            // continuous surface through the glass and around the tripod.
            const int segments = 24;
            const float radius = .36f;
            var profile = new List<Vector2> { new Vector2(-4, 0), new Vector2(.55f, 0) };
            for (int row = 1; row <= segments; row++)
            {
                float angle = .5f * Mathf.PI * row / segments;
                profile.Add(new Vector2(.55f + radius * Mathf.Sin(angle), radius * (1 - Mathf.Cos(angle))));
            }
            profile.Add(new Vector2(.55f + radius, 3));

            var vertices = new List<Vector3>();
            var triangles = new List<int>();
            for (int row = 0; row < profile.Count; row++)
            {
                Vector2 point = profile[row];
                vertices.Add(new Vector3(-4, point.y, point.x));
                vertices.Add(new Vector3(4, point.y, point.x));
                if (row == 0) continue;
                int a = 2 * (row - 1);
                triangles.AddRange(new[] { a, a + 2, a + 1, a + 1, a + 2, a + 3 });
            }

            backdrop = new Mesh { name = "Curved worktop and backdrop" };
            backdrop.SetVertices(vertices);
            backdrop.SetTriangles(triangles, 0);
            backdrop.RecalculateNormals();
            backdrop.RecalculateBounds();
            var sweep = new GameObject("Sage backdrop");
            sweep.transform.SetParent(stage.transform, false);
            sweep.AddComponent<MeshFilter>().sharedMesh = backdrop;
            sweep.AddComponent<MeshRenderer>().sharedMaterial = surface;

            var sun = new GameObject("Window sunlight").AddComponent<Light>();
            sun.transform.SetParent(stage.transform, false);
            sun.transform.localRotation = Quaternion.LookRotation(new Vector3(.65f, -.55f, .75f));
            sun.type = LightType.Directional;
            sun.color = new Color(1, .92f, .79f);
            sun.intensity = 1.15f;
            sun.shadows = LightShadows.Soft;
            sun.shadowStrength = .75f;
        }

        private void DetailBeaker(Beaker3DSetup setup)
        {
            MeshFilter glass = null;
            foreach (var candidate in setup.GetComponentsInChildren<MeshFilter>())
                if (candidate.gameObject.name == "Glass beaker") glass = candidate;
            if (glass == null) return;
            glass.GetComponent<MeshRenderer>().sharedMaterial.SetFloat("_StudioReflections", 1);

            var geometry = setup.scenario.physical.geometry;
            float rim = (float)geometry.RimHeightM;
            float outer = (float)geometry.OuterRadiusM;
            float floor = (float)(geometry.externalBaseHeightM + geometry.baseThicknessM);
            // Revolve the entire glass cross-section from the underside to the
            // inside floor. The rolled lip is part of this same surface.
            float bottom = (float)geometry.externalBaseHeightM;
            float inner = (float)geometry.innerRadiusM;
            var profile = new List<Vector2>
            {
                new Vector2(0, bottom),
                new Vector2(outer - .004f, bottom),
                new Vector2(outer - .0017f, bottom + .0006f),
                new Vector2(outer - .0005f, bottom + .0018f),
                new Vector2(outer, bottom + .004f),
                new Vector2(outer + .001f, floor + .02f),
                new Vector2(outer + .002f, rim - .012f)
            };
            float lipCentre = (outer + inner) * .5f + .002f;
            float lipRadius = (outer - inner) * .5f + .00055f;
            for (int step = 0; step <= 12; step++)
            {
                float angle = Mathf.PI * step / 12;
                profile.Add(new Vector2(lipCentre + lipRadius * Mathf.Cos(angle),
                    rim - .0018f + lipRadius * Mathf.Sin(angle)));
            }
            profile.AddRange(new[]
            {
                new Vector2(inner + .002f, rim - .012f),
                new Vector2(inner + .001f, floor + .02f),
                new Vector2(inner, floor + .004f),
                new Vector2(inner - .0007f, floor + .0018f),
                new Vector2(inner - .002f, floor + .0005f),
                new Vector2(inner - .004f, floor),
                new Vector2(0, floor)
            });

            const int segments = 144;
            var vertices = new List<Vector3>();
            var triangles = new List<int>();
            var rings = new int[profile.Count];
            for (int row = 0; row < profile.Count; row++)
            {
                var point = profile[row];
                rings[row] = vertices.Count;
                if (point.x == 0)
                {
                    vertices.Add(new Vector3(0, point.y, 0));
                    continue;
                }

                float blend = Mathf.SmoothStep(0, 1, Mathf.InverseLerp(rim - .024f, rim - .002f, point.y));
                for (int segment = 0; segment < segments; segment++)
                {
                    float angle = 2 * Mathf.PI * segment / segments;
                    float spout = .012f * blend * Mathf.Pow(Mathf.Max(0, Mathf.Cos(angle + 2.35f)), 28);
                    float radius = point.x + spout;
                    vertices.Add(new Vector3(radius * Mathf.Cos(angle), point.y - spout * .14f,
                        radius * Mathf.Sin(angle)));
                }
            }

            for (int row = 0; row < profile.Count - 1; row++)
            for (int segment = 0; segment < segments; segment++)
            {
                int next = (segment + 1) % segments;
                bool lowerPole = profile[row].x == 0;
                bool upperPole = profile[row + 1].x == 0;
                int a = rings[row] + (lowerPole ? 0 : segment);
                int b = rings[row] + (lowerPole ? 0 : next);
                int d = rings[row + 1] + (upperPole ? 0 : segment);
                int e = rings[row + 1] + (upperPole ? 0 : next);
                if (!upperPole) triangles.AddRange(new[] { a, d, e });
                if (!lowerPole) triangles.AddRange(new[] { a, e, b });
            }

            detailedGlass = new Mesh { name = "Moulded Griffin beaker glass" };
            detailedGlass.SetVertices(vertices);
            detailedGlass.SetTriangles(triangles, 0);
            detailedGlass.RecalculateNormals();
            detailedGlass.RecalculateBounds();
            glass.sharedMesh = detailedGlass;

            float labelAngle = -.85f;
            for (int millilitres = 25; millilitres <= MaxMarkedVolumeMl; millilitres += 25)
            {
                float y = floor + millilitres * 1e-6f / (Mathf.PI * Mathf.Pow((float)geometry.innerRadiusM, 2));
                bool major = millilitres % 50 == 0;
                Mark(major ? "--- " + millilitres : "-", y);
            }
            Mark("mL", floor + .082f);

            void Mark(string value, float y)
            {
                var mark = new GameObject("Graduation " + value).AddComponent<TextMesh>();
                mark.transform.SetParent(stage.transform, false);
                var outward = new Vector3(Mathf.Cos(labelAngle), 0, Mathf.Sin(labelAngle));
                mark.transform.localPosition = outward * (outer + .0026f) + Vector3.up * y;
                mark.transform.localRotation = Quaternion.LookRotation(-outward, Vector3.up);
                mark.text = value;
                mark.fontSize = 64;
                mark.characterSize = .00085f;
                mark.anchor = TextAnchor.MiddleCenter;
                mark.color = new Color(.94f, .96f, .97f, .85f);
            }
        }

        private Material CreateMaterial(Color colour, float metallic, float smoothness)
        {
            var material = new Material(Shader.Find("Standard")) { color = colour };
            material.SetFloat("_Metallic", metallic);
            material.SetFloat("_Glossiness", smoothness);
            materials.Add(material);
            return material;
        }

        private void Part(string label, PrimitiveType type, Vector3 position, Vector3 size, Material material)
        {
            var part = GameObject.CreatePrimitive(type);
            part.name = label;
            part.transform.SetParent(stage.transform, false);
            part.transform.localPosition = position;
            part.transform.localScale = size;
            if (type == PrimitiveType.Cylinder) part.GetComponent<MeshFilter>().sharedMesh = cylinder;
            part.GetComponent<Renderer>().sharedMaterial = material;
            Destroy(part.GetComponent<Collider>());
        }

        private void OnDestroy()
        {
            if (stage != null) Destroy(stage);
            if (detailedGlass != null) Destroy(detailedGlass);
            if (backdrop != null) Destroy(backdrop);
            if (cylinder != null) Destroy(cylinder);
            foreach (var mesh in flameMeshes) Destroy(mesh);
            foreach (var texture in metalNormals) Destroy(texture);
            foreach (var material in materials) Destroy(material);
        }
    }
}
