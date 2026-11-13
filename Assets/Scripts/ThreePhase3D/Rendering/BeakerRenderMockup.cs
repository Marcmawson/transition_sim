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
        private Mesh cylinder;
        private readonly List<Texture2D> metalNormals = new List<Texture2D>();
        private Transform flameCore, flameSheath;
        private Light burnerGlow;
        private readonly List<Mesh> burnerMeshes = new List<Mesh>();

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
            var rayShader = Resources.Load<Shader>("ThreePhase3D/BurnerRayReflections");
            var steel = CreateMaterial(new Color(.72f, .74f, .77f), 1f, .82f,
                rayShader != null && rayShader.isSupported ? rayShader : null);
            SetMetalGrain(iron, false);
            SetMetalGrain(steel, true);
            foreach (var renderer in setup.GetComponentsInChildren<MeshRenderer>())
            {
                if (renderer.name == "Tripod contact ring") renderer.sharedMaterial = iron;
                if (!renderer.name.StartsWith("Tripod leg ", System.StringComparison.Ordinal)) continue;
                renderer.sharedMaterial = iron;
                renderer.GetComponent<MeshFilter>().sharedMesh = cylinder;
            }

            BuildDaylightStage(steel);
            BuildBurner(steel, iron);

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

        private void BuildBurner(Material steel, Material iron)
        {
            MeshPart("Stepped stainless base", steel, new[]
            {
                new Vector2(0, .002f), new Vector2(.024f, .002f), new Vector2(.026f, .0035f),
                new Vector2(.026f, .0075f), new Vector2(.025f, .009f), new Vector2(.021f, .010f),
                new Vector2(.0205f, .0114f), new Vector2(.012f, .013f), new Vector2(.010f, .016f),
                new Vector2(0, .016f)
            });
            MeshPart("Burner tube and hollow rolled nozzle", steel, new[]
            {
                new Vector2(0, .014f), new Vector2(.0077f, .014f), new Vector2(.0084f, .017f),
                new Vector2(.0074f, .023f), new Vector2(.0074f, .0695f), new Vector2(.0083f, .072f),
                new Vector2(.0092f, .074f), new Vector2(.0092f, .0778f),
                new Vector2(.0088f, .0792f), new Vector2(.0072f, .0794f),
                new Vector2(.0064f, .0785f), new Vector2(.0064f, .067f)
            });
            MeshPart("Recessed nozzle interior", iron, new[]
            {
                new Vector2(.0064f, .067f), new Vector2(.0064f, .066f), new Vector2(0, .066f)
            });

            var collar = new List<Vector2> { new Vector2(.0074f, .022f), new Vector2(.0096f, .022f) };
            for (int ridge = 0; ridge < 6; ridge++)
            {
                float y = .023f + ridge * .00175f;
                collar.Add(new Vector2(.0096f, y));
                collar.Add(new Vector2(.0108f, y + .00035f));
                collar.Add(new Vector2(.0108f, y + .0008f));
                collar.Add(new Vector2(.0096f, y + .00115f));
            }
            collar.Add(new Vector2(.0093f, .034f));
            collar.Add(new Vector2(.0074f, .034f));
            MeshPart("Ribbed air adjustment collar", steel, collar.ToArray());
        }

        private void MeshPart(string label, Material material, Vector2[] profile)
        {
            var part = new GameObject(label);
            part.transform.SetParent(stage.transform, false);
            var mesh = Revolve(profile, 128, label);
            burnerMeshes.Add(mesh);
            part.AddComponent<MeshFilter>().sharedMesh = mesh;
            part.AddComponent<MeshRenderer>().sharedMaterial = material;
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

        private void BuildDaylightStage(Material steel)
        {
            const float groundY = -.84f;
            var benchSize = new Vector3(1.28f, .055f, .66f);
            var benchCentre = new Vector3(0, -benchSize.y * .5f, 0);
            var timber = CreateMaterial(new Color(.47f, .29f, .15f), 0, .28f);
            var frame = CreateMaterial(new Color(.11f, .13f, .12f), .65f, .35f);
            var paving = CreateMaterial(new Color(.56f, .55f, .51f), 0, .18f);
            Box("Timber bench top", benchCentre, benchSize, timber);
            for (int x = -1; x <= 1; x += 2)
            for (int z = -1; z <= 1; z += 2)
                Box("Bench leg", new Vector3(x * .55f, (groundY - benchSize.y) * .5f, z * .26f),
                    new Vector3(.055f, -groundY - benchSize.y, .055f), frame);
            for (int z = -1; z <= 1; z += 2)
                Box("Bench crossbar", new Vector3(0, -.105f, z * .26f), new Vector3(1.1f, .075f, .035f), frame);
            Box("Outdoor concrete floor", new Vector3(0, groundY - .06f, 0), new Vector3(32, .12f, 32), paving);

            if (steel.HasProperty("_GroundY"))
            {
                steel.SetFloat("_GroundY", groundY);
                steel.SetVector("_BenchMin", benchCentre - benchSize * .5f);
                steel.SetVector("_BenchMax", benchCentre + benchSize * .5f);
                steel.SetColor("_BenchColour", timber.color);
                steel.SetColor("_GroundColour", paving.color);
            }

            var sun = new GameObject("Direct sunlight").AddComponent<Light>();
            sun.transform.SetParent(stage.transform, false);
            sun.transform.localRotation = Quaternion.LookRotation(new Vector3(.45f, -.78f, .43f));
            sun.type = LightType.Directional;
            sun.color = new Color(1, .95f, .85f);
            sun.intensity = 1.25f;
            sun.shadows = LightShadows.Hard;
            sun.shadowStrength = .9f;

            var skyShader = Shader.Find("Skybox/Procedural");
            if (skyShader != null)
            {
                var sky = new Material(skyShader);
                materials.Add(sky);
                var camera = Camera.main;
                if (camera != null)
                {
                    camera.clearFlags = CameraClearFlags.Skybox;
                    camera.gameObject.AddComponent<Skybox>().material = sky;
                }
                RenderSettings.sun = sun;
            }
        }

        private void Box(string label, Vector3 centre, Vector3 size, Material material)
        {
            var box = GameObject.CreatePrimitive(PrimitiveType.Cube);
            box.name = label;
            box.transform.SetParent(stage.transform, false);
            box.transform.localPosition = centre;
            box.transform.localScale = size;
            box.GetComponent<MeshRenderer>().sharedMaterial = material;
            Destroy(box.GetComponent<Collider>());
        }

        private void DetailBeaker(Beaker3DSetup setup)
        {
            MeshFilter glass = null;
            foreach (var candidate in setup.GetComponentsInChildren<MeshFilter>())
                if (candidate.gameObject.name == "Glass beaker") glass = candidate;
            if (glass == null) return;
            glass.GetComponent<MeshRenderer>().sharedMaterial.SetFloat("_StudioReflections", 0);

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

        private Material CreateMaterial(Color colour, float metallic, float smoothness, Shader shader = null)
        {
            var material = new Material(shader ?? Shader.Find("Standard")) { color = colour };
            material.SetFloat("_Metallic", metallic);
            material.SetFloat("_Glossiness", smoothness);
            materials.Add(material);
            return material;
        }

        private void OnDestroy()
        {
            if (stage != null) Destroy(stage);
            if (detailedGlass != null) Destroy(detailedGlass);
            if (cylinder != null) Destroy(cylinder);
            foreach (var mesh in flameMeshes) Destroy(mesh);
            foreach (var mesh in burnerMeshes) Destroy(mesh);
            foreach (var texture in metalNormals) Destroy(texture);
            foreach (var material in materials) Destroy(material);
        }
    }
}
