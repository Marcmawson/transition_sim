using System.Collections.Generic;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    [RequireComponent(typeof(Beaker3DSetup))]
    public sealed class BeakerRenderMockup : MonoBehaviour
    {
        private readonly List<Material> materials = new List<Material>();
        private GameObject stage;
        private Mesh detailedGlass;
        private Mesh backdrop;

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
            var brass = CreateMaterial(new Color(.48f, .29f, .11f), .8f, .68f);
            var steel = CreateMaterial(new Color(.18f, .2f, .22f), .7f, .72f);
            var rubber = CreateMaterial(new Color(.014f, .017f, .022f), 0, .19f);
            BuildDaylightStage();
            Part("Burner base", PrimitiveType.Cylinder, new Vector3(0, .007f, 0),
                new Vector3(.052f, .007f, .052f), rubber);
            Part("Burner barrel", PrimitiveType.Cylinder, new Vector3(0, .042f, 0),
                new Vector3(.015f, .034f, .015f), brass);
            Part("Air collar", PrimitiveType.Cylinder, new Vector3(0, .029f, 0),
                new Vector3(.021f, .007f, .021f), steel);

            for (int ring = 0; ring < 5; ring++)
                Part("Collar groove", PrimitiveType.Cylinder, new Vector3(0, .024f + ring * .002f, 0),
                    new Vector3(.022f, .00045f, .022f), steel);
            Part("Burner nozzle", PrimitiveType.Cylinder, new Vector3(0, .075f, 0),
                new Vector3(.018f, .004f, .018f), brass);

            var blue = CreateMaterial(new Color(.025f, .12f, .55f), 0, .2f);
            blue.EnableKeyword("_EMISSION");
            blue.SetColor("_EmissionColor", new Color(.04f, .35f, 1.3f));
            Part("Blue flame core", PrimitiveType.Sphere, new Vector3(0, .088f, 0),
                new Vector3(.007f, .024f, .007f), blue);
            var glow = new GameObject("Burner glow").AddComponent<Light>();
            glow.transform.SetParent(stage.transform, false);
            glow.transform.localPosition = new Vector3(0, .092f, 0);
            glow.type = LightType.Point;
            glow.color = new Color(.2f, .48f, 1);
            glow.range = .12f;
            glow.intensity = .75f;
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
            for (int millilitres = 25; millilitres <= 250; millilitres += 25)
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
            part.GetComponent<Renderer>().sharedMaterial = material;
            Destroy(part.GetComponent<Collider>());
        }

        private void OnDestroy()
        {
            if (stage != null) Destroy(stage);
            if (detailedGlass != null) Destroy(detailedGlass);
            if (backdrop != null) Destroy(backdrop);
            foreach (var material in materials) Destroy(material);
        }
    }
}
