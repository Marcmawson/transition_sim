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

        private void Start()
        {
            var setup = GetComponent<Beaker3DSetup>();
            setup.scenario.display.showSetupReport = false;
            setup.scenario.display.cylinderSegments = 96;
            setup.Refresh();

            stage = new GameObject("Studio tabletop and burner");
            stage.transform.SetParent(transform, false);
            DetailBeaker(setup);
            var brass = CreateMaterial(new Color(.48f, .29f, .11f), .8f, .68f);
            var steel = CreateMaterial(new Color(.18f, .2f, .22f), .7f, .72f);
            var rubber = CreateMaterial(new Color(.035f, .04f, .045f), 0, .27f);
            Part("Graphite tabletop", PrimitiveType.Cube, new Vector3(0, -.015f, 0),
                new Vector3(4, .03f, 4), rubber);
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

        private void DetailBeaker(Beaker3DSetup setup)
        {
            MeshFilter glass = null;
            foreach (var candidate in setup.GetComponentsInChildren<MeshFilter>())
                if (candidate.gameObject.name == "Glass beaker") glass = candidate;
            if (glass == null) return;

            var geometry = setup.scenario.physical.geometry;
            float rim = (float)geometry.RimHeightM;
            float outer = (float)geometry.OuterRadiusM;
            float floor = (float)(geometry.externalBaseHeightM + geometry.baseThicknessM);
            float lipRadius = (outer - (float)geometry.innerRadiusM) * .65f;
            float lipCentre = outer - lipRadius * .75f;
            detailedGlass = Instantiate(glass.sharedMesh);
            var vertices = new List<Vector3>(detailedGlass.vertices);
            var triangles = new List<int>(detailedGlass.triangles);

            float Spout(float angle) => .012f * Mathf.Pow(Mathf.Max(0, Mathf.Cos(angle + 2.35f)), 48);
            for (int i = 0; i < vertices.Count; i++)
            {
                var p = vertices[i];
                var radial = new Vector2(p.x, p.z);
                if (radial.sqrMagnitude < .00001f) continue;
                float blend = Mathf.SmoothStep(0, 1, Mathf.InverseLerp(rim - .026f, rim, p.y));
                float extension = Spout(Mathf.Atan2(p.z, p.x)) * blend;
                radial += radial.normalized * extension;
                vertices[i] = new Vector3(radial.x, p.y - extension * .18f, radial.y);
            }

            const int segments = 96, sides = 12;
            int first = vertices.Count;
            for (int segment = 0; segment < segments; segment++)
            {
                float angle = 2 * Mathf.PI * segment / segments;
                float extension = Spout(angle);
                for (int side = 0; side < sides; side++)
                {
                    float section = 2 * Mathf.PI * side / sides;
                    float radius = lipCentre + extension + lipRadius * Mathf.Cos(section);
                    vertices.Add(new Vector3(radius * Mathf.Cos(angle),
                        rim - extension * .18f + lipRadius * Mathf.Sin(section), radius * Mathf.Sin(angle)));
                    int a = first + segment * sides + side;
                    int b = first + ((segment + 1) % segments) * sides + side;
                    int d = first + segment * sides + (side + 1) % sides;
                    int e = first + ((segment + 1) % segments) * sides + (side + 1) % sides;
                    triangles.AddRange(new[] { a, d, b, b, d, e });
                }
            }
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
                Mark(major ? "----  " + millilitres : "--", y);
            }
            Mark("mL", floor + .029f);

            void Mark(string value, float y)
            {
                var mark = new GameObject("Graduation " + value).AddComponent<TextMesh>();
                mark.transform.SetParent(stage.transform, false);
                var outward = new Vector3(Mathf.Cos(labelAngle), 0, Mathf.Sin(labelAngle));
                mark.transform.localPosition = outward * (outer + .0005f) + Vector3.up * y;
                mark.transform.localRotation = Quaternion.LookRotation(-outward, Vector3.up);
                mark.text = value;
                mark.fontSize = 64;
                mark.characterSize = .0055f;
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
            foreach (var material in materials) Destroy(material);
        }
    }
}
