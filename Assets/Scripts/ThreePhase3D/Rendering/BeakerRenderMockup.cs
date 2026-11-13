using System.Collections.Generic;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    [RequireComponent(typeof(Beaker3DSetup))]
    public sealed class BeakerRenderMockup : MonoBehaviour
    {
        private readonly List<Material> materials = new List<Material>();
        private GameObject stage;

        private void Start()
        {
            var setup = GetComponent<Beaker3DSetup>();
            setup.scenario.display.showSetupReport = false;
            setup.scenario.display.cylinderSegments = 96;
            setup.Refresh();

            stage = new GameObject("Studio tabletop and burner");
            stage.transform.SetParent(transform, false);
            var steel = CreateMaterial(new Color(.18f, .2f, .22f), .7f, .72f);
            var rubber = CreateMaterial(new Color(.035f, .04f, .045f), 0, .27f);
            Part("Graphite tabletop", PrimitiveType.Cube, new Vector3(0, -.015f, 0),
                new Vector3(4, .03f, 4), rubber);
            Part("Burner base", PrimitiveType.Cylinder, new Vector3(0, .007f, 0),
                new Vector3(.052f, .007f, .052f), rubber);
            Part("Burner barrel", PrimitiveType.Cylinder, new Vector3(0, .042f, 0),
                new Vector3(.015f, .034f, .015f), steel);
            Part("Air collar", PrimitiveType.Cylinder, new Vector3(0, .029f, 0),
                new Vector3(.021f, .007f, .021f), steel);

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
            foreach (var material in materials) Destroy(material);
        }
    }
}
