using PhaseTransition.Physics;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class CreatePhaseChangeDemo
{
    private const string ScenePath = "Assets/Scenes/PhaseChange Beaker.unity";

    [MenuItem("Physics Sim/Create Phase Change Beaker Scene")]
    public static void CreateScene()
    {
        if (AssetDatabase.LoadAssetAtPath<SceneAsset>(ScenePath) != null && !EditorUtility.DisplayDialog(
                "Replace demo scene?", "The phase-change beaker scene already exists. Replace it?", "Replace", "Cancel"))
            return;

        var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        var cameraObject = new GameObject("Main Camera");
        cameraObject.tag = "MainCamera";
        var camera = cameraObject.AddComponent<Camera>();
        camera.orthographic = true;
        camera.orthographicSize = 5f;
        camera.transform.position = new Vector3(0f, 0f, -10f);
        camera.clearFlags = CameraClearFlags.SolidColor;
        camera.backgroundColor = new Color(0.045f, 0.055f, 0.075f);

        var demo = new GameObject("Phase Change Beaker Demo");
        demo.AddComponent<BeakerEnergyLedger>();
        demo.AddComponent<PhaseChangeDemoGUI>();

        EditorSceneManager.SaveScene(scene, ScenePath);
        AssetDatabase.Refresh();
        EditorUtility.DisplayDialog("Scene created", "Saved and opened:\n" + ScenePath, "OK");
    }
}
