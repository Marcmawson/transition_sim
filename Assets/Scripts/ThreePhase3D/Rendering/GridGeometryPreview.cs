using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Geometry;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    [ExecuteAlways,DisallowMultipleComponent]
    public sealed class GridGeometryPreview:MonoBehaviour
    {
        [SerializeField] private Beaker3DSetup setup;
        private CutCellGeometry geometry;private string fingerprint;private Mesh iceMesh;private Material iceMaterial;private GameObject iceObject;
        public void Configure(Beaker3DSetup source){setup=source;Rebuild();}
        void OnEnable(){if(setup!=null)Rebuild();}
        [ContextMenu("Rebuild S03 geometry from current inputs")]
        public void Rebuild()
        {
            if(setup==null)return;var validation=setup.scenario.Validate();if(!validation.CanPreview)throw new System.ArgumentException(validation.ToString());var g=setup.scenario.physical.geometry;
            if(g.OuterRadiusM>=.09||g.externalBaseHeightM<.10||g.RimHeightM>.32)throw new System.NotSupportedException("Apparatus leaves the declared S03 preview grid; use a configured geometry test instead.");
            var grid=new Grid3D(16,20,16,new D3(.18/16,.22/20,.18/16),new RigidFrame(new D3(-.09,.10,-.09),new D3(1,0,0),new D3(0,1,0),new D3(0,0,1)));
            var ice=SurfacePrimitives.Box(new D3(-.018,g.externalBaseHeightM+g.baseThicknessM,-.018),new D3(.018,g.externalBaseHeightM+g.baseThicknessM+.036,.018));
            var next=CutCellGeometry.Build(grid,ApparatusSurface.Beaker(g,setup.scenario.display.cylinderSegments),ice);Initialize(next,ice,setup);
        }
        public void Initialize(CutCellGeometry metrics,TriangleSurface ice,Beaker3DSetup apparatus)
        {
            Release(iceObject);Release(iceMesh);Release(iceMaterial);
            geometry=metrics;setup=apparatus;fingerprint=RecordJson.Hash(RecordJson.Encode(setup.scenario));
            var vertices=new Vector3[ice.positionsM.Length/3];for(int i=0;i<vertices.Length;i++)vertices[i]=new Vector3((float)ice.positionsM[3*i],(float)ice.positionsM[3*i+1],(float)ice.positionsM[3*i+2]);
            iceMesh=new Mesh{name="S03 authoritative ice surface",hideFlags=HideFlags.HideAndDontSave};iceMesh.vertices=vertices;iceMesh.triangles=ice.triangles;iceMesh.RecalculateNormals();iceMesh.RecalculateBounds();
            Shader shader=Shader.Find("Standard");if(shader==null)throw new System.InvalidOperationException("Built-in Standard shader unavailable.");iceMaterial=new Material(shader){color=new Color(.5f,.82f,.94f),hideFlags=HideFlags.HideAndDontSave};
            iceObject=new GameObject("Ice geometry (static snapshot)"){hideFlags=HideFlags.HideAndDontSave};iceObject.transform.SetParent(transform,false);iceObject.AddComponent<MeshFilter>().sharedMesh=iceMesh;iceObject.AddComponent<MeshRenderer>().sharedMaterial=iceMaterial;iceObject.AddComponent<MeshCollider>().sharedMesh=iceMesh;
        }
        void OnDrawGizmosSelected()
        {
            if(geometry==null)return;var grid=geometry.Grid;
            for(int q=0;q<grid.Cells.Count;q++)
            {
                double fraction=(geometry.FixedVolumeM3[q]+geometry.IceVolumeM3[q])/grid.CellVolume;if(fraction<=1e-9||fraction>=1-1e-9)continue;grid.Cells.Coordinates(q,out int i,out int j,out int k);D3 c=grid.Centre(i,j,k);
                Gizmos.color=geometry.IceVolumeM3[q]>0?Color.cyan:new Color(1,.65f,.1f,.6f);Gizmos.DrawWireCube(new Vector3((float)c.X,(float)c.Y,(float)c.Z),new Vector3((float)grid.Spacing.X,(float)grid.Spacing.Y,(float)grid.Spacing.Z));
            }
        }
        void OnGUI()
        {
            if(geometry==null||setup==null)return;GUILayout.BeginArea(new Rect(12,12,460,180),GUI.skin.box);GUILayout.Label("S03 | THREE-DIMENSIONAL GRID AND CUT GEOMETRY");
            GUILayout.Label(geometry.Grid.Cells.X+" x "+geometry.Grid.Cells.Y+" x "+geometry.Grid.Cells.Z+" cells; fixed glass / static ice / accessible fluid");GUILayout.Label("Select root in Scene view: amber = glass cut cells, cyan = ice cut cells.");
            double ratio=setup.scenario.physical.geometry.wallThicknessM/System.Math.Max(geometry.Grid.Spacing.X,System.Math.Max(geometry.Grid.Spacing.Y,geometry.Grid.Spacing.Z));GUILayout.Label("Wall thickness / cell size: "+ratio.ToString("F3")+". This coarse preview is not resolved for thermal/flow use.");
            if(fingerprint!=RecordJson.Hash(RecordJson.Encode(setup.scenario)))GUILayout.Label("STALE geometry: recreate the S03 preview after changing its inputs.");GUILayout.Label("No velocity, heat transfer or phase-change integration yet (S04 onwards).");GUILayout.EndArea();
        }
        void OnDisable(){Release(iceObject);Release(iceMesh);Release(iceMaterial);iceObject=null;iceMesh=null;iceMaterial=null;geometry=null;}
        static void Release(Object resource){if(resource==null)return;if(Application.isPlaying)Destroy(resource);else DestroyImmediate(resource);}
    }
}
