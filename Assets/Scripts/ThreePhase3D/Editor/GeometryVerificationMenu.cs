using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Runtime.Serialization;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.Rendering;
using PhysicsSim.ThreePhase3D.Verification;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Debug=UnityEngine.Debug;

namespace PhysicsSim.ThreePhase3D.Editor
{
    [DataContract] public sealed class GeometryGpuAllocation
    { [DataMember] public string case_id,name;[DataMember] public int count,stride;[DataMember] public long bytes; }
    [DataContract] public sealed class GeometryKernelTiming
    { [DataMember] public string case_id,kernel,measurement="Host wall time including synchronous GPU readback; not isolated GPU execution time";[DataMember] public double elapsed_ms; }
    [DataContract] public sealed class GeometryGpuInputs
    { [DataMember] public string case_id;[DataMember] public int nx,ny,nz,ghost;[DataMember] public double[] spacing_m,frame_origin_uvw;[DataMember] public TriangleSurface fixed_surface,ice_surface; }
    [DataContract] public sealed class GeometryGpuReport
    {
        [DataMember] public string schema="S03-GPU-1",utc,device,api,unity,shader_sha256,error;
        [DataMember] public bool gpu_executed,passed,all_resources_released;
        [DataMember] public long peak_owned_bytes;
        [DataMember] public MaterialComparison[] comparisons;
        [DataMember] public GeometryGpuAllocation[] allocations;
        [DataMember] public GeometryKernelTiming[] timings;
    }
    public static class GeometryVerificationMenu
    {
        [MenuItem("Physics Sim/Three Phase 3D/Run S03 CPU grid and geometry checks")]
        public static void RunCpu()=>Debug.Log("S03 CPU evidence: "+GeometryChecks.Execute(Directory.GetParent(Application.dataPath).FullName));
        [MenuItem("Physics Sim/Three Phase 3D/Run S03 GPU resource and geometry checks")]
        public static void RunGpu()
        {
            string root=Directory.GetParent(Application.dataPath).FullName,output=Path.Combine(root,"ValidationRuns/local","s03-gpu-"+DateTime.UtcNow.ToString("yyyyMMddTHHmmss")+"-"+Guid.NewGuid().ToString("N"));Directory.CreateDirectory(output);
            var report=new GeometryGpuReport{utc=DateTime.UtcNow.ToString("o"),device=SystemInfo.graphicsDeviceName,api=SystemInfo.graphicsDeviceType.ToString(),unity=Application.unityVersion,all_resources_released=true};var rows=new List<MaterialComparison>();var allocations=new List<GeometryGpuAllocation>();var timings=new List<GeometryKernelTiming>();
            try
            {
                GeometryChecks.Execute(root);var shader=Resources.Load<ComputeShader>("ThreePhase3D/GridGeometryChecks");if(shader==null)throw new InvalidOperationException("GridGeometryChecks.compute failed to import/load.");report.shader_sha256=RecordJson.FileHash(Path.Combine(root,"Assets/Resources/ThreePhase3D/GridGeometryChecks.compute"));
                foreach(string id in new[]{"box","cylinder","sphere","beaker","fixed-and-ice","aligned-contact"})
                {
                    var orientation=RigidFrame.Euler(new D3(),.27,-.14,.09);var frame=new RigidFrame(orientation.ToWorld(new D3(-.5,-.5,-.5)),orientation.U,orientation.V,orientation.W);var grid=new Grid3D(5,7,9,new D3(.2,1.0/7,1.0/9),frame,2);
                    if(id=="aligned-contact"){frame=new RigidFrame(new D3(-.5,-.5,-.5),new D3(1,0,0),new D3(0,1,0),new D3(0,0,1));grid=new Grid3D(6,8,10,new D3(1.0/6,1.0/8,.1),frame,2);}
                    TriangleSurface ice=null,fixedSurface;
                    if(id=="box")fixedSurface=SurfacePrimitives.Box(new D3(-.2,-.17,-.13),new D3(.2,.17,.13));
                    else if(id=="cylinder")fixedSurface=SurfacePrimitives.Cylinder(.2,.4,24);
                    else if(id=="sphere")fixedSurface=SurfacePrimitives.Sphere(.21,12,24);
                    else if(id=="beaker")fixedSurface=SurfacePrimitives.Transform(ApparatusSurface.Beaker(new GeometryInputs{innerRadiusM=.15,wallThicknessM=.015,baseThicknessM=.025,cavityHeightM=.3},24),RigidFrame.Euler(new D3(0,-.28,0),.1,0,0));
                    else if(id=="aligned-contact"){fixedSurface=SurfacePrimitives.Box(new D3(-.25,-.25,-.25),new D3(0,.25,.25));ice=SurfacePrimitives.Box(new D3(0,-.25,-.25),new D3(.25,.25,.25));}
                    else{fixedSurface=SurfacePrimitives.Box(new D3(-.35,-.2,-.2),new D3(-.05,.2,.2));ice=SurfacePrimitives.Transform(SurfacePrimitives.Sphere(.1,8,16),RigidFrame.Euler(new D3(.23,0,0),0,0,0));}
                    var inputs=new GeometryGpuInputs{case_id=id,nx=grid.Cells.X,ny=grid.Cells.Y,nz=grid.Cells.Z,ghost=grid.Cells.Ghost,spacing_m=new[]{grid.Spacing.X,grid.Spacing.Y,grid.Spacing.Z},frame_origin_uvw=new[]{frame.Origin.X,frame.Origin.Y,frame.Origin.Z,frame.U.X,frame.U.Y,frame.U.Z,frame.V.X,frame.V.Y,frame.V.Z,frame.W.X,frame.W.Y,frame.W.Z},fixed_surface=fixedSurface,ice_surface=ice};File.WriteAllBytes(Path.Combine(output,id+"-inputs.json"),RecordJson.Encode(inputs));
                    var geometry=CutCellGeometry.Build(grid,fixedSurface,ice);GridGpuResources resources=null;
                    try
                    {
                        resources=new GridGpuResources(geometry);report.peak_owned_bytes=Math.Max(report.peak_owned_bytes,resources.LiveBytes);
                        foreach(var spec in resources.Plan.Buffers)allocations.Add(new GeometryGpuAllocation{case_id=id,name=spec.Name,count=spec.Count,stride=spec.Stride,bytes=spec.Bytes});
                        foreach(string name in new[]{"CellField","VelocityX","VelocityY","VelocityZ"})resources.Buffer(name).SetData(Enumerable.Repeat(42f,resources.Buffer(name).count-1).ToArray());
                        var clock=Stopwatch.StartNew();resources.Dispatch(shader,"ClearFields");double maximum=0;
                        foreach(string name in new[]{"CellField","VelocityX","VelocityY","VelocityZ"}){var values=new float[resources.Buffer(name).count-1];resources.Buffer(name).GetData(values,0,0,values.Length);foreach(float value in values)maximum=Math.Max(maximum,Math.Abs(value));}
                        clock.Stop();report.gpu_executed=true;timings.Add(new GeometryKernelTiming{case_id=id,kernel="ClearFields",elapsed_ms=clock.Elapsed.TotalMilliseconds});Add(rows,id,"zero initialized cell and all staggered face fields","1",maximum,0,0);
                        resources.Buffer("Status").SetData(Enumerable.Repeat(uint.MaxValue,grid.Cells.Count).ToArray());
                        clock.Restart();resources.Dispatch(shader,"ProbeMetrics");var status=new uint[grid.Cells.Count];var residual=new Vector4[grid.Cells.Count];resources.Buffer("Status").GetData(status,0,0,status.Length);resources.Buffer("Results").GetData(residual,0,0,residual.Length);clock.Stop();timings.Add(new GeometryKernelTiming{case_id=id,kernel="ProbeMetrics",elapsed_ms=clock.Elapsed.TotalMilliseconds});
                        for(int q=0;q<status.Length;q++)
                        {
                            Add(rows,id+"-"+q,"geometry status","1",status[q],0,0);grid.Cells.Coordinates(q,out int i,out int j,out int k);bool physical=grid.Cells.Interior(i,j,k);
                            var expected=physical?new D3(geometry.OpenAreaX[grid.XFaces.Index(i+1,j,k)]-geometry.OpenAreaX[grid.XFaces.Index(i,j,k)],geometry.OpenAreaY[grid.YFaces.Index(i,j+1,k)]-geometry.OpenAreaY[grid.YFaces.Index(i,j,k)],geometry.OpenAreaZ[grid.ZFaces.Index(i,j,k+1)]-geometry.OpenAreaZ[grid.ZFaces.Index(i,j,k)])-geometry.FixedNormalArea[q]-geometry.IceNormalArea[q]:new D3();
                            Add(rows,id+"-"+q,"partition residual","m3",residual[q].x,0,grid.CellVolume*2e-5);for(int a=0;a<3;a++)Add(rows,id+"-"+q,"oriented closure-"+a,"m2",residual[q][a+1],expected[a],Math.Max(grid.FaceArea(0),Math.Max(grid.FaceArea(1),grid.FaceArea(2)))*2e-5);
                        }
                        Add(rows,id,"all tail guards intact","1",resources.TailGuardsIntact()?1:0,1,0);
                        int first=grid.Cells.Index(0,0,0);resources.Buffer("FixedVolume").SetData(new[]{float.NaN},0,first,1);resources.Dispatch(shader,"ProbeMetrics");var flag=new uint[1];resources.Buffer("Status").GetData(flag,0,first,1);Add(rows,id,"injected nonfinite metric rejected","1",(flag[0]&1)!=0?1:0,1,0);
                    }
                    finally
                    {
                        if(resources!=null){var handles=resources.Plan.Buffers.Select(s=>resources.Buffer(s.Name)).ToArray();resources.Dispose();resources.Dispose();bool released=resources.Disposed&&resources.LiveBytes==0&&handles.All(h=>!h.IsValid());report.all_resources_released&=released;Add(rows,id,"all owned GPU resources released","1",released?1:0,1,0);}
                    }
                }
                report.passed=rows.Count>0&&rows.All(r=>r.passed)&&report.all_resources_released;if(!report.passed)throw new InvalidOperationException("S03 GPU checks failed.");
            }
            catch(Exception e){if(e.Data.Contains("S03_PartialAllocationPeakBytes"))report.peak_owned_bytes=Math.Max(report.peak_owned_bytes,(long)e.Data["S03_PartialAllocationPeakBytes"]);report.error=e.ToString();throw;}
            finally
            {
                report.comparisons=rows.ToArray();report.allocations=allocations.ToArray();report.timings=timings.ToArray();File.WriteAllBytes(Path.Combine(output,"gpu-results.json"),RecordJson.Encode(report));File.WriteAllText(Path.Combine(output,"files.sha256"),string.Join("\n",Directory.GetFiles(output).Where(p=>!p.EndsWith("files.sha256",StringComparison.Ordinal)).Select(p=>RecordJson.FileHash(p)+"  "+Path.GetFileName(p)))+"\n");Debug.Log("S03 GPU evidence: "+output+"; executed="+report.gpu_executed+"; passed="+report.passed);
            }
        }
        private static void Add(List<MaterialComparison> rows,string id,string quantity,string unit,double actual,double expected,double tolerance)
        {double error=Math.Abs(actual-expected);rows.Add(new MaterialComparison{case_id=id,quantity=quantity,unit=unit,source="S03 actual GPU vs CPU geometry/resource contract",actual=actual,expected=expected,absolute_error=error,tolerance=tolerance,passed=!double.IsNaN(error)&&error<=tolerance});}
        public static void RunGpuBatch(){try{RunGpu();EditorApplication.Exit(0);}catch(Exception e){Debug.LogException(e);EditorApplication.Exit(1);}}
        [MenuItem("Physics Sim/Three Phase 3D/Create S03 grid geometry preview")]
        public static void CreatePreview()
        {
            if(!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo())return;
            try
            {
                EditorUtility.DisplayProgressBar("S03 geometry","Clipping the shared glass and ice surfaces; no fluid solve",.2f);EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects,NewSceneMode.Single);
                var root=new GameObject("S03 shared geometry preview");var setup=root.AddComponent<Beaker3DSetup>();setup.scenario.scenarioId="S03-synthetic-grid-geometry-preview";setup.scenario.display.cylinderSegments=48;setup.scenario.display.showSetupReport=false;setup.Refresh();
                root.AddComponent<GridGeometryPreview>().Configure(setup);
                var camera=Camera.main;if(camera!=null){Vector3 target=new Vector3(0,.20f,0);camera.transform.position=target+new Vector3(.23f,.13f,-.30f);camera.transform.LookAt(target);camera.orthographic=true;camera.orthographicSize=.14f;camera.clearFlags=CameraClearFlags.SolidColor;camera.backgroundColor=new Color(.035f,.045f,.055f);}
                Selection.activeGameObject=root;SceneView.lastActiveSceneView?.FrameSelected();Debug.Log("S03 preview created. Select the root and enable Scene gizmos to see cut cells. Geometry only; wall resolution is reported and dynamics remain disabled.");
            }
            finally{EditorUtility.ClearProgressBar();}
        }
    }
}
