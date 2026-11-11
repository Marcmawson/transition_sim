using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Runtime.Serialization;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Geometry;

namespace PhysicsSim.ThreePhase3D.Verification
{
    [DataContract] public sealed class GeometryCaseRecord
    {
        [DataMember] public string id;
        [DataMember] public int nx,ny,nz,triangles,patches;
        [DataMember] public long planned_gpu_bytes;
        [DataMember] public double analytic_volume_m3,mesh_volume_m3,cut_volume_m3,analytic_area_m2,mesh_area_m2,cut_area_m2,elapsed_cpu_ms,roundoff_volume_adjustment_m3,roundoff_face_adjustment_m2,feature_thickness_m,cells_across_feature;
    }
    [DataContract] public sealed class GeometryReport
    {
        [DataMember] public string schema="S03-CPU-1",utc,source_base="b6dc56f0bb7c00695195827dbb212c5427bc8258";
        [DataMember] public bool gpu_executed=false,complete_stage=false;
        [DataMember] public MaterialComparison[] comparisons;
        [DataMember] public GeometryCaseRecord[] cases;
    }
    public sealed class GeometryChecks
    {
        private readonly List<MaterialComparison> rows=new List<MaterialComparison>();
        private readonly List<GeometryCaseRecord> cases=new List<GeometryCaseRecord>();
        private void Check(string id,string quantity,string unit,double actual,double expected,double tolerance)
        {double error=Math.Abs(actual-expected);rows.Add(new MaterialComparison{case_id=id,quantity=quantity,unit=unit,source="S03 synthetic analytic geometry and declared numerical contract",actual=actual,expected=expected,tolerance=tolerance,absolute_error=error,passed=SiUnits.IsFinite(error)&&error<=tolerance});}
        private void Reject(string id,Action action)
        {bool rejected=false;try{action();}catch(ArgumentException){rejected=true;}catch(InvalidOperationException){rejected=true;}catch(NotSupportedException){rejected=true;}catch(IndexOutOfRangeException){rejected=true;}Check(id,"rejected","1",rejected?1:0,1,0);}
        public GeometryReport Run()
        {
            var grid=new Grid3D(3,5,7,new D3(.11,.07,.13),ghost:2);
            Check("MAC-x-extent","physical x-face extent","1",grid.XFaces.X,4,0);Check("MAC-y-extent","physical y-face extent","1",grid.YFaces.Y,6,0);Check("MAC-z-extent","physical z-face extent","1",grid.ZFaces.Z,8,0);
            foreach(GridLocation location in Enum.GetValues(typeof(GridLocation)))
            {
                var shape=grid.Shape(location);var seen=new HashSet<int>();
                for(int q=0;q<shape.Count;q++){shape.Coordinates(q,out int i,out int j,out int k);if(shape.Index(i,j,k)!=q||!seen.Add(q))throw new InvalidOperationException("Grid index/inverse alias.");}
                Check("index-"+location,"unique allocated indices","1",seen.Count,shape.Count,0);Reject("index-outside-"+location,()=>shape.Index(shape.X+shape.Ghost,0,0));
            }
            Reject("index-overflow",()=>new GridShape(int.MaxValue,8,8,1));Reject("grid-negative-spacing",()=>new Grid3D(2,2,2,new D3(1,-1,1)));
            Reject("grid-nonrigid-frame",()=>new RigidFrame(new D3(),new D3(2,0,0),new D3(0,1,0),new D3(0,0,1)));
            var plan=new GridGpuPlan(grid);Check("memory-plan","owned bytes","byte",plan.OwnedBytes,plan.Buffers.Sum(b=>b.Bytes),0);
            Check("memory-plan","independent typed-layout byte formula","byte",plan.OwnedBytes,80L*grid.Cells.Count+8L*(grid.XFaces.Count+grid.YFaces.Count+grid.ZFaces.Count)+104,0);
            var bindings=plan.Buffers.ToDictionary(s=>s.Name,s=>s);plan.VerifyBindings("ProbeMetrics",bindings);bindings.Remove("IceInterface");Reject("missing-per-kernel-binding",()=>plan.VerifyBindings("ProbeMetrics",bindings));
            bindings=plan.Buffers.ToDictionary(s=>s.Name,s=>s);bindings["FixedInterface"]=new GridBufferSpec("FixedInterface",bindings["FixedInterface"].Count,12);Reject("wrong-structured-stride",()=>plan.VerifyBindings("ProbeMetrics",bindings));Reject("unregistered-kernel",()=>plan.Required("NeverRegistered"));Reject("preflight-memory-budget",()=>new GridGpuPlan(grid,1));
            double priorCylinder=double.MaxValue,priorSphere=double.MaxValue;
            for(int level=0;level<3;level++)
            {
                int n=4<<level,segments=12<<level;var rotation=RigidFrame.Euler(new D3(),.17+.23*level,-.11*level,.07*level);
                var frame=new RigidFrame(rotation.ToWorld(new D3(-.5,-.5,-.5)),rotation.U,rotation.V,rotation.W);
                var g=new Grid3D(n,n+1,n+2,new D3(1.0/n,1.0/(n+1),1.0/(n+2)),frame);
                var pose=RigidFrame.Euler(new D3(.023,-.031,.017),.31,-.19,.13);
                var box=SurfacePrimitives.Transform(SurfacePrimitives.Box(new D3(-.18,-.15,-.13),new D3(.18,.15,.13)),pose);
                Case("box-offset-rotated-"+level,g,box,.36*.30*.26,2*(.36*.30+.36*.26+.30*.26));
                var cylinder=SurfacePrimitives.Transform(SurfacePrimitives.Cylinder(.18,.4,segments),pose);
                var c=Case("cylinder-offset-rotated-"+level,g,cylinder,Math.PI*.18*.18*.4,2*Math.PI*.18*(.18+.4));
                double ce=Math.Abs(c.cut_volume_m3/c.analytic_volume_m3-1);Check("cylinder-convergence-"+level,"decreasing analytic error","1",ce<priorCylinder?1:0,1,0);priorCylinder=ce;
                var sphere=SurfacePrimitives.Transform(SurfacePrimitives.Sphere(.21,segments/2,segments),pose);
                var s=Case("sphere-offset-rotated-"+level,g,sphere,4*Math.PI*Math.Pow(.21,3)/3,4*Math.PI*.21*.21);
                double se=Math.Abs(s.cut_volume_m3/s.analytic_volume_m3-1);Check("sphere-convergence-"+level,"decreasing analytic error","1",se<priorSphere?1:0,1,0);priorSphere=se;
                if(level==2){Check("cylinder-final","relative analytic volume error","1",ce,0,.01);Check("sphere-final","relative analytic volume error","1",se,0,.01);Check("sphere-final","relative analytic area error","1",Math.Abs(s.cut_area_m2/s.analytic_area_m2-1),0,.01);}
                var beakerInputs=new GeometryInputs{innerRadiusM=.15,wallThicknessM=.015,baseThicknessM=.025,cavityHeightM=.3};
                var beaker=SurfacePrimitives.Transform(ApparatusSurface.Beaker(beakerInputs,segments),RigidFrame.Euler(new D3(0,-.28,0),.21,0,0));double r=beakerInputs.innerRadiusM,R=beakerInputs.OuterRadiusM,h=beakerInputs.cavityHeightM,b=beakerInputs.baseThicknessM;
                Case("hollow-beaker-"+level,g,beaker,Math.PI*(R*R*(h+b)-r*r*h),2*Math.PI*(R*R+R*(h+b)+r*h),.015);
            }
            var alignedGrid=new Grid3D(4,4,4,new D3(.25,.25,.25),new RigidFrame(new D3(-.5,-.5,-.5),new D3(1,0,0),new D3(0,1,0),new D3(0,0,1)));
            var aligned=SurfacePrimitives.Box(new D3(-.25,-.25,-.25),new D3(.25,.25,.25));Case("aligned-box",alignedGrid,aligned,.125,1.5);
            var metrics=CutCellGeometry.Build(alignedGrid,aligned);Check("aligned-face","blocked area at wall","m2",metrics.OpenAreaX[alignedGrid.XFaces.Index(1,1,1)],0,1e-10);
            Reject("immutable-geometry-snapshot",()=>((IList<double>)metrics.FixedVolumeM3)[0]=1);
            Check("aligned-face","open exterior face","m2",metrics.OpenAreaX[alignedGrid.XFaces.Index(0,1,1)],.0625,1e-10);
            var translated=CutCellGeometry.Build(alignedGrid,SurfacePrimitives.Box(new D3(-.27,-.13,-.17),new D3(.23,.19,.14)));Check("translated-partial-face","analytic open rectangle area","m2",translated.OpenAreaX[alignedGrid.XFaces.Index(2,1,2)],.0625-.13*.14,1e-10);
            var thin=SurfacePrimitives.Box(new D3(-.0075,-.2,-.2),new D3(.0075,.2,.2));var thinMetric=CutCellGeometry.Build(alignedGrid,thin);Check("thin-wall-not-sampled-away","volume","m3",thinMetric.FixedVolumeM3.Sum(),.015*.4*.4,1e-12);Reject("unresolved-wall-gate",()=>thinMetric.RequireFeatureResolution(.015));
            var emptyFine=CutCellGeometry.Build(new Grid3D(2,2,2,new D3(.005,.005,.005)));emptyFine.RequireFeatureResolution(.015);
            var merge=new TinyCellMergePlan(CutCellGeometry.Build(alignedGrid,SurfacePrimitives.Box(new D3(-.5,-.5,-.5),new D3(.249,-.25,.5))),.05);
            Check("merge-is-exercised","merged cells>0","1",merge.MergedCellCount>0?1:0,1,0);
            Reject("immutable-merge-map",()=>((IList<int>)merge.Group)[0]=1);
            Check("merge-volume","total fluid volume","m3",merge.GroupFluidVolumeM3.Sum(),1-.749*.25,1e-10);
            var mass=new double[merge.Group.Count];for(int q=0;q<mass.Length;q++){alignedGrid.Cells.Coordinates(q,out int i,out int j,out int k);if(alignedGrid.Cells.Interior(i,j,k))mass[q]=q*.013;}Check("merge-extensive-inventory","total mass","kg",merge.AggregateInventory(mass).Sum(),mass.Sum(),1e-9);mass[0]=1;Reject("ghosts-own-no-inventory",()=>merge.AggregateInventory(mass));
            var ice=SurfacePrimitives.Box(new D3(.3,.3,.3),new D3(.4,.4,.4));var separate=CutCellGeometry.Build(alignedGrid,aligned,ice);Check("ice-distinct-from-fixed","ice volume","m3",separate.IceVolumeM3.Sum(),.001,1e-12);Reject("fixed-ice-overlap",()=>CutCellGeometry.Build(alignedGrid,aligned,aligned));
            var contact=CutCellGeometry.Build(alignedGrid,SurfacePrimitives.Box(new D3(-.25,-.25,-.25),new D3(0,.25,.25)),SurfacePrimitives.Box(new D3(0,-.25,-.25),new D3(.25,.25,.25)));
            Check("aligned-solid-contact","blocked face counted once","m2",contact.OpenAreaX[alignedGrid.XFaces.Index(2,1,1)],0,1e-10);Check("aligned-solid-contact","two sided contact area","m2",contact.Patches.Sum(p=>p.ContactAreaM2),.5,1e-9);Check("aligned-solid-contact","external fluid interface area","m2",contact.FixedAreaM2.Sum()+contact.IceAreaM2.Sum(),1.5,1e-9);
            ContactClosure("aligned-solid-contact",contact);
            var previewGrid=new Grid3D(16,20,16,new D3(.18/16,.22/20,.18/16),new RigidFrame(new D3(-.09,.10,-.09),new D3(1,0,0),new D3(0,1,0),new D3(0,0,1)));var previewInputs=new GeometryInputs();var glass=ApparatusSurface.Beaker(previewInputs,48);
            var preview=CutCellGeometry.Build(previewGrid,glass,SurfacePrimitives.Box(new D3(-.018,.123,-.018),new D3(.018,.159,.018)));Check("preview-contact","ice volume","m3",preview.IceVolumeM3.Sum(),Math.Pow(.036,3),1e-12);Check("preview-contact","glass volume unchanged","m3",preview.FixedVolumeM3.Sum(),glass.SignedVolumeM3(),1e-12);Check("preview-contact","two sided glass-ice contact","m2",preview.Patches.Sum(p=>p.ContactAreaM2),2*.036*.036,1e-9);
            ContactClosure("preview-contact",preview);
            var isolated=CutCellGeometry.Build(alignedGrid,SurfacePrimitives.BoxShell(new D3(-.6,-.6,-.6),new D3(.6,.6,.6),new D3(.02,.02,.02),new D3(.03,.03,.03)));Check("isolated-pocket-retained","fluid volume","m3",isolated.FluidVolumeM3.Sum(),1e-6,1e-12);Reject("isolated-tiny-component-rejected",()=>new TinyCellMergePlan(isolated));
            var connectedTiny=CutCellGeometry.Build(alignedGrid,SurfacePrimitives.BoxShell(new D3(-.6,-.6,-.6),new D3(.6,.6,.6),new D3(-.01,.02,.02),new D3(.01,.03,.03)));Check("two-cell-pocket-retained","fluid volume","m3",connectedTiny.FluidVolumeM3.Sum(),2e-6,1e-12);Reject("tiny-connected-component-rejected",()=>new TinyCellMergePlan(connectedTiny));
            var broken=new TriangleSurface{positionsM=aligned.positionsM,triangles=aligned.triangles.Take(aligned.triangles.Length-3).ToArray()};Reject("open-surface-rejected",()=>CutCellGeometry.Build(alignedGrid,broken));
            return new GeometryReport{utc=DateTime.UtcNow.ToString("o"),comparisons=rows.ToArray(),cases=cases.ToArray()};
        }
        private void ContactClosure(string id,CutCellGeometry geometry)
        {
            var grid=geometry.Grid;double maximum=0;
            for(int k=0;k<grid.Cells.Z;k++)for(int j=0;j<grid.Cells.Y;j++)for(int i=0;i<grid.Cells.X;i++)
            {int q=grid.Cells.Index(i,j,k);var net=new D3(geometry.OpenAreaX[grid.XFaces.Index(i+1,j,k)]-geometry.OpenAreaX[grid.XFaces.Index(i,j,k)],geometry.OpenAreaY[grid.YFaces.Index(i,j+1,k)]-geometry.OpenAreaY[grid.YFaces.Index(i,j,k)],geometry.OpenAreaZ[grid.ZFaces.Index(i,j,k+1)]-geometry.OpenAreaZ[grid.ZFaces.Index(i,j,k)]);maximum=Math.Max(maximum,(net-geometry.FixedNormalArea[q]-geometry.IceNormalArea[q]).Length);}
            Check(id,"fluid interface closure excludes solid contact","m2",maximum,0,Math.Max(grid.FaceArea(0),Math.Max(grid.FaceArea(1),grid.FaceArea(2)))*2e-6);
        }
        private GeometryCaseRecord Case(string id,Grid3D grid,TriangleSurface surface,double analyticVolume,double analyticArea,double featureThickness=0)
        {
            var clock=Stopwatch.StartNew();var geometry=CutCellGeometry.Build(grid,surface);clock.Stop();
            double maxPartition=0,maxClosure=0;int ghostErrors=0;
            for(int q=0;q<grid.Cells.Count;q++)
            {
                grid.Cells.Coordinates(q,out int i,out int j,out int k);
                if(!grid.Cells.Interior(i,j,k)){if(geometry.FluidVolumeM3[q]!=0||geometry.FixedVolumeM3[q]!=0)ghostErrors++;continue;}
                maxPartition=Math.Max(maxPartition,Math.Abs(geometry.FluidVolumeM3[q]+geometry.FixedVolumeM3[q]+geometry.IceVolumeM3[q]-grid.CellVolume));
                var flux=new D3(geometry.OpenAreaX[grid.XFaces.Index(i+1,j,k)]-geometry.OpenAreaX[grid.XFaces.Index(i,j,k)],geometry.OpenAreaY[grid.YFaces.Index(i,j+1,k)]-geometry.OpenAreaY[grid.YFaces.Index(i,j,k)],geometry.OpenAreaZ[grid.ZFaces.Index(i,j,k+1)]-geometry.OpenAreaZ[grid.ZFaces.Index(i,j,k)]);
                maxClosure=Math.Max(maxClosure,(flux-geometry.FixedNormalArea[q]-geometry.IceNormalArea[q]).Length);
            }
            var record=new GeometryCaseRecord{id=id,nx=grid.Cells.X,ny=grid.Cells.Y,nz=grid.Cells.Z,triangles=surface.triangles.Length/3,patches=geometry.Patches.Count,planned_gpu_bytes=new GridGpuPlan(grid).OwnedBytes,analytic_volume_m3=analyticVolume,analytic_area_m2=analyticArea,mesh_volume_m3=surface.SignedVolumeM3(),mesh_area_m2=SurfacePrimitives.Area(surface),cut_volume_m3=geometry.FixedVolumeM3.Sum(),cut_area_m2=geometry.FixedAreaM2.Sum(),elapsed_cpu_ms=clock.Elapsed.TotalMilliseconds,roundoff_volume_adjustment_m3=geometry.RoundoffVolumeAdjustmentM3,roundoff_face_adjustment_m2=geometry.RoundoffFaceAreaAdjustmentM2};cases.Add(record);
            record.feature_thickness_m=featureThickness;record.cells_across_feature=featureThickness/Math.Max(grid.Spacing.X,Math.Max(grid.Spacing.Y,grid.Spacing.Z));
            Check(id,"mesh-to-cut volume","m3",record.cut_volume_m3,record.mesh_volume_m3,analyticVolume*1e-8);Check(id,"mesh-to-cut area","m2",record.cut_area_m2,record.mesh_area_m2,analyticArea*1e-8);Check(id,"partition residual","m3",maxPartition,0,grid.CellVolume*1e-8);Check(id,"oriented face/interface closure","m2",maxClosure,0,grid.FaceArea(0)*2e-6);Check(id,"ghost cells untouched","1",ghostErrors,0,0);
            Console.WriteLine("S03 "+id+" CPU_ms="+clock.Elapsed.TotalMilliseconds.ToString("F1",CultureInfo.InvariantCulture));return record;
        }
        public static string Execute(string root)
        {
            var checks=new GeometryChecks();var report=checks.Run();string output=Path.Combine(root,"ValidationRuns/local","s03-geometry-"+DateTime.UtcNow.ToString("yyyyMMddTHHmmss")+"-"+Guid.NewGuid().ToString("N"));Directory.CreateDirectory(output);File.WriteAllBytes(Path.Combine(output,"geometry-results.json"),RecordJson.Encode(report));
            string[] sources=Directory.GetFiles(Path.Combine(root,"Assets/Scripts/ThreePhase3D"),"*.cs",SearchOption.AllDirectories).Concat(Directory.GetFiles(Path.Combine(root,"Assets/Resources/ThreePhase3D"),"*.compute")).Concat(new[]{Path.Combine(root,"Tools/ThreePhase3D/GeometryMain.cs"),Path.Combine(root,"Tools/ThreePhase3D/RunGeometryChecks.ps1")}).OrderBy(p=>p,StringComparer.Ordinal).ToArray();
            File.WriteAllText(Path.Combine(output,"source-hashes.txt"),string.Join("\n",sources.Select(p=>Diagnostics.ProvenanceCapture.SourceHash(p)+"  "+p.Substring(root.Length+1).Replace('\\','/')))+"\n");
            File.WriteAllText(Path.Combine(output,"runner.txt"),"Managed executable SHA256: "+RecordJson.FileHash(typeof(GeometryChecks).Assembly.Location)+"\nManaged runtime: "+Environment.Version+"\nHost OS: "+Environment.OSVersion+"\nCompiler/reference environment: Unity 2022.3.46f1; see RunGeometryChecks.ps1\nCPU geometry timings include clipping and metric assembly; no GPU speed claim.\nGPU executed: false\nStage complete: false\n");
            File.WriteAllText(Path.Combine(output,"files.sha256"),string.Join("\n",Directory.GetFiles(output).Select(p=>RecordJson.FileHash(p)+"  "+Path.GetFileName(p)))+"\n");
            foreach(var row in report.comparisons.Where(r=>!r.passed))Console.WriteLine("FAIL "+row.case_id+" "+row.quantity+" actual="+row.actual+" expected="+row.expected+" tolerance="+row.tolerance);
            Console.WriteLine("S03 CPU comparisons="+report.comparisons.Length+" failures="+report.comparisons.Count(r=>!r.passed)+"; GPU executed=false. "+output);if(report.comparisons.Any(r=>!r.passed))throw new InvalidOperationException("S03 geometry checks failed; evidence retained.");return output;
        }
    }
}
