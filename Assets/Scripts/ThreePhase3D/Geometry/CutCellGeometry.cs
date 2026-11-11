using System;
using System.Collections.Generic;
using System.Linq;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Geometry
{
    public enum SolidRole { Fixed, Ice }
    public sealed class InterfacePatch
    {
        public readonly int Cell;public readonly SolidRole Role;
        public readonly D3 CentroidLocalM,NormalLocal;
        public readonly double AreaM2,ContactAreaM2,CellCentreNormalDistanceM;
        public double FluidExposedAreaM2=>AreaM2-ContactAreaM2;
        internal readonly IReadOnlyList<D3> Polygon;
        internal InterfacePatch(int cell,SolidRole role,D3 centroid,D3 normal,double area,double contact,D3 centre,IReadOnlyList<D3> polygon)
        {Polygon=polygon;Cell=cell;Role=role;CentroidLocalM=centroid;NormalLocal=normal;AreaM2=area;ContactAreaM2=contact;CellCentreNormalDistanceM=Math.Abs(D3.Dot(centre-centroid,normal));}
    }
    internal sealed class ClosedSurface
    {
        internal readonly List<D3[]> Triangles=new List<D3[]>();
        internal readonly List<SignedTetrahedron> Tetrahedra=new List<SignedTetrahedron>();
        internal readonly D3 Min,Max;
        internal ClosedSurface(TriangleSurface surface,Grid3D grid)
        {
            if(surface==null||surface.positionsM==null||surface.triangles==null||surface.positionsM.Length<12||surface.positionsM.Length%3!=0||surface.triangles.Length%3!=0)throw new ArgumentException("A closed oriented triangle surface is required.");
            var vertices=new D3[surface.positionsM.Length/3];
            for(int i=0;i<vertices.Length;i++)vertices[i]=grid.Frame.ToLocal(new D3(surface.positionsM[3*i],surface.positionsM[3*i+1],surface.positionsM[3*i+2]));
            foreach(var p in vertices)for(int a=0;a<3;a++)if(p[a]/grid.Spacing[a]<int.MinValue+2.0||p[a]/grid.Spacing[a]>int.MaxValue-2.0)throw new ArgumentException("Surface coordinates exceed indexable grid-space range.");
            Min=new D3(vertices.Min(p=>p.X),vertices.Min(p=>p.Y),vertices.Min(p=>p.Z));Max=new D3(vertices.Max(p=>p.X),vertices.Max(p=>p.Y),vertices.Max(p=>p.Z));
            double scale=(Max-Min).Length;if(scale<1e-9)throw new ArgumentException("Surface scale below supported 1 nm geometric envelope.");
            double weld=scale*1e-11;var ids=new Dictionary<string,int>();var vid=new int[vertices.Length];
            for(int i=0;i<vertices.Length;i++){D3 p=vertices[i]-Min;string key=Math.Round(p.X/weld)+":"+Math.Round(p.Y/weld)+":"+Math.Round(p.Z/weld);if(!ids.TryGetValue(key,out int id)){id=ids.Count;ids.Add(key,id);}vid[i]=id;}
            var edges=new Dictionary<string,int>();var multiplicity=new Dictionary<string,int>();double signedVolume=0;
            D3 origin=Min+new D3((Max.X-Min.X)*.419,(Max.Y-Min.Y)*.371,(Max.Z-Min.Z)*.317);
            for(int t=0;t<surface.triangles.Length;t+=3)
            {
                int[] indices={surface.triangles[t],surface.triangles[t+1],surface.triangles[t+2]};foreach(int index in indices)if(index<0||index>=vertices.Length)throw new ArgumentException("Triangle index out of range.");
                var triangle=indices.Select(i=>vertices[i]).ToArray();if(D3.Cross(triangle[1]-triangle[0],triangle[2]-triangle[0]).Length<scale*scale*1e-14)throw new ArgumentException("Degenerate triangle.");Triangles.Add(triangle);
                var tetra=new SignedTetrahedron(origin,triangle[0],triangle[1],triangle[2]);if(tetra.Sign!=0)Tetrahedra.Add(tetra);
                signedVolume+=D3.Dot(triangle[0]-origin,D3.Cross(triangle[1]-origin,triangle[2]-origin))/6;
                for(int e=0;e<3;e++){int a=vid[indices[e]],b=vid[indices[(e+1)%3]];string key=Math.Min(a,b)+":"+Math.Max(a,b);if(!edges.ContainsKey(key)){edges[key]=0;multiplicity[key]=0;}edges[key]+=a<b?1:-1;multiplicity[key]++;}
            }
            if(edges.Values.Any(v=>v!=0)||multiplicity.Values.Any(v=>v!=2)||signedVolume<=0)throw new ArgumentException("Surface must be closed, consistently outward oriented and manifold.");
        }
        internal double ContactArea(IList<D3> polygon,D3 normal,double delta)
        {
            var shifted=polygon.Select(p=>p+normal*delta).ToArray();
            for(int axis=0;axis<3;axis++)if(shifted.Max(p=>p[axis])<Min[axis]||shifted.Min(p=>p[axis])>Max[axis])return 0;
            double area=0;foreach(var tetra in Tetrahedra)
            {
                bool outside=false;for(int axis=0;axis<3;axis++)if(shifted.Max(p=>p[axis])<tetra.Min[axis]||shifted.Min(p=>p[axis])>tetra.Max[axis])outside=true;if(outside)continue;
                var clipped=new List<D3>(shifted);foreach(var plane in tetra.Planes){clipped=PolyhedronClip.Polygon(clipped,plane);if(clipped.Count<3)break;}area+=tetra.Sign*PolyhedronClip.Area(clipped);
            }
            return area;
        }
    }
    // Physics-ID: GEOMETRY-CONSERVATIVE-CUT-CELLS
    // Implements: R009 R055 R058
    // Model: oriented closed-surface signed tetrahedral decomposition and convex clipping
    // Sources: project geometric construction, divergence theorem
    // Source-Locators: Docs/ThreePhase3D/S03-Grid-Geometry.md
    // Inputs-SI: closed fixed/ice meshes [m], shared Grid3D [m]
    // Outputs-SI: solid/fluid volumes [m3], shared apertures [m2], oriented interface patches [m2,m,1]
    // State-Ownership: one immutable geometry snapshot; fixed and ice occupancy are distinct
    // Conservation: physical cell partitions close without density scaling; roundoff changes recorded
    // Discretisation: FP64 clipped signed tetrahedra; exact for the supplied polygonal surface up to roundoff
    // Validity: closed oriented non-self-intersecting meshes; fixed and ice must not overlap
    // Validation: V06 V35
    // Evidence: PENDING
    // Status: IMPLEMENTED_UNVERIFIED
    // Limitations: CPU geometry preprocessing; no moving-body update, thermal law or field solve
    public sealed class CutCellGeometry
    {
        public readonly Grid3D Grid;
        private TriangleSurface fixedInput, iceInput;
        private D3[] solidMomentOffsets;
        internal CutCellGeometry WithFluidCentroidMoments() =>
            solidMomentOffsets != null ? this : Build(Grid, fixedInput, iceInput, true);
        internal D3 FluidCentroidOffset(int cell) => -solidMomentOffsets[cell] / _FluidVolumeM3[cell];

        internal IEnumerable<List<SignedTetrahedron>> BuildSolidTetrahedra()
        {
            if(fixedInput!=null)yield return new ClosedSurface(fixedInput,Grid).Tetrahedra;
            if(iceInput!=null)yield return new ClosedSurface(iceInput,Grid).Tetrahedra;
        }

        internal CutCellGeometry BuildVelocityDual(int axis)
        {
            var shape=Grid.Shape((GridLocation)(axis+1));
            var shift=new D3(axis==0?-.5*Grid.Spacing.X:0,axis==1?-.5*Grid.Spacing.Y:0,axis==2?-.5*Grid.Spacing.Z:0);
            var frame=new RigidFrame(Grid.Frame.ToWorld(shift),Grid.Frame.U,Grid.Frame.V,Grid.Frame.W);
            return Build(new Grid3D(shape.X,shape.Y,shape.Z,Grid.Spacing,frame,shape.Ghost),fixedInput,iceInput,true);
        }

        private static TriangleSurface CopySurface(TriangleSurface surface) => surface==null?null:new TriangleSurface
        {
            positionsM=(double[])surface.positionsM.Clone(),
            triangles=(int[])surface.triangles.Clone()
        };
        private readonly double[] _FixedVolumeM3,_IceVolumeM3,_FluidVolumeM3,_OpenAreaX,_OpenAreaY,_OpenAreaZ;
        private readonly D3[] _FixedNormalArea,_IceNormalArea;
        private readonly double[] _FixedAreaM2,_IceAreaM2,_FixedDistanceM,_IceDistanceM;
        private readonly bool[] fixedHasBoundary,iceHasBoundary;
        private readonly double[][] faceMinus,facePlus;
        public readonly IReadOnlyList<double> FixedVolumeM3;
        public readonly IReadOnlyList<double> IceVolumeM3;
        public readonly IReadOnlyList<double> FluidVolumeM3;
        public readonly IReadOnlyList<double> OpenAreaX;
        public readonly IReadOnlyList<double> OpenAreaY;
        public readonly IReadOnlyList<double> OpenAreaZ;
        public readonly IReadOnlyList<double> FixedAreaM2;
        public readonly IReadOnlyList<double> IceAreaM2;
        public readonly IReadOnlyList<double> FixedDistanceM;
        public readonly IReadOnlyList<double> IceDistanceM;
        public readonly IReadOnlyList<D3> FixedNormalArea;
        public readonly IReadOnlyList<D3> IceNormalArea;
        private readonly List<InterfacePatch> patches=new List<InterfacePatch>();
        public IReadOnlyList<InterfacePatch> Patches=>patches.AsReadOnly();
        public double RoundoffVolumeAdjustmentM3 {get;private set;}
        public double RoundoffFaceAreaAdjustmentM2 {get;private set;}
        private CutCellGeometry(Grid3D grid)
        {
            Grid=grid;int n=grid.Cells.Count;_FixedVolumeM3=new double[n];_IceVolumeM3=new double[n];_FluidVolumeM3=new double[n];_FixedNormalArea=new D3[n];_IceNormalArea=new D3[n];_FixedAreaM2=new double[n];_IceAreaM2=new double[n];_FixedDistanceM=Enumerable.Repeat(-1.0,n).ToArray();_IceDistanceM=Enumerable.Repeat(-1.0,n).ToArray();_OpenAreaX=new double[grid.XFaces.Count];_OpenAreaY=new double[grid.YFaces.Count];_OpenAreaZ=new double[grid.ZFaces.Count];
            fixedHasBoundary=new bool[n];iceHasBoundary=new bool[n];faceMinus=new[]{new double[grid.XFaces.Count],new double[grid.YFaces.Count],new double[grid.ZFaces.Count]};facePlus=new[]{new double[grid.XFaces.Count],new double[grid.YFaces.Count],new double[grid.ZFaces.Count]};
            FixedVolumeM3=Array.AsReadOnly(_FixedVolumeM3);IceVolumeM3=Array.AsReadOnly(_IceVolumeM3);FluidVolumeM3=Array.AsReadOnly(_FluidVolumeM3);OpenAreaX=Array.AsReadOnly(_OpenAreaX);OpenAreaY=Array.AsReadOnly(_OpenAreaY);OpenAreaZ=Array.AsReadOnly(_OpenAreaZ);FixedAreaM2=Array.AsReadOnly(_FixedAreaM2);IceAreaM2=Array.AsReadOnly(_IceAreaM2);FixedDistanceM=Array.AsReadOnly(_FixedDistanceM);IceDistanceM=Array.AsReadOnly(_IceDistanceM);FixedNormalArea=Array.AsReadOnly(_FixedNormalArea);IceNormalArea=Array.AsReadOnly(_IceNormalArea);
        }
        public static CutCellGeometry Build(Grid3D grid,TriangleSurface fixedSurface=null,TriangleSurface iceSurface=null)
            => Build(grid,fixedSurface,iceSurface,false);

        private static CutCellGeometry Build(Grid3D grid,TriangleSurface fixedSurface,TriangleSurface iceSurface,bool moments)
        {
            if(grid==null)throw new ArgumentNullException(nameof(grid));var result=new CutCellGeometry(grid)
            {
                solidMomentOffsets=moments?new D3[grid.Cells.Count]:null
            };
            var fixedMesh=fixedSurface==null?null:new ClosedSurface(fixedSurface,grid);var iceMesh=iceSurface==null?null:new ClosedSurface(iceSurface,grid);
            result.fixedInput=CopySurface(fixedSurface);result.iceInput=CopySurface(iceSurface);
            if(fixedMesh!=null&&iceMesh!=null)RejectOverlap(fixedMesh,iceMesh,grid.CellVolume);
            if(fixedMesh!=null)result.Accumulate(fixedMesh,SolidRole.Fixed,iceMesh);if(iceMesh!=null)result.Accumulate(iceMesh,SolidRole.Ice,fixedMesh);
            for(int k=0;k<grid.Cells.Z;k++)for(int j=0;j<grid.Cells.Y;j++)for(int i=0;i<grid.Cells.X;i++)
            {
                int q=grid.Cells.Index(i,j,k);result._FixedVolumeM3[q]=result.VolumeGuard(result._FixedVolumeM3[q],!result.fixedHasBoundary[q]);result._IceVolumeM3[q]=result.VolumeGuard(result._IceVolumeM3[q],!result.iceHasBoundary[q]);
                double fluid=grid.CellVolume-result._FixedVolumeM3[q]-result._IceVolumeM3[q];result._FluidVolumeM3[q]=result.VolumeGuard(fluid);
            }
            for(int axis=0;axis<3;axis++)
            {
                GridShape shape=grid.Shape((GridLocation)(axis+1));double[] open=result.AreaStorage(axis);double full=grid.FaceArea(axis);
                for(int k=0;k<shape.Z;k++)for(int j=0;j<shape.Y;j++)for(int i=0;i<shape.X;i++){int q=shape.Index(i,j,k);double blocked=Math.Max(result.faceMinus[axis][q],result.facePlus[axis][q]);if(blocked< -full*1e-7||blocked>full*(1+1e-7))throw new InvalidOperationException("Invalid overlapping/nonphysical face occupancy.");double bounded=Math.Max(0,Math.Min(full,blocked));result.RoundoffFaceAreaAdjustmentM2+=Math.Abs(blocked-bounded);open[q]=full-bounded;}
                result.faceMinus[axis]=null;result.facePlus[axis]=null;
            }
            return result;
        }
        private double VolumeGuard(double v,bool noInterface=false)
        {
            if(v< -Grid.CellVolume*1e-8||v>Grid.CellVolume*(1+1e-8))throw new InvalidOperationException("Invalid solid/fluid partition: overlap, winding or geometric tolerance failure.");double bounded=Math.Max(0,Math.Min(Grid.CellVolume,v));
            // A cell without any reconstructed boundary can only be full or empty.
            // Snap roundoff only in that topologically uncut case; retain every
            // positive cut-cell volume, however small, for conservative grouping.
            if(noInterface){if(bounded<Grid.CellVolume*1e-12)bounded=0;else if(Grid.CellVolume-bounded<Grid.CellVolume*1e-12)bounded=Grid.CellVolume;}
            RoundoffVolumeAdjustmentM3+=Math.Abs(v-bounded);return bounded;
        }
        private double[] AreaStorage(int axis)=>axis==0?_OpenAreaX:axis==1?_OpenAreaY:axis==2?_OpenAreaZ:throw new ArgumentOutOfRangeException(nameof(axis));
        public IReadOnlyList<double> Areas(int axis)=>axis==0?OpenAreaX:axis==1?OpenAreaY:axis==2?OpenAreaZ:throw new ArgumentOutOfRangeException(nameof(axis));
        public double SolidFraction(SolidRole role,int cell)
        {if(role!=SolidRole.Fixed&&role!=SolidRole.Ice)throw new ArgumentOutOfRangeException(nameof(role));return (role==SolidRole.Fixed?FixedVolumeM3[cell]:IceVolumeM3[cell])/Grid.CellVolume;}
        public double AccessibleFraction(int cell)=>FluidVolumeM3[cell]/Grid.CellVolume;
        private void Accumulate(ClosedSurface mesh,SolidRole role,ClosedSurface other)
        {
            double[] volume=role==SolidRole.Fixed?_FixedVolumeM3:_IceVolumeM3;var grid=Grid;
            foreach(var tetra in mesh.Tetrahedra)
            {
                int[] lo=new int[3],hi=new int[3];for(int a=0;a<3;a++){lo[a]=Math.Max(0,(int)Math.Floor(tetra.Min[a]/grid.Spacing[a]));hi[a]=Math.Min((a==0?grid.Cells.X:a==1?grid.Cells.Y:grid.Cells.Z)-1,(int)Math.Floor(tetra.Max[a]/grid.Spacing[a]));}
                for(int k=lo[2];k<=hi[2];k++)for(int j=lo[1];j<=hi[1];j++)for(int i=lo[0];i<=hi[0];i++){D3 lower=grid.Lower(i,j,k);int cell=grid.Cells.Index(i,j,k);double clipped=tetra.InBox(lower,lower+grid.Spacing,solidMomentOffsets!=null,out D3 centroid);
                    volume[cell]+=clipped;
                    if(solidMomentOffsets!=null&&clipped!=0)solidMomentOffsets[cell]+=(centroid-lower-grid.Spacing*.5)*clipped;}
            }
            // Signed cone sections are evaluated on both one-sided limits, avoiding
            // double-counted tetra boundaries and blocking an exactly aligned wall.
            for(int axis=0;axis<3;axis++)
            {
                var shape=grid.Shape((GridLocation)(axis+1));double delta=grid.Spacing[axis]*1e-9;
                for(int k=0;k<shape.Z;k++)for(int j=0;j<shape.Y;j++)for(int i=0;i<shape.X;i++)
                {
                    D3 lower=grid.Lower(i,j,k),upper=lower+grid.Spacing;double c=lower[axis];if(c<mesh.Min[axis]-delta||c>mesh.Max[axis]+delta)continue;
                    double minus=0,plus=0;
                    foreach(var tetra in mesh.Tetrahedra)
                    {
                        int u=(axis+1)%3,v=(axis+2)%3;
                        if(upper[u]<tetra.Min[u]||lower[u]>tetra.Max[u]||upper[v]<tetra.Min[v]||lower[v]>tetra.Max[v])continue;
                        if(c-delta>=tetra.Min[axis]&&c-delta<=tetra.Max[axis])minus+=tetra.OnFace(axis,c-delta,lower,upper);
                        if(c+delta>=tetra.Min[axis]&&c+delta<=tetra.Max[axis])plus+=tetra.OnFace(axis,c+delta,lower,upper);
                    }
                    int face=shape.Index(i,j,k);faceMinus[axis][face]+=minus;facePlus[axis][face]+=plus;
                }
            }
            foreach(var tri in mesh.Triangles)
            {
                D3 normal=D3.Cross(tri[1]-tri[0],tri[2]-tri[0]).Unit();int[] lo=new int[3],hi=new int[3];
                for(int a=0;a<3;a++){double min=tri.Min(p=>p[a]),max=tri.Max(p=>p[a]);int n=a==0?grid.Cells.X:a==1?grid.Cells.Y:grid.Cells.Z;lo[a]=Math.Max(0,(int)Math.Floor(min/grid.Spacing[a])-1);hi[a]=Math.Min(n-1,(int)Math.Floor(max/grid.Spacing[a]));}
                for(int k=lo[2];k<=hi[2];k++)for(int j=lo[1];j<=hi[1];j++)for(int i=lo[0];i<=hi[0];i++)
                {
                    D3 lower=grid.Lower(i,j,k),upper=lower+grid.Spacing;bool exclude=false;
                    for(int a=0;a<3;a++){if(tri.All(p=>Math.Abs(p[a]-upper[a])<grid.Spacing[a]*1e-11)&&normal[a]>0)exclude=true;if(tri.All(p=>Math.Abs(p[a]-lower[a])<grid.Spacing[a]*1e-11)&&normal[a]<0)exclude=true;}if(exclude)continue;
                    var polygon=new List<D3>(tri);foreach(var plane in PolyhedronClip.Bounds(lower,upper))polygon=PolyhedronClip.Polygon(polygon,plane);double area=PolyhedronClip.Area(polygon);if(area<=0)continue;
                    int cell=grid.Cells.Index(i,j,k);D3 centroid=PolyhedronClip.Centroid(polygon);double contact=other==null?0:other.ContactArea(polygon,normal,Math.Min(grid.Spacing.X,Math.Min(grid.Spacing.Y,grid.Spacing.Z))*1e-9);
                    double contactTolerance=Math.Max(area*1e-7,Math.Max(grid.FaceArea(0),Math.Max(grid.FaceArea(1),grid.FaceArea(2)))*1e-10);
                    if(contact< -contactTolerance||contact>area+contactTolerance)throw new InvalidOperationException("Invalid solid contact area.");double boundedContact=Math.Max(0,Math.Min(area,contact));RoundoffFaceAreaAdjustmentM2+=Math.Abs(contact-boundedContact);contact=boundedContact;var patch=new InterfacePatch(cell,role,centroid,normal,area,contact,lower+grid.Spacing*.5,polygon);patches.Add(patch);(role==SolidRole.Fixed?fixedHasBoundary:iceHasBoundary)[cell]=true;
                    var areaArray=role==SolidRole.Fixed?_FixedAreaM2:_IceAreaM2;var normals=role==SolidRole.Fixed?_FixedNormalArea:_IceNormalArea;var distance=role==SolidRole.Fixed?_FixedDistanceM:_IceDistanceM;
                    areaArray[cell]+=patch.FluidExposedAreaM2;normals[cell]+=normal*patch.FluidExposedAreaM2;distance[cell]=distance[cell]<0?patch.CellCentreNormalDistanceM:Math.Min(distance[cell],patch.CellCentreNormalDistanceM);
                }
            }
        }
        private static void RejectOverlap(ClosedSurface a,ClosedSurface b,double cellVolume)
        {
            if(a.Max.X<b.Min.X||b.Max.X<a.Min.X||a.Max.Y<b.Min.Y||b.Max.Y<a.Min.Y||a.Max.Z<b.Min.Z||b.Max.Z<a.Min.Z)return;
            double overlap=0;
            foreach(var x in a.Tetrahedra)foreach(var y in b.Tetrahedra)
            {
                if(x.Max.X<y.Min.X||y.Max.X<x.Min.X||x.Max.Y<y.Min.Y||y.Max.Y<x.Min.Y||x.Max.Z<y.Min.Z||y.Max.Z<x.Min.Z)continue;
                var faces=x.Faces;foreach(var plane in y.Planes){faces=PolyhedronClip.Clip(faces,plane);if(faces.Count==0)break;}overlap+=x.Sign*y.Sign*PolyhedronClip.Volume(faces);
            }
            if(Math.Abs(overlap)>cellVolume*1e-8)throw new InvalidOperationException("Fixed solid and ice overlap; reject geometry instead of double-counting accessible volume.");
        }
        public void RequireFeatureResolution(double thicknessM,double minimumCells=3)
        {if(!SiUnits.IsFinite(thicknessM)||thicknessM<=0||!SiUnits.IsFinite(minimumCells)||minimumCells<1)throw new ArgumentException("Feature resolution inputs.");double ratio=thicknessM/Math.Max(Grid.Spacing.X,Math.Max(Grid.Spacing.Y,Grid.Spacing.Z));if(ratio<minimumCells)throw new NotSupportedException("Feature spans only "+ratio+" cells; through-thickness flow/thermal use requires "+minimumCells+" or a separately verified subgrid model.");}
    }
}
