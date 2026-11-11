using System;

namespace PhysicsSim.ThreePhase3D.Core
{
    public readonly struct D3
    {
        public readonly double X,Y,Z;
        public D3(double x,double y,double z){if(!SiUnits.IsFinite(x)||!SiUnits.IsFinite(y)||!SiUnits.IsFinite(z))throw new ArgumentException("Nonfinite geometry coordinate.");X=x;Y=y;Z=z;}
        public double this[int a]=>a==0?X:a==1?Y:a==2?Z:throw new ArgumentOutOfRangeException(nameof(a));
        public double Length=>Math.Sqrt(Dot(this,this));
        public static D3 operator +(D3 a,D3 b)=>new D3(a.X+b.X,a.Y+b.Y,a.Z+b.Z);
        public static D3 operator -(D3 a,D3 b)=>new D3(a.X-b.X,a.Y-b.Y,a.Z-b.Z);
        public static D3 operator -(D3 a)=>new D3(-a.X,-a.Y,-a.Z);
        public static D3 operator *(D3 a,double s)=>new D3(a.X*s,a.Y*s,a.Z*s);
        public static D3 operator /(D3 a,double s)=>a*(1/s);
        public static double Dot(D3 a,D3 b)=>a.X*b.X+a.Y*b.Y+a.Z*b.Z;
        public static D3 Cross(D3 a,D3 b)=>new D3(a.Y*b.Z-a.Z*b.Y,a.Z*b.X-a.X*b.Z,a.X*b.Y-a.Y*b.X);
        public D3 Unit(){double n=Length;if(!SiUnits.IsFinite(n)||n<=0)throw new ArgumentException("Zero or nonrepresentable direction length.");return this/n;}
    }
    public sealed class RigidFrame
    {
        public readonly D3 Origin,U,V,W;
        public static RigidFrame Identity=>new RigidFrame(new D3(),new D3(1,0,0),new D3(0,1,0),new D3(0,0,1));
        public RigidFrame(D3 origin,D3 u,D3 v,D3 w)
        {
            if(Math.Abs(u.Length-1)>1e-12||Math.Abs(v.Length-1)>1e-12||Math.Abs(w.Length-1)>1e-12||Math.Abs(D3.Dot(u,v))>1e-12||Math.Abs(D3.Dot(u,w))>1e-12||Math.Abs(D3.Dot(v,w))>1e-12||D3.Dot(D3.Cross(u,v),w)<1-1e-12)throw new ArgumentException("Frame must be orthonormal and right handed; scale/shear is unsupported.");
            Origin=origin;U=u;V=v;W=w;
        }
        public static RigidFrame Euler(D3 origin,double yaw,double pitch,double roll)
        {
            Func<D3,D3> rotate=p=>{double x=p.X*Math.Cos(roll)-p.Y*Math.Sin(roll),y=p.X*Math.Sin(roll)+p.Y*Math.Cos(roll),z=p.Z;double yy=y*Math.Cos(pitch)-z*Math.Sin(pitch),zz=y*Math.Sin(pitch)+z*Math.Cos(pitch);return new D3(x*Math.Cos(yaw)+zz*Math.Sin(yaw),yy,-x*Math.Sin(yaw)+zz*Math.Cos(yaw));};
            return new RigidFrame(origin,rotate(new D3(1,0,0)),rotate(new D3(0,1,0)),rotate(new D3(0,0,1)));
        }
        public D3 ToWorld(D3 p)=>Origin+U*p.X+V*p.Y+W*p.Z;
        public D3 ToLocal(D3 p){D3 q=p-Origin;return new D3(D3.Dot(q,U),D3.Dot(q,V),D3.Dot(q,W));}
    }
    public enum GridLocation { Cell, XFace, YFace, ZFace }
    public sealed class GridShape
    {
        public readonly int X,Y,Z,Ghost,Count;
        public GridShape(int x,int y,int z,int ghost)
        {if(x<=0||y<=0||z<=0||ghost<1||ghost>8)throw new ArgumentOutOfRangeException("Grid dimensions/ghost width.");X=x;Y=y;Z=z;Ghost=ghost;long n=checked((x+2L*ghost)*(y+2L*ghost)*(z+2L*ghost));if(n>int.MaxValue)throw new ArgumentOutOfRangeException("Grid exceeds signed GPU index capacity.");Count=(int)n;}
        public int Index(int i,int j,int k)
        {if(i< -Ghost||j< -Ghost||k< -Ghost||i>=X+Ghost||j>=Y+Ghost||k>=Z+Ghost)throw new IndexOutOfRangeException("Grid/ghost index outside allocation.");return ((k+Ghost)*(Y+2*Ghost)+j+Ghost)*(X+2*Ghost)+i+Ghost;}
        public void Coordinates(int index,out int i,out int j,out int k)
        {if(index<0||index>=Count)throw new IndexOutOfRangeException();int sx=X+2*Ghost,sy=Y+2*Ghost;i=index%sx-Ghost;j=index/sx%sy-Ghost;k=index/(sx*sy)-Ghost;}
        public bool Interior(int i,int j,int k)=>i>=0&&j>=0&&k>=0&&i<X&&j<Y&&k<Z;
    }
    // Physics-ID: CORE-MAC-GRID-3D
    // Implements: R009 R055 R058
    // Model: Specification.md sections4,9; staggered Cartesian finite volumes
    // Sources: project discrete geometry contract
    // Source-Locators: ImplementationPlan.md S03; Validation.md V06,V35
    // Inputs-SI: cell dimensions [1], spacings [m], rigid frame [m,1]
    // Outputs-SI: cell volume [m3], face areas [m2], bounds/indices
    // State-Ownership: immutable layout shared by geometry and GPU allocations
    // Conservation: one stored shared face per neighbouring cell pair
    // Discretisation: x fastest; separate cell and x/y/z face extents with ghosts
    // Validity: positive finite spacings; right-handed rigid frame; int32 buffer indices
    // Validation: V06 V35
    // Evidence: PENDING
    // Status: IMPLEMENTED_UNVERIFIED
    // Limitations: uniform spatial spacing per axis; no adaptive blocks or field evolution
    public sealed class Grid3D
    {
        public readonly GridShape Cells,XFaces,YFaces,ZFaces;
        public readonly D3 Spacing;
        public readonly RigidFrame Frame;
        public double CellVolume=>Spacing.X*Spacing.Y*Spacing.Z;
        public Grid3D(int nx,int ny,int nz,D3 spacing,RigidFrame frame=null,int ghost=1)
        {if(spacing.X<=0||spacing.Y<=0||spacing.Z<=0)throw new ArgumentOutOfRangeException(nameof(spacing));Spacing=spacing;Frame=frame??RigidFrame.Identity;Cells=new GridShape(nx,ny,nz,ghost);XFaces=new GridShape(checked(nx+1),ny,nz,ghost);YFaces=new GridShape(nx,checked(ny+1),nz,ghost);ZFaces=new GridShape(nx,ny,checked(nz+1),ghost);if(!SiUnits.IsFinite(CellVolume)||CellVolume<=0)throw new ArgumentOutOfRangeException("Cell volume under/overflow.");}
        public GridShape Shape(GridLocation location)=>location==GridLocation.Cell?Cells:location==GridLocation.XFace?XFaces:location==GridLocation.YFace?YFaces:location==GridLocation.ZFace?ZFaces:throw new ArgumentOutOfRangeException(nameof(location));
        public D3 Lower(int i,int j,int k)=>new D3(i*Spacing.X,j*Spacing.Y,k*Spacing.Z);
        public D3 Centre(int i,int j,int k)=>Frame.ToWorld(Lower(i,j,k)+Spacing*.5);
        public double FaceArea(int axis)=>axis==0?Spacing.Y*Spacing.Z:axis==1?Spacing.X*Spacing.Z:axis==2?Spacing.X*Spacing.Y:throw new ArgumentOutOfRangeException(nameof(axis));
    }
}
