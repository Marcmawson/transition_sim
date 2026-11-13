using System;
using System.Collections.Generic;
using System.Linq;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Geometry
{
    internal readonly struct ClipPlane
    {
        public readonly D3 N;public readonly double Offset;
        public ClipPlane(D3 normal,D3 point){N=normal.Unit();Offset=D3.Dot(N,point);}
        public double Distance(D3 point)=>D3.Dot(N,point)-Offset;
    }
    internal static class PolyhedronClip
    {
        internal static List<D3> Polygon(IList<D3> source,ClipPlane plane,List<D3> crossings=null)
        {
            var result=new List<D3>();if(source.Count==0)return result;
            D3 a=source[source.Count-1];double da=plane.Distance(a);
            foreach(D3 b in source)
            {
                double db=plane.Distance(b);
                if((da<=0)!=(db<=0)){D3 p=a+(b-a)*(da/(da-db));result.Add(p);crossings?.Add(p);}
                if(db<=0)result.Add(b);a=b;da=db;
            }
            return result;
        }
        internal static List<List<D3>> Clip(List<List<D3>> faces,ClipPlane plane) => Clip(faces, plane, false, out _);
        internal static double SectionArea(List<List<D3>> faces, ClipPlane plane)
        {
            Clip(faces, plane, true, out double area);
            return area;
        }
        internal static List<D3> SectionPolygon(List<List<D3>> faces, ClipPlane plane)
        {
            var clipped = Clip(faces, plane, true, out double area);
            return area > 0 ? clipped[clipped.Count - 1] : new List<D3>();
        }

        private static List<List<D3>> Clip(List<List<D3>> faces,ClipPlane plane,bool measureSection,out double sectionArea)
        {
            var result=new List<List<D3>>();var cuts=new List<D3>();
            foreach(var face in faces){var polygon=Polygon(face,plane,cuts);if(polygon.Count>=3)result.Add(polygon);}
            var unique=new List<D3>();foreach(var p in cuts)if(!unique.Any(q=>(p-q).Length<1e-13))unique.Add(p);
            sectionArea = 0;
            if(unique.Count>=3)
            {
                D3 centre=new D3();foreach(var p in unique)centre+=p;centre/=unique.Count;
                D3 axis=Math.Abs(plane.N.X)<.8?new D3(1,0,0):new D3(0,1,0),u=D3.Cross(axis,plane.N).Unit(),v=D3.Cross(plane.N,u);
                unique.Sort((a,b)=>Math.Atan2(D3.Dot(a-centre,v),D3.Dot(a-centre,u)).CompareTo(Math.Atan2(D3.Dot(b-centre,v),D3.Dot(b-centre,u))));result.Add(unique);
                if (measureSection) sectionArea = Area(unique);
            }
            return result;
        }
        internal static ClipPlane[] Bounds(D3 lo,D3 hi)=>new[]{new ClipPlane(new D3(-1,0,0),lo),new ClipPlane(new D3(1,0,0),hi),new ClipPlane(new D3(0,-1,0),lo),new ClipPlane(new D3(0,1,0),hi),new ClipPlane(new D3(0,0,-1),lo),new ClipPlane(new D3(0,0,1),hi)};
        internal static double Volume(List<List<D3>> faces)
        {
            if(faces.Count==0)return 0;D3 origin=faces[0][0];double volume=0;
            foreach(var f in faces)for(int i=1;i+1<f.Count;i++)volume+=D3.Dot(f[0]-origin,D3.Cross(f[i]-origin,f[i+1]-origin))/6;
            return Math.Abs(volume);
        }
        internal static D3 VolumeCentroid(List<List<D3>> faces)
        {
            D3 origin=faces[0][0], moment=new D3();
            double volume=0;
            foreach(var face in faces)
            for(int i=1;i+1<face.Count;i++)
            {
                D3 a=face[0]-origin,b=face[i]-origin,c=face[i+1]-origin;
                double weight=D3.Dot(a,D3.Cross(b,c))/6;
                moment+=(a+b+c)*(weight*.25);
                volume+=weight;
            }
            return volume==0?origin:origin+moment/volume;
        }

        internal static double Area(IList<D3> polygon)
        {if(polygon.Count<3)return 0;D3 sum=new D3();for(int i=1;i+1<polygon.Count;i++)sum+=D3.Cross(polygon[i]-polygon[0],polygon[i+1]-polygon[0]);return sum.Length*.5;}
        internal static D3 Centroid(IList<D3> polygon)
        {D3 moment=new D3();double area=0;for(int i=1;i+1<polygon.Count;i++){double a=D3.Cross(polygon[i]-polygon[0],polygon[i+1]-polygon[0]).Length*.5;area+=a;moment+=(polygon[0]+polygon[i]+polygon[i+1])*(a/3);}return area>0?moment/area:polygon[0];}
    }
    internal sealed class SignedTetrahedron
    {
        internal readonly List<List<D3>> Faces=new List<List<D3>>();
        internal readonly List<ClipPlane> Planes=new List<ClipPlane>();
        internal readonly D3 Min,Max;internal readonly double Sign;
        internal SignedTetrahedron(D3 o,D3 a,D3 b,D3 c)
        {
            var p=new[]{o,a,b,c};double signed=D3.Dot(a-o,D3.Cross(b-o,c-o));Sign=Math.Sign(signed);
            Min=new D3(p.Min(x=>x.X),p.Min(x=>x.Y),p.Min(x=>x.Z));Max=new D3(p.Max(x=>x.X),p.Max(x=>x.Y),p.Max(x=>x.Z));
            if(Sign==0)return;
            int[,] indices={{0,1,2,3},{0,3,1,2},{0,2,3,1},{1,3,2,0}};
            for(int i=0;i<4;i++)
            {D3 x=p[indices[i,0]],y=p[indices[i,1]],z=p[indices[i,2]],other=p[indices[i,3]];D3 n=D3.Cross(y-x,z-x);if(D3.Dot(n,other-x)>0){D3 temp=y;y=z;z=temp;n=-n;}Faces.Add(new List<D3>{x,y,z});Planes.Add(new ClipPlane(n,x));}
        }
        internal double InBox(D3 lo,D3 hi) => InBox(lo,hi,false,out _);
        internal double InBox(D3 lo,D3 hi,bool needCentroid,out D3 centroid)
        {
            centroid=(lo+hi)*.5;
            D3 centre=(lo+hi)*.5,half=(hi-lo)*.5;bool inside=true;
            foreach(var plane in Planes){double d=plane.Distance(centre),r=Math.Abs(plane.N.X)*half.X+Math.Abs(plane.N.Y)*half.Y+Math.Abs(plane.N.Z)*half.Z;if(d-r>=0)return 0;if(d+r>0)inside=false;}
            if(inside)return Sign*8*half.X*half.Y*half.Z;
            var faces=Faces;foreach(var plane in PolyhedronClip.Bounds(lo,hi)){faces=PolyhedronClip.Clip(faces,plane);if(faces.Count==0)return 0;}if(needCentroid)centroid=PolyhedronClip.VolumeCentroid(faces);return Sign*PolyhedronClip.Volume(faces);
        }
        internal double OnFace(int axis,double coordinate,D3 lo,D3 hi)
        {
            int u=(axis+1)%3,v=(axis+2)%3;Func<double,double,D3> point=(a,b)=>axis==0?new D3(coordinate,a,b):axis==1?new D3(b,coordinate,a):new D3(a,b,coordinate);
            var polygon=new List<D3>{point(lo[u],lo[v]),point(hi[u],lo[v]),point(hi[u],hi[v]),point(lo[u],hi[v])};
            foreach(var plane in Planes){polygon=PolyhedronClip.Polygon(polygon,plane);if(polygon.Count<3)return 0;}return Sign*PolyhedronClip.Area(polygon);
        }
    }
}
