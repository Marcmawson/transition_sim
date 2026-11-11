using System;
using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Geometry
{
    public static class SurfacePrimitives
    {
        public static TriangleSurface Transform(TriangleSurface source,RigidFrame frame)
        {var p=new double[source.positionsM.Length];for(int i=0;i<p.Length;i+=3){D3 v=frame.ToWorld(new D3(source.positionsM[i],source.positionsM[i+1],source.positionsM[i+2]));p[i]=v.X;p[i+1]=v.Y;p[i+2]=v.Z;}return new TriangleSurface{positionsM=p,triangles=(int[])source.triangles.Clone()};}
        public static TriangleSurface Box(D3 min,D3 max)
        {
            if(max.X<=min.X||max.Y<=min.Y||max.Z<=min.Z)throw new ArgumentException("Box dimensions must be positive.");
            var p=new D3[8];for(int i=0;i<8;i++)p[i]=new D3((i&1)==0?min.X:max.X,(i&2)==0?min.Y:max.Y,(i&4)==0?min.Z:max.Z);
            int[,] faces={{0,2,6,4},{1,5,7,3},{0,4,5,1},{2,3,7,6},{0,1,3,2},{4,6,7,5}};var points=new List<double>();var triangles=new List<int>();
            for(int i=0;i<6;i++){Add(points,triangles,p[faces[i,0]],p[faces[i,2]],p[faces[i,1]]);Add(points,triangles,p[faces[i,0]],p[faces[i,3]],p[faces[i,2]]);}return new TriangleSurface{positionsM=points.ToArray(),triangles=triangles.ToArray()};
        }
        public static TriangleSurface Cylinder(double radius,double height,int segments)
        {
            if(radius<=0||height<=0||segments<3)throw new ArgumentException("Cylinder dimensions/segments.");var p=new List<double>();var indices=new List<int>();
            for(int i=0;i<segments;i++)
            {
                double a=2*Math.PI*i/segments,b=2*Math.PI*(i+1)/segments;
                D3 x=new D3(radius*Math.Cos(a),-height/2,radius*Math.Sin(a)),y=new D3(radius*Math.Cos(b),-height/2,radius*Math.Sin(b)),u=x+new D3(0,height,0),v=y+new D3(0,height,0);
                Add(p,indices,x,u,v);Add(p,indices,x,v,y);Add(p,indices,new D3(0,-height/2,0),x,y);Add(p,indices,new D3(0,height/2,0),v,u);
            }
            return new TriangleSurface{positionsM=p.ToArray(),triangles=indices.ToArray()};
        }
        public static TriangleSurface Sphere(double radius,int latitude,int longitude)
        {
            if(radius<=0||latitude<3||longitude<4)throw new ArgumentException("Sphere dimensions/subdivisions.");var p=new List<double>();var indices=new List<int>();
            Func<int,int,D3> point=(i,j)=>new D3(radius*Math.Sin(Math.PI*i/latitude)*Math.Cos(2*Math.PI*j/longitude),radius*Math.Cos(Math.PI*i/latitude),radius*Math.Sin(Math.PI*i/latitude)*Math.Sin(2*Math.PI*j/longitude));
            for(int i=0;i<latitude;i++)for(int j=0;j<longitude;j++)
            {D3 a=point(i,j),b=point(i+1,j),c=point(i+1,j+1),d=point(i,j+1);if(i<latitude-1)Outward(p,indices,a,b,c);if(i>0)Outward(p,indices,a,c,d);}
            return new TriangleSurface{positionsM=p.ToArray(),triangles=indices.ToArray()};
        }
        public static TriangleSurface BoxShell(D3 min,D3 max,D3 cavityMin,D3 cavityMax)
        {
            for(int a=0;a<3;a++)if(cavityMin[a]<=min[a]||cavityMax[a]>=max[a]||cavityMax[a]<=cavityMin[a])throw new ArgumentException("Shell cavity must lie strictly inside outer box.");
            var outer=Box(min,max);var inner=Box(cavityMin,cavityMax);var points=new double[outer.positionsM.Length+inner.positionsM.Length];Array.Copy(outer.positionsM,points,outer.positionsM.Length);Array.Copy(inner.positionsM,0,points,outer.positionsM.Length,inner.positionsM.Length);var triangles=new int[outer.triangles.Length+inner.triangles.Length];Array.Copy(outer.triangles,triangles,outer.triangles.Length);int offset=outer.positionsM.Length/3;
            for(int i=0;i<inner.triangles.Length;i+=3){triangles[outer.triangles.Length+i]=offset+inner.triangles[i];triangles[outer.triangles.Length+i+1]=offset+inner.triangles[i+2];triangles[outer.triangles.Length+i+2]=offset+inner.triangles[i+1];}return new TriangleSurface{positionsM=points,triangles=triangles};
        }
        private static void Outward(List<double> p,List<int> indices,D3 a,D3 b,D3 c)
        {if(D3.Dot(D3.Cross(b-a,c-a),a+b+c)<0)Add(p,indices,a,c,b);else Add(p,indices,a,b,c);}
        private static void Add(List<double> p,List<int> indices,D3 a,D3 b,D3 c)
        {foreach(var x in new[]{a,b,c}){indices.Add(p.Count/3);p.Add(x.X);p.Add(x.Y);p.Add(x.Z);}}
        public static double Area(TriangleSurface surface)
        {double area=0;for(int i=0;i<surface.triangles.Length;i+=3){D3[] p=new D3[3];for(int j=0;j<3;j++){int q=surface.triangles[i+j]*3;p[j]=new D3(surface.positionsM[q],surface.positionsM[q+1],surface.positionsM[q+2]);}area+=D3.Cross(p[1]-p[0],p[2]-p[0]).Length*.5;}return area;}
    }
}
