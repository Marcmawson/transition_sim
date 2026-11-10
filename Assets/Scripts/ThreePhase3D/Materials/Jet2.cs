using System;

namespace PhysicsSim.ThreePhase3D.Materials
{
    // Second-order forward differentiation in two independent SI variables.
    // X,Y are first derivatives; XX,XY,YY are derivatives (not Taylor coefficients).
    internal readonly struct Jet2
    {
        public readonly double V,X,Y,XX,XY,YY;
        public Jet2(double v,double x=0,double y=0,double xx=0,double xy=0,double yy=0)
        {V=v;X=x;Y=y;XX=xx;XY=xy;YY=yy;}
        public static implicit operator Jet2(double v)=>new Jet2(v);
        public static Jet2 operator +(Jet2 a,Jet2 b)=>new Jet2(a.V+b.V,a.X+b.X,a.Y+b.Y,a.XX+b.XX,a.XY+b.XY,a.YY+b.YY);
        public static Jet2 operator -(Jet2 a)=>new Jet2(-a.V,-a.X,-a.Y,-a.XX,-a.XY,-a.YY);
        public static Jet2 operator -(Jet2 a,Jet2 b)=>a+-b;
        public static Jet2 operator *(Jet2 a,Jet2 b)=>new Jet2(a.V*b.V,a.X*b.V+a.V*b.X,a.Y*b.V+a.V*b.Y,
            a.XX*b.V+2*a.X*b.X+a.V*b.XX,a.XY*b.V+a.X*b.Y+a.Y*b.X+a.V*b.XY,a.YY*b.V+2*a.Y*b.Y+a.V*b.YY);
        public static Jet2 operator /(Jet2 a,Jet2 b)=>a*Pow(b,-1);
        private static Jet2 Chain(Jet2 a,double f,double d,double dd)=>new Jet2(f,d*a.X,d*a.Y,dd*a.X*a.X+d*a.XX,dd*a.X*a.Y+d*a.XY,dd*a.Y*a.Y+d*a.YY);
        public static Jet2 Pow(Jet2 a,double b)
        {
            if(b==0)return 1;
            if(b==1)return a;
            // All noninteger zero-base powers used here have b>2. No critical-point extension is inferred.
            if(a.V==0 && (b>2 || (b>1 && a.X==0 && a.Y==0)))return 0;
            return Chain(a,Math.Pow(a.V,b),b*Math.Pow(a.V,b-1),b*(b-1)*Math.Pow(a.V,b-2));
        }
        public static Jet2 Exp(Jet2 a) {double e=Math.Exp(a.V);return Chain(a,e,e,e);}
        public static Jet2 Log(Jet2 a)=>Chain(a,Math.Log(a.V),1/a.V,-1/(a.V*a.V));
    }
}
