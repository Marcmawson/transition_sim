using System;

namespace PhysicsSim.ThreePhase3D.Materials
{
    // Explicit low-density approximation to Herrmann2021 Eqs5-8,14-20,23,27-32.
    // Contact factors=1, bulk/critical terms=0; A*=B*=1.1. This is not RP1767's
    // complete finite-density model. Kept separate until comparisons establish scope.
    public static class HumidDiluteTransport
    {
        // Physics-ID: MAT-HUMID-DILUTE-VW-CANDIDATE
        // Implements: R016
        // Model: low-density limit of Herrmann2021 modified Vesovic-Wakeham model
        // Sources: D20
        // Source-Locators: Eqs5-8,14-20,23,27-32
        // Inputs-SI: T [K], water mass fraction [kg/kg]
        // Outputs-SI: viscosity [Pa s], thermal conductivity [W/(m K)]
        // State-Ownership: research-only pure evaluation
        // Conservation: explicit mass-to-mole conversion; no species transfer
        // Discretisation: FP64 two-component matrix solve
        // Validity: interaction reduced temperature 1.2-10; project T<=473 K, Y<=10/11
        // Validation: V05 V32
        // Evidence: PENDING
        // Status: IMPLEMENTED_UNVERIFIED
        // Limitations: not wired to production; finite-density contact functions,
        // exact collision-integral ratios and critical enhancement remain unresolved
        public static double[] Reference(double t,double y)
        {
            Guard.Range(t,1.2*276.28,473);Guard.Range(y,0,10.0/11);
            double x=HumidGas.WaterMoleFraction(y),a=1-x,A=1.1,B=1.1;
            double ma=HumidGas.Ma,mw=HumidGas.Mw,sum=ma+mw;
            double muA=DryAirTransport.Viscosity(t,0),muW=WaterTransport.ReferenceViscosityBackground(t,0);
            // Eq32 ordering: component1 water, component2 dry air.
            double log=Math.Log(t/276.28),sigma=.31562*(1+a*x*(.34831934*(x-a)+.23343484*Math.Pow(x-a,2)+1.5288226*Math.Pow(x-a,3)));
            double collision=Math.Exp(.2218816-.5079322*log+.1285776*log*log-.008328165*Math.Pow(log,3)-.002713173*Math.Pow(log,4));
            double muAW=1e-6*.021357*Math.Sqrt(1000*2*ma*mw/sum*t)/(sigma*sigma*collision);
            double cross=a*x*ma*mw/(2*A*muAW*sum*sum);
            double h00=a*a/muA+cross*(20.0/3+4*mw/ma*A),h11=x*x/muW+cross*(20.0/3+4*ma/mw*A),h01=-cross*(20.0/3-4*A);
            double mu=Solve(h00,h11,h01,a,x);
            double ka=3.75*8.3144621/ma*muA,kw=3.75*8.3144621/mw*muW,kaw=3.75*8.3144621/(2*ma*mw/sum)*muAW;
            double pref=a*x/(2*A*kaw*sum*sum);
            double l00=a*a/ka+pref*(7.5*ma*ma+6.25*mw*mw-3*mw*mw*B+4*ma*mw*A);
            double l11=x*x/kw+pref*(7.5*mw*mw+6.25*ma*ma-3*ma*ma*B+4*ma*mw*A);
            double l01=-pref*ma*mw*(13.75-3*B-4*A);
            double k=Solve(l00,l11,l01,a,x)+a*(DryAirTransport.Conductivity(t,0)-ka)/(a+x*ka/kaw)+x*(WaterTransport.ReferenceConductivityBackground(t,0)-kw)/(x+a*kw/kaw);
            return new[]{mu,k};
        }
        private static double Solve(double m00,double m11,double m01,double a,double x)
        {if(x==0)return 1/m00;if(a==0)return 1/m11;double determinant=m00*m11-m01*m01;if(determinant<=0)throw new InvalidOperationException("Transport matrix is not positive definite.");return (a*a*m11+x*x*m00-2*a*x*m01)/determinant;}
    }
}
