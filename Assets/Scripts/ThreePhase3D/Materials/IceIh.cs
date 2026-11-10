using System;
using System.Numerics;

namespace PhysicsSim.ThreePhase3D.Materials
{
    public enum IceConductivityModel { Unspecified, BonalesSlowFrozenCorrected2018, Yen1981PurePolycrystalline }
    public sealed class IceIh : IIceProperties
    {
        public const string Version="S02-IceIh-2";
        public readonly IceConductivityModel ConductivityModel;
        public IceIh(IceConductivityModel model=IceConductivityModel.Unspecified)
        {if(!Enum.IsDefined(typeof(IceConductivityModel),model))throw new ArgumentOutOfRangeException(nameof(model));ConductivityModel=model;}
        // IAPWS R10-06(2009), Eq 1 and Tables 1-4. Values here use J, Pa throughout.
        private static readonly double[] G={-632020.233335886,.655022213658955,-.189369929326131e-7,.339746123271053e-14,-.556464869058991e-21};
        private static readonly Complex T1=new Complex(.0368017112855051,.0510878114959572),T2=new Complex(.337315741065416,.335449415919309);
        private static readonly Complex R1=new Complex(44.7050716285388,65.6876847463481);
        private static readonly Complex[] R2={new Complex(-72.5974574329220,-78.1008427112870),new Complex(-.0000557107698030123,.0000464578634580806),new Complex(.234801409215913e-10,-.285651142904972e-10)};
        private static Complex Basis(Complex z,double tau)=>(z-tau)*Complex.Log(z-tau)+(z+tau)*Complex.Log(z+tau)-2*z*Complex.Log(z)-tau*tau/z;
        private static Complex First(Complex z,double tau)=>-Complex.Log(z-tau)+Complex.Log(z+tau)-2*tau/z;
        private static Complex Second(Complex z,double tau)=>1/(z-tau)+1/(z+tau)-2/z;
        // Physics-ID: MAT-ICE-GIBBS
        // Implements: R014
        // Model: IAPWS R10-06(2009), Eq 1 Tables 1-4
        // Sources: D02; Inputs-SI: T [K], p_abs [Pa]; Outputs-SI: thermodynamic state [SI]
        // State-Ownership: pure evaluation; Conservation: shared IAPWS reference, no shifts
        // Discretisation: analytic complex Gibbs derivatives, FP64
        // Validity: guarded stable Ice Ih; homogeneous reference evaluator is explicitly separate
        // Validation: V04 V32
        // Source-Locators: Official release equations and tables listed above.
        // Inputs-SI: Temperature K, absolute pressure Pa, density kg/m3 or mass fraction kg/kg as applicable.
        // Outputs-SI: Specific thermodynamic state or transport coefficient in SI.
        // Conservation: No transfers; shared IAPWS reference convention.
        // Evidence: PENDING
        // Status: IMPLEMENTED_UNVERIFIED
        // Limitations: S02 remains incomplete; see Docs/ThreePhase3D/S02-Handoff.md.
        public ThermodynamicState ReferenceTp(double t,double p)
        {
            Guard.Range(t,100,273.16);Guard.Range(p,1e-6,208.566e6);
            double tau=t/273.16,q=(p-101325)/611.657,g0=0,gp0=0,gpp0=0;
            Complex r2=0,rp=0,rpp=2*R2[2]/(611.657*611.657);
            for(int i=0;i<5;i++) {g0+=G[i]*Math.Pow(q,i);if(i>0)gp0+=i*G[i]*Math.Pow(q,i-1)/611.657;if(i>1)gpp0+=i*(i-1)*G[i]*Math.Pow(q,i-2)/(611.657*611.657);}
            for(int i=0;i<3;i++){r2+=R2[i]*Math.Pow(q,i);if(i>0)rp+=i*R2[i]*Math.Pow(q,i-1)/611.657;}
            double g=g0+3327.33756492168*t+273.16*(R1*Basis(T1,tau)+r2*Basis(T2,tau)).Real;
            double gt=3327.33756492168+(R1*First(T1,tau)+r2*First(T2,tau)).Real;
            double gp=gp0+273.16*(rp*Basis(T2,tau)).Real;
            double gtt=(R1*Second(T1,tau)+r2*Second(T2,tau)).Real/273.16;
            double gtp=(rp*First(T2,tau)).Real,gpp=gpp0+273.16*(rpp*Basis(T2,tau)).Real;
            double rho=1/gp,cp=-t*gtt,cv=cp+t*gtp*gtp/gpp;
            double ks=(gtp*gtp/gtt-gpp)/gp;
            return new ThermodynamicState(t,p,rho,g-t*gt,g-t*gt-p*gp,-gt,cp,cv,Math.Sqrt(1/(rho*ks)),-gtp/(gp*gp),-gpp/(gp*gp));
        }
        public ThermodynamicState Evaluate(double t,double p)
        {
            Guard.Range(t,200,273.16);Guard.Range(p,611.657,5e6);
            if(t>PhaseEquilibrium.MeltingTemperature(p)+2e-7)throw new ArgumentOutOfRangeException("Ice state is above the melting curve.");
            return ReferenceTp(t,p);
        }
        public double ConductivityWmK(double t)
        {
            Guard.Finite(t);
            // Yen, CRREL Report 81-10 (1981), p15 Eq33 and p16 Table3.
            // Broad regression of pure polycrystalline ice data, not a continuation
            // of the Bonales preparation. Project restricts to 200-273.15 K.
            // No pressure, porosity or crystal-orientation correction is implied.
            if(ConductivityModel==IceConductivityModel.Yen1981PurePolycrystalline)
            {Guard.Range(t,200,273.15);return 9.828*Math.Exp(-.0057*t);}
            if(ConductivityModel!=IceConductivityModel.BonalesSlowFrozenCorrected2018)
                throw new NotSupportedException("MAT011: explicitly select an applicable sourced ice preparation model.");
            // Bonales et al. 2017: slow-frozen ice, -40 to -5 Celsius, 5-minute
            // needle heating and LLSA. 2018 corrigendum replaces interchanged coefficients.
            // No extrapolation to the inherited -50 C initial condition or melting point.
            Guard.Range(t,233.15,268.15);
            return 2.0526-.0176*(t-273.15);
        }
    }
    public static class PhaseEquilibrium
    {
        // IAPWS R14-08(2011) Eq 1/Table 1 and Eq 6/Table 6. Ice Ih branch only.
        public static double MeltingPressure(double t)
        {
            Guard.Range(t,251.165,273.16);double theta=t/273.16;
            return 611.657*(1+1195393.37*(1-Math.Pow(theta,3))+80818.3159*(1-Math.Pow(theta,25.75))+3338.26860*(1-Math.Pow(theta,103.75)));
        }
        public static double MeltingTemperature(double p)
        {
            Guard.Range(p,611.657,208.566e6);double lo=251.165,hi=273.16;
            for(int i=0;i<60;i++){double m=(lo+hi)/2;if(MeltingPressure(m)>p)lo=m;else hi=m;}return (lo+hi)/2;
        }
        public static double SublimationPressure(double t)
        {
            Guard.Range(t,50,273.16);double theta=t/273.16;
            return 611.657*Math.Exp((-21.2144006*Math.Pow(theta,.00333333333)+27.3203819*Math.Pow(theta,1.20666667)-6.10598130*Math.Pow(theta,1.70333333))/theta);
        }
    }
}
