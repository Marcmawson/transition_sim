using System;

namespace PhysicsSim.ThreePhase3D.Materials
{
    public static class DryAirTransport
    {
        public const double MolarMassKgMol=.0289586,Tc=132.6312,Rhoc=10.4477*28.9586;
        // Physics-ID: MAT-AIR-TRANSPORT
        // Implements: R016
        // Model: Lemmon and Jacobsen 2004 Eqs 1-11, Tables I-V
        // Sources: D13
        // Source-Locators: trc.nist.gov/refprop/FAQ/NAO.PDF printed pp 23-28
        // Inputs-SI: T [K], dry-air mass density [kg/m3] using M=0.0289586 kg/mol
        // Outputs-SI: viscosity [Pa s], conductivity [W/(m K)]
        // State-Ownership: pure evaluation; no mutable thermodynamic inventory
        // Conservation: mass-basis conversion preserves molar density across the G8-10 air EOS
        // Discretisation: FP64 correlation including conductivity enhancement
        // Validity: homogeneous reference checks; production mixture restricts T and pressure further
        // Validation: V05 V32
        // Evidence: PENDING
        // Status: IMPLEMENTED_UNVERIFIED
        // Limitations: not a mixture rule and not a global dry-air phase selector
        public static double Viscosity(double t,double rho)
        {
            Guard.Range(t,100,1000);Guard.Range(rho,0,1000);
            double tr=Tc/t,d=rho/Rhoc;
            double[] n={10.72,1.122,.002019,-8.876,-.02916},tt={.2,.05,2.4,.6,3.6},dd={1,4,9,1,8},ll={0,0,0,1,1};
            double residual=0;for(int i=0;i<n.Length;i++)residual+=n[i]*Math.Pow(tr,tt[i])*Math.Pow(d,dd[i])*(ll[i]==0?1:Math.Exp(-Math.Pow(d,ll[i])));
            return 1e-6*(DiluteViscosityMicroPaS(t)+residual);
        }
        private static double DiluteViscosityMicroPaS(double t)
        {
            double x=Math.Log(t/103.3);
            double omega=Math.Exp(.431-.4623*x+.08406*x*x+.005341*x*x*x-.00331*x*x*x*x);
            return .0266958*Math.Sqrt(28.9586*t)/(.36*.36*omega);
        }
        private static ThermodynamicState State(double t,double rho)
        {
            double conversion=HumidGas.Ma/MolarMassKgMol;
            Jet2 f=conversion*HumidGas.AirHelmholtz(new Jet2(t,1),new Jet2(rho,0,1)*conversion);
            return Water95.FromHelmholtz(t,rho,f);
        }
        public static double Conductivity(double t,double rho)
        {
            Guard.Range(t,100,1000);Guard.Range(rho,0,1000);
            double tr=Tc/t,d=rho/Rhoc;
            double background=1.308*DiluteViscosityMicroPaS(t)+1.405*Math.Pow(tr,-1.1)-1.036*Math.Pow(tr,-.3);
            double[] n={8.743,14.76,-16.62,3.793,-6.142,-.3778},tt={.1,0,.5,2.7,.3,1.3},dd={1,2,3,7,7,11},ll={0,0,2,2,2,2};
            for(int i=0;i<n.Length;i++)background+=n[i]*Math.Pow(tr,tt[i])*Math.Pow(d,dd[i])*(ll[i]==0?1:Math.Exp(-Math.Pow(d,ll[i])));
            if(rho==0)return .001*background;
            var state=State(t,rho);var reference=State(265.262,rho);
            double bracket=3.78502e6*rho/(Rhoc*Rhoc)*(state.DensityPressureDerivative-reference.DensityPressureDerivative*265.262/t);
            double enhancement=0;
            if(bracket>0)
            {
                double xi=.11*Math.Pow(bracket/.055,.63/1.2415),q=xi/.31;
                double omega=2/Math.PI*((1-state.CvJkgK/state.CpJkgK)*Math.Atan(q)+state.CvJkgK/state.CpJkgK*q);
                double omega0=2/Math.PI*(1-Math.Exp(-1/(1/q+q*q/(3*d*d))));
                enhancement=rho*state.CpJkgK*1.380658e-23*1.01*t/(6*Math.PI*xi*1e-9*Viscosity(t,rho))*(omega-omega0);
            }
            return .001*background+enhancement;
        }
    }

    public static class HumidGasTransport
    {
        public const string Version="S02-Melling-shape-modern-endpoints-1";
        // Physics-ID: MAT-HUMID-TRANSPORT
        // Implements: R016
        // Model: Melling et al. 1997 Eqs 13-16, g=0.80; modern component endpoints
        // Sources: D04 D12 D13 D16
        // Source-Locators: doi:10.1063/1.555998 Eq 13 Table 8 and Eqs 14-16
        // Inputs-SI: T [K], total p [Pa], water mass fraction [kg/kg], mixture density [kg/m3]
        // Outputs-SI: dynamic viscosity [Pa s], conductivity [W/(m K)]
        // State-Ownership: pure property evaluation; no species/energy transfer
        // Conservation: convert mass to mole fractions explicitly; no density substitution
        // Discretisation: FP64 algebra; Melling viscosity curvature rebased to modern pure-component values
        // Validity: production 273.16-473 K, 80-120 kPa, Y<=0.10 or pure vapour; wider reference comparisons are separately named
        // Validation: V05 V32
        // Evidence: PENDING
        // Status: IMPLEMENTED_UNVERIFIED
        // Limitations: low-temperature extension is project verification against reference data, not original-paper endorsement; mixture uncertainty exceeds lookup error
        public static double Viscosity(double t,double p,double y,double rho)
        {
            Validate(t,p,y,rho);
            ValidateProductionComposition(y);
            if(y==1)return WaterTransport.Viscosity(new Water95().ReferenceRhoT(t,rho));
            return ReferenceViscosity(t,p,y,rho);
        }
        // Explicit research comparison seam: does not certify a steam-rich mixture.
        public static double ReferenceViscosity(double t,double p,double y,double rho)
        {
            Validate(t,p,y,rho);double x=HumidGas.WaterMoleFraction(y);
            double air=DryAirTransport.Viscosity(t,(1-y)*rho*DryAirTransport.MolarMassKgMol/HumidGas.Ma);
            double vapour=WaterTransport.ReferenceViscosityBackground(t,y*rho);
            double c=-.67799257-.011338714*t;
            // Eq 13 = linear endpoint interpolation + C(T)*(x^2-x).
            // Retain its measured curvature while replacing the historical endpoints.
            return (1-x)*air+x*vapour+1e-6*c*(x*x-x);
        }
        public static double Conductivity(double t,double p,double y,double rho)
        {
            Validate(t,p,y,rho);
            ValidateProductionComposition(y);
            if(y==1)return WaterTransport.Conductivity(new Water95().ReferenceRhoT(t,rho));
            return ReferenceConductivity(t,p,y,rho);
        }
        public static double ReferenceConductivity(double t,double p,double y,double rho)
        {
            Validate(t,p,y,rho);double x=HumidGas.WaterMoleFraction(y);
            double rhoAir=(1-y)*rho*DryAirTransport.MolarMassKgMol/HumidGas.Ma;
            double muA=DryAirTransport.Viscosity(t,rhoAir),muV=WaterTransport.ReferenceViscosityBackground(t,y*rho);
            double kA=DryAirTransport.Conductivity(t,rhoAir),kV=WaterTransport.ReferenceConductivityBackground(t,y*rho);
            double av=Interaction(muA,muV,HumidGas.Ma,HumidGas.Mw),va=Interaction(muV,muA,HumidGas.Mw,HumidGas.Ma);
            return (1-x)*kA/(1-x+av*x)+x*kV/(x+va*(1-x));
        }
        private static double Interaction(double muI,double muJ,double massI,double massJ)
        {double q=1+Math.Sqrt(muI/muJ)*Math.Pow(massJ/massI,.25);return .80*q*q/Math.Sqrt(8*(1+massI/massJ));}
        private static void Validate(double t,double p,double y,double rho)
        {Guard.Range(t,273.16,473);Guard.Range(p,80000,120000);Guard.Range(y,0,1);Guard.Range(rho,1e-8,2);}
        private static void ValidateProductionComposition(double y)
        {
            Guard.Range(y,0,1);
            if(y>.10&&y!=1)throw new NotSupportedException("Steam-rich humid-mixture transport needs a better verified model: the historical approximation disagrees with modern references. Supported production Y<=0.10 or pure vapour.");
        }
    }
}
