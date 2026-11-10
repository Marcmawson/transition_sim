using System;

namespace PhysicsSim.ThreePhase3D.Materials
{
    public enum GasDiffusionModel { Massman1998Dilute, MasonMonchickAshrae2025Tabulated }
    public sealed partial class HumidGas : IHumidGasProperties
    {
        public const string Version="S02-SeaAir-3";
        public const double Ma=.02896546, Mw=.018015268;
        internal static Jet2 AirHelmholtz(Jet2 t,Jet2 rho)
        {
            Jet2 tau=132.6312/t,d=rho/(10.4477*28.96546);
            double[] np={.6057194e-7,-.210274769e-4,-.158860716e-3,9.7450251743948,10.0986147428912,-.19536342e-3},tp={-3,-2,-1,0,1,1.5};
            Jet2 phi=Jet2.Log(d)+2.490888032*Jet2.Log(tau);
            for(int i=0;i<np.Length;i++)phi+=np[i]*Jet2.Pow(tau,tp[i]);
            phi+=.791309509*Jet2.Log(1-Jet2.Exp(-25.36365*tau))+.212236768*Jet2.Log(1-Jet2.Exp(-16.90741*tau))-.197938904*Jet2.Log(2.0/3+Jet2.Exp(87.31279*tau));
            double[] n1={.118160747229,.713116392079,-1.61824192067,.0714140178971,-.0865421396646,.134211176704,.0112626704218,-.0420533228842,.0349008431982,.000164957183186},d1={1,1,1,2,3,3,4,4,4,6},t1={0,.33,1.01,0,0,.15,0,.2,.35,1.35};
            double[] n2={-.101365037912,-.173813690970,-.0472103183731,-.0122523554253,-.146629609713,-.0316055879821,.000233594806142,.0148287891978,-.00938782884667},d2={1,3,5,6,1,3,11,1,3},t2={1.6,.8,.95,1.25,3.6,6,3.25,3.5,15},c2={1,1,1,1,2,2,2,3,3};
            for(int i=0;i<n1.Length;i++)phi+=n1[i]*Jet2.Pow(d,d1[i])*Jet2.Pow(tau,t1[i]);
            for(int i=0;i<n2.Length;i++)phi+=n2[i]*Jet2.Pow(d,d2[i])*Jet2.Pow(tau,t2[i])*Jet2.Exp(-Jet2.Pow(d,c2[i]));
            return (8.31451/Ma)*t*phi;
        }
        // Physics-ID: MAT-HUMID-HELMHOLTZ
        // Implements: R015 R016
        // Model: IAPWS G8-10 Tables 5-11 (T11,T45,T55,T58,T61)
        // Sources: D01 D06; Inputs-SI: T [K], rho [kg/m3], Y_water [kg/kg]
        // Outputs-SI: mixture f [J/kg] and T,rho derivatives
        // State-Ownership: pure evaluation; Conservation: water reference unchanged, air G8-10 reference
        // Discretisation: forward FP64 derivatives of full component Helmholtz plus cross virials
        // Validity: reference check evaluator separate from guarded atmospheric provider
        // Validation: V05 V32
        // Source-Locators: Official release equations and tables listed above.
        // Inputs-SI: Temperature K, absolute pressure Pa, density kg/m3 or mass fraction kg/kg as applicable.
        // Conservation: No transfers; shared IAPWS reference convention.
        // Evidence: PENDING
        // Status: IMPLEMENTED_UNVERIFIED
        // Limitations: S02 remains incomplete; see Docs/ThreePhase3D/S02-Handoff.md.
        internal static Jet2 Helmholtz(Jet2 t,Jet2 rho,Jet2 y)
        {
            Jet2 a=1-y;Jet2 result=0;
            if(a.V>0)result+=a*AirHelmholtz(t,a*rho);
            if(y.V>0)result+=y*Water95.Helmholtz(t,y*rho);
            Jet2 theta=t/100;
            Jet2 baw=1e-6*(66.5687*Jet2.Pow(theta,-.237)-238.834*Jet2.Pow(theta,-1.048)-176.755*Jet2.Pow(theta,-3.183));
            Jet2 caaw=1e-6*(.000482737+.00105678/theta-.00656394/Jet2.Pow(theta,2)+.0294442/Jet2.Pow(theta,3)-.0319317/Jet2.Pow(theta,4));
            Jet2 caww=-1e-6*Jet2.Exp(-10.728876+34.7802/theta-38.3383/Jet2.Pow(theta,2)+33.406/Jet2.Pow(theta,3));
            return result+2*a*y*rho*8.314472*t/(Ma*Mw)*(baw+.75*rho*(a/Ma*caaw+y/Mw*caww));
        }
        public ThermodynamicState ReferenceRhoT(double t,double rho,double y)
        {Guard.Range(t,193,473);Guard.Range(rho,1e-8,20);Guard.Range(y,0,1);return Water95.FromHelmholtz(t,rho,Helmholtz(new Jet2(t,1),new Jet2(rho,0,1),y));}
        public static double WaterMoleFraction(double y)
        {Guard.Range(y,0,1);return (y/Mw)/(y/Mw+(1-y)/Ma);}
        public static double WaterMassFraction(double x)
        {Guard.Range(x,0,1);return x*Mw/(x*Mw+(1-x)*Ma);}
        public ThermodynamicState Evaluate(double t,double p,double y)
        {
            Guard.Range(t,200,473);Guard.Range(p,80000,1e6);Guard.Range(y,0,1);
            var gas=ReferenceTp(t,p,y);
            if(t<PhaseEquilibrium.MeltingTemperature(p))
            {
                if(y>0&&WaterChemicalPotential(t,gas.DensityKgM3,y)>new IceIh().Evaluate(t,p).GibbsJkg+2e-5)
                    throw new ArgumentOutOfRangeException("Gas is supersaturated relative to ice Ih.");
                return gas;
            }
            if(t<273.16)throw new NotSupportedException("The narrow near-melting liquid-contact interval needs a supercooled-liquid provider.");
            // G8-10 section 6: stability against liquid is determined by equal chemical
            // potentials, not by an ideal partial-pressure percentage margin.
            var water=new Water95();
            if(y>0 && water.Saturation(t).PressurePa<=p)
            {
                double liquidMu=water.Evaluate(t,p,WaterBranch.Liquid).GibbsJkg;
                if(WaterChemicalPotential(t,gas.DensityKgM3,y)>liquidMu+2e-5)
                    throw new ArgumentOutOfRangeException("Humid gas is supersaturated relative to liquid water at T,p.");
            }
            return gas;
        }
        // Homogeneous check/iteration seam. Does not assert global phase stability.
        public ThermodynamicState ReferenceTp(double t,double p,double y)
        {
            Guard.Range(t,193,473);Guard.Range(p,1,1e6);Guard.Range(y,0,1);
            double rho=p/(t*((1-y)*8.31451/Ma+y*Water95.R));
            for(int i=0;i<20;i++)
            {
                var s=ReferenceRhoT(t,rho,y);double e=s.PressurePa-p;
                if(Math.Abs(e)<1e-6)return s;
                rho-=e*s.DensityPressureDerivative;
                if(rho<=0||rho>20)throw new InvalidOperationException("Humid density iteration left supported bracket.");
            }
            throw new InvalidOperationException("Humid density iteration did not converge.");
        }
        public double ViscosityPaS(double t,double p,double y)
        {var state=Evaluate(t,p,y);return HumidGasTransport.Viscosity(t,p,y,state.DensityKgM3);}
        public double ConductivityWmK(double t,double p,double y)
        {var state=Evaluate(t,p,y);return HumidGasTransport.Conductivity(t,p,y,state.DensityKgM3);}
        public double BinaryDiffusivityM2s(double t,double p,double y)
        {return BinaryDiffusivityM2s(t,p,y,GasDiffusionModel.Massman1998Dilute);}
        public double BinaryDiffusivityM2s(double t,double p,double y,GasDiffusionModel model)
        {
            Evaluate(t,p,y);Guard.Range(p,80000,101325);Guard.Range(y,0,.02);
            if(model==GasDiffusionModel.Massman1998Dilute){Guard.Range(t,273.16,373.15);return ReferenceDiffusivity(t,p);}
            if(model==GasDiffusionModel.MasonMonchickAshrae2025Tabulated)return ReferenceTabulatedDiffusivity(t,p);
            throw new ArgumentOutOfRangeException(nameof(model));
        }
        // Marrero & Mason (1972), Table13 air-H2O, Eq4.3-2: pD=A*T^s.
        // First fit 282-450 K; second 450-1070 K. SI conversion 1 cm2=1e-4 m2.
        // This reference is the source's equimolar correlation; dilute application
        // is a declared project approximation, not composition-resolved validation.
        public static double ReferenceHotDiluteDiffusivity(double t,double p)
        {
            Guard.Range(t,282,473);Guard.Range(p,80000,101325);
            return (t<=450?1.87e-10*Math.Pow(t,2.072):2.75e-9*Math.Pow(t,1.632))*101325/p;
        }
        // ASHRAE2025 Fundamentals Ch1 Table4, calculated Mason-Monchick1965
        // values. Explicit table model, not an inferred fit to the missing equations.
        // Log-log interpolation and dilute-gas inverse-pressure scaling are project
        // choices. No high-composition correction or uncertainty bound is asserted.
        public static double ReferenceTabulatedDiffusivity(double t,double p)
        {
            Guard.Range(t,203.15,473);Guard.Range(p,80000,101325);
            double[] tc={-70,-50,-40,-35,-30,-25,-20,-15,-10,-5,0,5,10,15,20,25,30,35,40,45,50,55,60,70,100,130,160,190,220};
            double[] d={13.2,15.6,16.9,17.5,18.2,18.8,19.5,20.2,20.8,21.5,22.2,22.9,23.6,24.3,25.1,25.8,26.5,27.3,28.0,28.8,29.5,30.3,31.1,32.7,37.6,42.8,48.3,54.0,60.0};
            int i=0;while(i<tc.Length-2&&t>tc[i+1]+273.15)i++;
            double w=Math.Log(t/(tc[i]+273.15))/Math.Log((tc[i+1]+273.15)/(tc[i]+273.15));
            return 1e-6*d[i]*Math.Pow(d[i+1]/d[i],w)*101325/p;
        }
        // Massman 1998, Eq 1, H2O discussion and Table 8: 0.2178 cm2/s at
        // 273.15 K/1 atm, exponent 1.81, stated absolute uncertainty +/-7%.
        // Project envelope is deliberately narrower: dilute, near-atmospheric gas;
        // this is not a validated steam-rich or hot-plume diffusion model.
        public static double ReferenceDiffusivity(double t,double p)
        {Guard.Range(t,253.15,373.15);Guard.Range(p,80000,101325);return 2.178e-5*(101325/p)*Math.Pow(t/273.15,1.81);}
        public double ReferenceWaterChemicalPotential(double t,double rho,double y)
        {
            Guard.Range(t,193,473);Guard.Range(rho,1e-8,20);Guard.Range(y,1e-12,1-1e-12);
            Jet2 composition=Helmholtz(t,rho,new Jet2(y,1)),density=Helmholtz(t,new Jet2(rho,0,1),y);
            return composition.V+rho*density.Y+(1-y)*composition.X;
        }
        private double WaterChemicalPotential(double t,double rho,double y)
        {return y==1?ReferenceRhoT(t,rho,1).GibbsJkg:ReferenceWaterChemicalPotential(t,rho,y);}

        // Physics-ID: MAT-HUMID-LIQUID-EQUILIBRIUM
        // Implements: R013 R015
        // Model: IAPWS G8-10 Eq 12; Table 12 T66 and Table 13
        // Sources: D01 D06
        // Source-Locators: G8-10 Eq 12 and section 6; R6-95(2018) Table 3
        // Inputs-SI: temperature [K], total absolute pressure [Pa]
        // Outputs-SI: water mass fraction [kg/kg], gas/liquid state [SI], chemical residual [J/kg]
        // State-Ownership: pure reference solve; no phase mass transfer or time integration
        // Conservation: phase energies retain the common IAPWS reference
        // Discretisation: bracketed composition bisection with converged density inversion
        // Validity: 273.16 <= T <= 473 K; 80 kPa <= p <= 1 MPa; stable liquid must exist
        // Validation: V05 V32
        // Evidence: PENDING
        // Status: IMPLEMENTED_UNVERIFIED
        // Limitations: liquid contact only; no ice-contact humid equilibrium or interfacial kinetic law
        public GasLiquidEquilibrium EquilibriumWithLiquid(double t,double p)
        {
            Guard.Range(t,273.16,473);Guard.Range(p,80000,1e6);
            var water=new Water95();var sat=water.Saturation(t);
            if(p<=sat.PressurePa)throw new ArgumentOutOfRangeException("A humid liquid-contact equilibrium requires total pressure above pure-water saturation.");
            var liquid=water.Evaluate(t,p,WaterBranch.Liquid);
            double lo=1e-12,hi=WaterMassFraction(Math.Min(1-1e-12,1.15*sat.PressurePa/p));
            double flo=ChemicalPotentialResidual(t,p,lo,liquid.GibbsJkg),fhi=ChemicalPotentialResidual(t,p,hi,liquid.GibbsJkg);
            if(flo>=0||fhi<=0)throw new InvalidOperationException("Chemical-equilibrium root is not bracketed in the supported gas neighbourhood.");
            for(int i=0;i<80;i++)
            {
                double y=(lo+hi)/2;var gas=ReferenceTp(t,p,y);
                double residual=WaterChemicalPotential(t,gas.DensityKgM3,y)-liquid.GibbsJkg;
                if(Math.Abs(residual)<=1e-6)return new GasLiquidEquilibrium(t,p,y,liquid,gas,residual);
                if(residual>0)hi=y;else lo=y;
            }
            throw new InvalidOperationException("Humid equilibrium did not converge to the chemical-potential tolerance.");
        }
        private double ChemicalPotentialResidual(double t,double p,double y,double liquidMu)
        {var gas=ReferenceTp(t,p,y);return WaterChemicalPotential(t,gas.DensityKgM3,y)-liquidMu;}
    }

    public sealed class GasLiquidEquilibrium
    {
        public readonly double TemperatureK,PressurePa,WaterMassFraction,ChemicalPotentialResidualJkg;
        public readonly ThermodynamicState Liquid,Gas;
        internal GasLiquidEquilibrium(double t,double p,double y,ThermodynamicState liquid,ThermodynamicState gas,double residual)
        {TemperatureK=t;PressurePa=p;WaterMassFraction=y;Liquid=liquid;Gas=gas;ChemicalPotentialResidualJkg=residual;}
    }

    // Product data are deliberately not represented by a guessed cp curve.
    public sealed class DuranDataGap : ISolidProperties
    {
        public string MaterialId=>"DURAN-MAT012-014-INCOMPLETE";
        public double DensityKgM3(double t) {Guard.Range(t,298.15,298.15);return 2230;}
        public double ConductivityWmK(double t) {Guard.Range(t,363.15,363.15);return 1.2;}
        public double CpJkgK(double t) {Guard.Finite(t);throw new NotSupportedException("MAT014 DURAN heat capacity curve is missing.");}
        public double EnthalpyJkg(double t) {return CpJkgK(t);}
        public double EntropyJkgK(double t) {return CpJkgK(t);}
    }
}
