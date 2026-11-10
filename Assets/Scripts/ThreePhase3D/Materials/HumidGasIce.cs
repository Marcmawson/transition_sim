using System;

namespace PhysicsSim.ThreePhase3D.Materials
{
    public sealed partial class HumidGas
    {
        // G8-10 chemical potential converted to fugacity with its own ideal-gas
        // reference. The residual at the 1e-12 kg/m3 ideal-limit anchor is negligible
        // at this check envelope; this is a separately named comparison seam.
        public double ReferenceHelmholtzFugacity(double t,double p,double y)
        {
            Guard.Range(t,200,473);Guard.Range(y,1e-12,1-1e-12);
            var gas=ReferenceTp(t,p,y);double xp=WaterMoleFraction(y)*p;
            double ideal=Water95.Helmholtz(t,1e-12).V+Water95.R*t+Water95.R*t*Math.Log(xp/(Water95.R*t*1e-12));
            double mu=ReferenceWaterChemicalPotential(t,gas.DensityKgM3,y);
            return xp*Math.Exp((mu-ideal)/(Water95.R*t));
        }
        // Physics-ID: MAT-HUMID-ICE-EQUILIBRIUM
        // Implements: R013 R015
        // Model: G8-10 Eq12 with R10-06 ice Gibbs energy
        // Sources: D02 D03 D06
        // Source-Locators: G8-10 Table13, ice-contact 200 K state
        // Inputs-SI: T [K], absolute p [Pa]
        // Outputs-SI: Y [kg/kg], states, mu residual [J/kg]
        // State-Ownership: pure solve
        // Conservation: common IAPWS reference, no transfer
        // Discretisation: bracketed bisection and FP64 density inversion
        // Validity: 200 K to pressure-dependent ice melting, 80 kPa to 1 MPa
        // Validation: V05 V32
        // Evidence: PENDING
        // Status: IMPLEMENTED_UNVERIFIED
        // Limitations: equilibrium only; no deposition kinetics or porous ice model
        public GasIceEquilibrium EquilibriumWithIce(double t,double p)
        {
            Guard.Range(t,200,273.16);Guard.Range(p,80000,1e6);
            var ice=new IceIh().Evaluate(t,p);
            double lo=1e-12,hi=WaterMassFraction(Math.Min(.99,2*PhaseEquilibrium.SublimationPressure(t)/p));
            if(ChemicalPotentialResidual(t,p,lo,ice.GibbsJkg)>=0||ChemicalPotentialResidual(t,p,hi,ice.GibbsJkg)<=0)
                throw new InvalidOperationException("Ice-contact composition root is not bracketed.");
            for(int i=0;i<90;i++)
            {
                double y=(lo+hi)/2;var gas=ReferenceTp(t,p,y);
                double residual=WaterChemicalPotential(t,gas.DensityKgM3,y)-ice.GibbsJkg;
                if(Math.Abs(residual)<=1e-6)return new GasIceEquilibrium(t,p,y,ice,gas,residual);
                if(residual>0)hi=y;else lo=y;
            }
            throw new InvalidOperationException("Ice-contact equilibrium did not converge.");
        }
    }
    public sealed class GasIceEquilibrium
    {
        public readonly double TemperatureK,PressurePa,WaterMassFraction,ChemicalPotentialResidualJkg;
        public readonly ThermodynamicState Ice,Gas;
        internal GasIceEquilibrium(double t,double p,double y,ThermodynamicState ice,ThermodynamicState gas,double residual)
        {TemperatureK=t;PressurePa=p;WaterMassFraction=y;Ice=ice;Gas=gas;ChemicalPotentialResidualJkg=residual;}
    }
}
