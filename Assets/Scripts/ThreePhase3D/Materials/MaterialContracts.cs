using System;

namespace PhysicsSim.ThreePhase3D.Materials
{
    public enum WaterBranch { Liquid, Vapour }

    // All energies are specific, per kg of the named material/mixture, in SI.
    // Water and ice share IAPWS's liquid triple-point u=s=0 convention.
    // Gas composition is water MASS fraction, never relative humidity.
    public sealed class ThermodynamicState
    {
        public readonly double TemperatureK, PressurePa, DensityKgM3, EnthalpyJkg,
            InternalEnergyJkg, EntropyJkgK, CpJkgK, CvJkgK, SoundSpeedMs,
            DensityTemperatureDerivative, DensityPressureDerivative;
        public double GibbsJkg => EnthalpyJkg - TemperatureK * EntropyJkgK;
        public ThermodynamicState(double t, double p, double rho, double h, double u,
            double s, double cp, double cv, double sound, double rhoT, double rhoP)
        {
            TemperatureK=t; PressurePa=p; DensityKgM3=rho; EnthalpyJkg=h;
            InternalEnergyJkg=u; EntropyJkgK=s; CpJkgK=cp; CvJkgK=cv;
            SoundSpeedMs=sound; DensityTemperatureDerivative=rhoT; DensityPressureDerivative=rhoP;
            foreach (double v in new[] {t,p,rho,h,u,s,cp,cv,sound,rhoT,rhoP}) Guard.Finite(v);
            if(t<=0 || p<=0 || rho<=0 || cp<=0 || cv<=0 || sound<=0 || rhoP<=0)
                throw new ArgumentOutOfRangeException("Unstable/nonphysical thermodynamic state.");
        }
    }
    public sealed class SaturationState
    {
        public readonly ThermodynamicState Liquid, Vapour;
        public double PressurePa => (Liquid.PressurePa + Vapour.PressurePa)/2;
        public double LatentEnthalpyJkg => Vapour.EnthalpyJkg-Liquid.EnthalpyJkg;
        public SaturationState(ThermodynamicState liquid, ThermodynamicState vapour) {Liquid=liquid;Vapour=vapour;}
    }
    public interface IWaterProperties
    {
        ThermodynamicState Evaluate(double temperatureK, double absolutePressurePa, WaterBranch branch);
        SaturationState Saturation(double temperatureK);
        double ViscosityPaS(ThermodynamicState state);
        double ConductivityWmK(ThermodynamicState state);
        double SurfaceTensionNm(double temperatureK);
    }
    public interface IIceProperties
    {
        ThermodynamicState Evaluate(double temperatureK, double absolutePressurePa);
        double ConductivityWmK(double temperatureK);
    }
    public interface IHumidGasProperties
    {
        ThermodynamicState Evaluate(double temperatureK, double absolutePressurePa, double waterMassFraction);
        GasLiquidEquilibrium EquilibriumWithLiquid(double temperatureK, double absolutePressurePa);
        GasIceEquilibrium EquilibriumWithIce(double temperatureK, double absolutePressurePa);
        double ViscosityPaS(double temperatureK, double absolutePressurePa, double waterMassFraction);
        double ConductivityWmK(double temperatureK, double absolutePressurePa, double waterMassFraction);
        double BinaryDiffusivityM2s(double temperatureK, double absolutePressurePa, double waterMassFraction);
    }
    public interface ISolidProperties
    {
        string MaterialId {get;}
        double DensityKgM3(double temperatureK);
        double EnthalpyJkg(double temperatureK);
        double EntropyJkgK(double temperatureK);
        double CpJkgK(double temperatureK);
        double ConductivityWmK(double temperatureK);
    }
    internal static class Guard
    {
        public static void Finite(double x) {if(double.IsNaN(x)||double.IsInfinity(x))throw new ArgumentOutOfRangeException("Nonfinite material input/output.");}
        public static void Range(double x,double lo,double hi) {Finite(x);if(x<lo||x>hi)throw new ArgumentOutOfRangeException("Material state outside ["+lo+", "+hi+"].");}
    }
}
