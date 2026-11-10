using System;
using PhysicsSim.ThreePhase3D.Materials;

namespace PhysicsSim.ThreePhase3D.Verification
{
    public sealed partial class MaterialChecks
    {
        private void GasCompletionChecks()
        {
            ExtendedMaterialChecks();
            // Numerical verification: primary printed Table V, not generated expected values.
            // Original density is mol/dm3; transport M=28.9586 g/mol performs conversion.
            double[,] air={{100,0,7.09559,9.35902},{300,0,18.5230,26.3529},
                {100,28,107.923,119.221},{200,10,21.1392,35.3185},
                {300,5,21.3241,32.6062},{132.64,10.4,17.7623,75.6231}};
            double[] muRounding={5.1e-12,5.1e-11,5.1e-10,5.1e-11,5.1e-11,5.1e-11},kRounding={5.1e-9,5.1e-8,5.1e-7,5.1e-8,5.1e-8,5.1e-8};
            for(int i=0;i<air.GetLength(0);i++)
            {
                double rho=air[i,1]*28.9586;
                Compare("D13-TableV-"+i,"mu_air","Pa s","Lemmon Jacobsen 2004 Table V",DryAirTransport.Viscosity(air[i,0],rho),air[i,2]*1e-6,muRounding[i]);
                Compare("D13-TableV-"+i,"k_air","W/(m K)","Lemmon Jacobsen 2004 Table V",DryAirTransport.Conductivity(air[i,0],rho),air[i,3]*.001,kRounding[i]);
            }
            var gas=new HumidGas();
            foreach(var item in new[]{new[]{300.0,100000,.022394202,1.14614216},new[]{400.0,1000000,.174434709,7.93354063}})
            {
                var e=gas.EquilibriumWithLiquid(item[0],item[1]);string id="D06-equilibrium-"+item[0];
                Compare(id,"Y_water","kg/kg","G8-10 Table 13 rounded saturation composition",e.WaterMassFraction,item[2],1e-9);
                Compare(id,"rho_gas","kg/m3","G8-10 Table 13 rounded saturation density",e.Gas.DensityKgM3,item[3],1e-8);
                Compare(id,"mu_water-mu_liquid","J/kg","G8-10 Eq 12 residual",e.ChemicalPotentialResidualJkg,0,1e-6);
                Compare(id,"p_gas-p_input","Pa","density solve residual",e.Gas.PressurePa-item[1],0,1e-6);
                Compare(id,"stable equilibrium accepted","1","V32",gas.Evaluate(item[0],item[1],e.WaterMassFraction)!=null?1:0,1,0);
                Compare(id,"just below saturation accepted","1","V32",gas.Evaluate(item[0],item[1],e.WaterMassFraction*.999)!=null?1:0,1,0);
                Reject(id+"-supersaturation",()=>gas.Evaluate(item[0],item[1],e.WaterMassFraction*1.001));
            }
            // Numerical/physical checks have different tolerances. 5% mu and 10% k are
            // preregistered project model-comparison limits, NOT 1e-4 table lookup limits.
            // These low-temperature measurements were not used in the original 100-150 C shape fit.
            double[,] measuredMu={{293.15,0,18.176},{293.15,.0143,18.150},{293.15,.0214,18.134},
                {303.15,.0171,18.620},{303.15,.0341,18.586},{313.15,.0148,19.111},{313.15,.0456,19.053},
                {323.15,.0251,19.553},{323.15,.0998,19.363},{298.15,.006,18.451},
                {323.15,.019,19.593},{348.15,.055,20.632},{348.15,.193,20.046},{348.15,.317,19.252}};
            for(int i=0;i<measuredMu.GetLength(0);i++)
            {
                double t=measuredMu[i,0],y=HumidGas.WaterMassFraction(measuredMu[i,1]),expected=measuredMu[i,2]*1e-6;
                Compare("D16-held-out-mu-"+i,"mu_mixture","Pa s","Melling 1997 Tables 5-6 reproducing Kestin/Whitelaw and Hochrainer/Munczak measurements",HumidGasTransport.ReferenceViscosity(t,100000,y,gas.ReferenceTp(t,100000,y).DensityKgM3),expected,.05*expected);
            }
            // Composition-shape regression against the data used to select g=0.80 in
            // Melling. This is NOT independent validation of that fitted factor.
            double[] xs={0,.071,.150,.197,.306,.444},ratios={1,1.020,1.035,1.036,1.026,.999};
            double dryK=gas.ConductivityWmK(353.15,100000,0);
            for(int i=0;i<xs.Length;i++) {double y=HumidGas.WaterMassFraction(xs[i]);Compare("D16-k-shape-"+i,"k/k_dry","1","Melling Table 9 calibration-data regression, not independent validation",HumidGasTransport.ReferenceConductivity(353.15,100000,y,gas.ReferenceTp(353.15,100000,y).DensityKgM3)/dryK,ratios[i],.05*ratios[i]);}
            // Independent modern calculated reference, not a new measurement. Record
            // disagreement without changing the historical model's fit/tolerances or
            // counting these observations as passing numerical assertions.
            double[,] modern={{283.15,17.65,.02507},{293.15,18.09,.02582},{303.15,18.53,.02661},
                {313.15,18.98,.02751},{323.15,19.38,.02844},{333.15,19.44,.02903},
                {343.15,18.73,.02867},{353.15,17.23,.02753},{363.15,14.57,.02535}};
            for(int i=0;i<modern.GetLength(0);i++)
            {
                double t=modern[i,0],y=gas.EquilibriumWithLiquid(t,101325).WaterMassFraction;
                var state=gas.ReferenceTp(t,101325,y);
                double[] predicted={HumidGasTransport.ReferenceViscosity(t,101325,y,state.DensityKgM3),HumidGasTransport.ReferenceConductivity(t,101325,y,state.DensityKgM3)},reference={modern[i,1]*1e-6,modern[i,2]};
                for(int j=0;j<2;j++)observations.Add(new MaterialModelObservation {case_id="ASHRAE2025-saturated-"+t,quantity=j==0?"mu":"k",unit=j==0?"Pa s":"W/(m K)",source="ASHRAE 2025 Fundamentals Ch1 Table5 LibHuAirProp/RP1767 calculated reference",expected=reference[j],actual=predicted[j],relative_difference_percent=100*(predicted[j]-reference[j])/reference[j]});
            }
            var dry=gas.Evaluate(300,101325,0);
            Compare("dry-endpoint","mu","Pa s","modern pure endpoint identity",gas.ViscosityPaS(300,101325,0),DryAirTransport.Viscosity(300,dry.DensityKgM3*DryAirTransport.MolarMassKgMol/HumidGas.Ma),1e-18);
            Compare("pure-vapour-endpoint","mu","Pa s","modern pure endpoint identity",gas.ViscosityPaS(400,101325,1),WaterTransport.Viscosity(gas.Evaluate(400,101325,1)),1e-18);
            foreach(double t in new[]{280.0,300,340,370,400,470})
            {
                double maxY=t<370?gas.EquilibriumWithLiquid(t,101325).WaterMassFraction*.9:.9;
                // At 370 K the saturation mole fraction remains below unity.
                if(t==370)maxY=gas.EquilibriumWithLiquid(t,101325).WaterMassFraction*.9;
                for(int i=0;i<4;i++)
                {
                    double y=maxY*i/3;var state=gas.Evaluate(t,101325,y);
                    Compare("mixture-positive-"+t+"-"+i,"mu>0 and k>0","1","reference transport sign",HumidGasTransport.ReferenceViscosity(t,101325,y,state.DensityKgM3)>0&&HumidGasTransport.ReferenceConductivity(t,101325,y,state.DensityKgM3)>0?1:0,1,0);
                    if(y>.10)Reject("unsupported-rich-transport-"+t+"-"+i,()=>gas.ConductivityWmK(t,101325,y));
                    else Compare("lean-transport-"+t+"-"+i,"production k>0","1","bounded production transport",gas.ConductivityWmK(t,101325,y)>0?1:0,1,0);
                    Compare("gas-identity-"+t+"-"+i,"h-u-p/rho","J/kg","G8-10 thermodynamic identity",state.EnthalpyJkg-state.InternalEnergyJkg-101325/state.DensityKgM3,0,2e-6);
                }
            }
            Reject("equilibrium-no-stable-liquid",()=>gas.EquilibriumWithLiquid(400,101325));
            Reject("equilibrium-temperature-limit",()=>gas.EquilibriumWithLiquid(500,1e6));
            Reject("transport-nonfinite",()=>gas.ConductivityWmK(double.NaN,101325,.01));
        }
    }
}
