using System;
using PhysicsSim.ThreePhase3D.Materials;

namespace PhysicsSim.ThreePhase3D.Verification
{
    public sealed partial class MaterialChecks
    {
        private void SurfaceChecks()
        {
            foreach(MaterialSurface surface in Enum.GetValues(typeof(MaterialSurface)))
            {
                var d=MaterialSurfaceTable.Generate(surface);surfaces.Add(d);var table=new MaterialSurfaceTable(d);
                for(int i=0;i<43;i++)
                {
                    float t=(float)(d.minimum_k+(d.temperature_count-1)*d.step_k*(i+.371)/43),p=(float)(80000+40000*((i*17)%43+.231)/43);
                    var actual=table.Evaluate(t,p,(float)d.water_mass_fraction);var expected=MaterialSurfaceTable.Reference(surface,t,p,d.water_mass_fraction);
                    for(int j=0;j<table.Width;j++)Compare("FP32-TP-"+surface+"-"+i,d.quantities[j],d.units[j],"Official-check-verified CPU oracle; 1e-4 table target",actual[j],expected[j],TableTolerance(j,expected[j]));
                }
                Reject("TP-outside-pressure-"+surface,()=>table.Evaluate((float)d.minimum_k,79999,(float)d.water_mass_fraction));
                Reject("TP-wrong-composition-"+surface,()=>table.Evaluate((float)d.minimum_k,101325,(float)d.water_mass_fraction+.001f));
                Reject("TP-NaN-"+surface,()=>table.Evaluate(float.NaN,101325,(float)d.water_mass_fraction));
                float saved=d.values[0];d.values[0]=float.PositiveInfinity;Reject("TP-nonfinite-table-"+surface,()=>new MaterialSurfaceTable(d));d.values[0]=saved;
                d.units[0]="g/cm3";Reject("TP-units-"+surface,()=>new MaterialSurfaceTable(d));
            }
        }
        private void ExtendedMaterialChecks()
        {
            double[,] printed={{200,-.392722567e-4,-.784874278e-4,-.186282737e-1,.227113063e-8,.105493575e-8,-.349872634e-5,-.263959706e-3},
                {300,-.776210977e-5,-.295672747e-4,-.120129928e-2,.181166638e-8,.801977741e-9,-.115552784e-6,-.420419196e-5},
                {400,.603953176e-5,-.100804610e-4,-.348784166e-3,.162604635e-8,.672018172e-9,-.200806021e-7,-.217733298e-6}};
            for(int i=0;i<3;i++)
            {
                var actual=HumidFugacity.VirialCoefficients(printed[i,0]);
                for(int j=0;j<7;j++)Compare("G11-Table7-"+i,"virial-"+j,j<3?"m3/mol":"m6/mol2","IAPWS G11-15 Table7",actual[j],printed[i,j+1],Math.Abs(printed[i,j+1])*6e-9);
            }
            double[,] f={{10000,998.917199,8956.77892},{100000,9890.90701,85431.8372},{1000000,88406.1686,360075.128}};
            for(int i=0;i<3;i++)for(int j=0;j<2;j++)Compare("G11-Table8-"+i+"-"+j,"fugacity","Pa","IAPWS G11-15 Table8 unstable reference states",HumidFugacity.ReferenceFugacity(300,f[i,0],j==0?.1:.9),f[i,j+1],f[i,j+1]*6e-9);
            var aux=HumidFugacity.Auxiliary(300,.1);
            Compare("G11-Table8-beta","beta","m3/mol","G11-15 Table8",aux[0],-.269858540e-3,6e-13);
            Compare("G11-Table8-gamma","gamma","m6/mol2","G11-15 Table8",aux[1],-.187149793e-6,6e-16);
            var gas=new HumidGas();
            foreach(var item in new[]{new[]{333.15,.1535,19.44e-6,.02903},new[]{343.15,.2792,18.73e-6,.02867},new[]{353.15,.5529,17.23e-6,.02753},new[]{363.15,1.4202,14.57e-6,.02535},new[]{473.0,1.0,20.32e-6,.03526}})
            {
                var candidate=HumidDiluteTransport.Reference(item[0],item[1]/(1+item[1]));
                for(int j=0;j<2;j++)observations.Add(new MaterialModelObservation {case_id="VW-dilute-candidate-"+item[0],quantity=j==0?"mu":"k",unit=j==0?"Pa s":"W/(m K)",source="Explicit dilute approximation vs author/ASHRAE calculated tables; not a passing assertion",expected=item[j+2],actual=candidate[j],relative_difference_percent=100*(candidate[j]-item[j+2])/item[j+2]});
            }
            // Table-provider node regression is distinct from an independent model
            // comparison. The older equimolar fit disagrees increasingly when hot.
            foreach(var item in new[]{new[]{303.15,26.5e-6},new[]{373.15,37.6e-6},new[]{403.15,42.8e-6},new[]{433.15,48.3e-6},new[]{463.15,54.0e-6}})
            {
                Compare("ASHRAE-diffusion-node-"+item[0],"D_water_air","m2/s","ASHRAE2025 Table4 node reproduction",gas.BinaryDiffusivityM2s(item[0],101325,.01,GasDiffusionModel.MasonMonchickAshrae2025Tabulated),item[1],1e-18);
                double predicted=HumidGas.ReferenceHotDiluteDiffusivity(item[0],101325);
                observations.Add(new MaterialModelObservation {case_id="Marrero-ASHRAE-"+item[0],quantity="D_water_air",unit="m2/s",source="Marrero1972 Table13 vs ASHRAE2025 Table4 calculated model difference; not a passing assertion",expected=item[1],actual=predicted,relative_difference_percent=100*(predicted-item[1])/item[1]});
            }
            Compare("ASHRAE-cold-diffusion","D_water_air","m2/s","ASHRAE2025 Table4 cold node",gas.BinaryDiffusivityM2s(223.15,101325,0,GasDiffusionModel.MasonMonchickAshrae2025Tabulated),15.6e-6,1e-18);
            Compare("ASHRAE-inverse-pressure","ratio","1","dilute-gas inverse-pressure assumption",HumidGas.ReferenceTabulatedDiffusivity(400,80000)/HumidGas.ReferenceTabulatedDiffusivity(400,101325),101325.0/80000,1e-14);
            Reject("diffusion-no-rich-mixture-claim",()=>gas.BinaryDiffusivityM2s(400,101325,.5,GasDiffusionModel.MasonMonchickAshrae2025Tabulated));
            Reject("diffusion-no-extrapolation",()=>gas.BinaryDiffusivityM2s(480,101325,.01,GasDiffusionModel.MasonMonchickAshrae2025Tabulated));
            foreach(var point in new[]{new[]{200.0,1.0,.107752281},new[]{250.0,101325.0,.0001},new[]{300.0,100000.0,.022394202},new[]{400.0,1000000.0,.174434709}})
            {
                double expected=HumidFugacity.ReferenceFugacity(point[0],point[1],HumidGas.WaterMoleFraction(point[2]));
                Compare("G8-G11-"+point[0],"fugacity","Pa","G8 full Helmholtz versus G11 truncated virial; project cross-model limit 0.1%",gas.ReferenceHelmholtzFugacity(point[0],point[1],point[2]),expected,.001*expected);
            }
            foreach(double t in new[]{200.0,223.15,250,270,273.0})foreach(double p in new[]{80000.0,101325,1e6})
            {
                var e=gas.EquilibriumWithIce(t,p);string id="ice-contact-"+t+"-"+p;
                Compare(id,"mu_gas-mu_ice","J/kg","G8-10 Eq12 with R10-06 ice",e.ChemicalPotentialResidualJkg,0,1e-6);
                Compare(id,"p residual","Pa","density inversion",e.Gas.PressurePa-p,0,1e-6);
                Compare(id,"stable accepted","1","ice-contact stability",gas.Evaluate(t,p,e.WaterMassFraction)!=null?1:0,1,0);
                Reject(id+"-supersaturated",()=>gas.Evaluate(t,p,e.WaterMassFraction*1.001));
            }
            // Official G8-10 200 K ice-contact check is at 1 Pa, outside the public
            // atmospheric envelope. Its rounded composition is checked directly by mu.
            var raw=gas.ReferenceRhoT(200,.0000163479657,.107752281);
            Compare("G8-ice-official","mu_gas-mu_ice","J/kg","G8-10 Table13 rounded inputs",gas.ReferenceWaterChemicalPotential(200,raw.DensityKgM3,.107752281)-new IceIh().ReferenceTp(200,1).GibbsJkg,0,.003);
            Reject("ice-contact-above-melting",()=>gas.EquilibriumWithIce(273.16,101325));
            Reject("G11-production-supersaturation",()=>HumidFugacity.StableFugacity(300,101325,.1));
            Reject("G11-outside-T",()=>HumidFugacity.ReferenceFugacity(190,101325,.001));
            var ice=new IceIh(IceConductivityModel.Yen1981PurePolycrystalline);
            Compare("Yen-near-melting","k","W/(m K)","Yen1981 p15 near273K approx2.2; model comparison 10%",ice.ConductivityWmK(273.15),2.2,.22);
            Compare("Yen-minus50","k positive","1","Yen Eq33 inside declared envelope",ice.ConductivityWmK(223.15)>ice.ConductivityWmK(273.15)?1:0,1,0);
            Reject("Yen-no-high-temperature",()=>ice.ConductivityWmK(274));
        }
    }
}
