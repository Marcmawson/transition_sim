using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Runtime.Serialization;
using System.Text;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Materials;

namespace PhysicsSim.ThreePhase3D.Verification
{
    [DataContract] public sealed class MaterialComparison
    {
        [DataMember] public string case_id, quantity, unit, source;
        [DataMember] public double expected, actual, absolute_error, tolerance;
        [DataMember] public bool passed;
    }
    [DataContract] public sealed class MaterialReport
    {
        [DataMember] public string schema="S02-CPU-3", utc, source_base="4d571dc2f0e2b2649b7cd2a7fae63063260778a6";
        [DataMember] public bool gpu_executed=false, complete_stage=false;
        [DataMember] public string[] open_items={"Steam-rich mixture transport beyond Y=0.10: historical-model disagreement with modern references", "Hot/steam-rich water-air diffusion beyond dilute near-STP scope", "Expanded pressure/ice/gas GPU tables require measured device execution"};
        [DataMember] public MaterialComparison[] comparisons;
        [DataMember] public MaterialModelObservation[] model_observations;
    }
    [DataContract] public sealed class MaterialModelObservation
    {
        [DataMember] public string case_id,quantity,unit,source;
        [DataMember] public double expected,actual,relative_difference_percent;
    }
    public sealed partial class MaterialChecks
    {
        private readonly List<MaterialComparison> rows=new List<MaterialComparison>();
        private readonly Water95 water=new Water95();
        private readonly List<WaterTableData> tables=new List<WaterTableData>();
        private readonly List<MaterialSurfaceData> surfaces=new List<MaterialSurfaceData>();
        private readonly List<MaterialModelObservation> observations=new List<MaterialModelObservation>();
        private void Compare(string id,string q,string unit,string source,double actual,double expected,double tolerance)
        {
            double error=Math.Abs(actual-expected);
            rows.Add(new MaterialComparison {case_id=id,quantity=q,unit=unit,source=source,actual=actual,expected=expected,absolute_error=error,tolerance=tolerance,passed=!double.IsNaN(error)&&error<=tolerance});
        }
        private void Reject(string id,Action action)
        {bool rejected=false;try{action();}catch(ArgumentException){rejected=true;}catch(NotSupportedException){rejected=true;}Compare(id,"rejected","1","V32",rejected?1:0,1,0);}
        public MaterialReport Run()
        {
            // Independent published oracle: IAPWS R6-95(2018) Table 7, SI conversion only.
            Jet2 delta=new Jet2(838.025/322,1),tau=new Jet2(Water95.Tc/500,0,1),temp=Water95.Tc/tau;
            Jet2 phi=Water95.Helmholtz(temp,322*delta)/(Water95.R*temp);
            double[] phiActual={phi.V,phi.X,phi.XX,phi.Y,phi.YY,phi.XY};
            double[] phiExpected={2.04797733-3.42693206,.384236747-.364366650,-.147637878+.856063701,9.04611106-5.81403435,-1.93249185-2.23440737,-1.12176915};
            string[] phiNames={"phi","phi_delta","phi_delta_delta","phi_tau","phi_tau_tau","phi_delta_tau"};
            for(int i=0;i<6;i++)Compare("D01-T6-total",phiNames[i],"1","D01 Table 6 sum of rounded ideal and residual columns",phiActual[i],phiExpected[i],1.1e-8);
            double[,] states={{300,996.556,.0992418352,4.13018112,1501.51914,.393062643},
                {300,1005.308,20.0022515,4.06798347,1534.92501,.387405401},
                {300,1188.202,700.004704,3.46135580,2443.57992,.132609616},
                {500,.435,.0999679423,1.50817541,548.314253,7.94488271},
                {500,4.532,.999938125,1.66991025,535.739001,6.82502725},
                {500,838.025,10.0003858,3.22106219,1271.28441,2.56690919},
                {500,1084.564,700.000405,3.07437693,2412.00877,2.03237509},
                {647,358,22.0384756,6.18315728,252.145078,4.32092307},
                {900,.241,.100062559,1.75890657,724.027147,9.16653194},
                {900,52.615,20.0000690,1.93510526,698.445674,6.59070225},
                {900,870.769,700.000006,2.66422350,2019.33608,4.17223802}};
            for(int i=0;i<states.GetLength(0);i++)
            {
                var s=water.ReferenceRhoT(states[i,0],states[i,1]);string id="D01-T7-"+i;
                Compare(id,"p","Pa","D01 Table 7",s.PressurePa,states[i,2]*1e6,Math.Max(1e-4,states[i,2]*1e6*6e-9));
                Compare(id,"cv","J/(kg K)","D01 Table 7",s.CvJkgK,states[i,3]*1000,6e-6);
                Compare(id,"sound","m/s","D01 Table 7",s.SoundSpeedMs,states[i,4],6e-6);
                Compare(id,"s","J/(kg K)","D01 Table 7",s.EntropyJkgK,states[i,5]*1000,6e-6);
            }
            double[,] sats={{275,.000698451167,999.887406,.00550664919,7.75972202,2504.28995,.0283094670,9.10660121},
                {450,.932203564,890.341250,4.81200360,749.161585,2774.41078,2.10865845,6.60921221},
                {625,16.9082693,567.090385,118.290280,1686.26976,2550.71625,3.80194683,5.18506121}};
            for(int i=0;i<3;i++)
            {
                var s=water.Saturation(sats[i,0]);string id="D01-T8-"+i;
                Compare(id,"p","Pa","D01 Table 8",s.PressurePa,sats[i,1]*1e6,Math.Max(1e-4,sats[i,1]*1e6*6e-9));
                Compare(id,"rho_l","kg/m3","D01 Table 8",s.Liquid.DensityKgM3,sats[i,2],6e-7);
                Compare(id,"rho_v","kg/m3","D01 Table 8",s.Vapour.DensityKgM3,sats[i,3],Math.Max(6e-12,sats[i,3]*6e-9));
                Compare(id,"h_l","J/kg","D01 Table 8",s.Liquid.EnthalpyJkg,sats[i,4]*1000,.006);
                Compare(id,"h_v","J/kg","D01 Table 8",s.Vapour.EnthalpyJkg,sats[i,5]*1000,.006);
                Compare(id,"s_l","J/(kg K)","D01 Table 8",s.Liquid.EntropyJkgK,sats[i,6]*1000,6e-6);
                Compare(id,"s_v","J/(kg K)","D01 Table 8",s.Vapour.EntropyJkgK,sats[i,7]*1000,6e-6);
                Compare(id,"g_l-g_v","J/kg","D01 phase equilibrium",s.Liquid.GibbsJkg-s.Vapour.GibbsJkg,0,2e-7);
            }
            var ice=new IceIh();var cold=ice.ReferenceTp(100,100e6);
            Compare("D02-T6-cold","rho","kg/m3","D02 Table 6",cold.DensityKgM3,941.678203297,1e-8);
            Compare("D02-T6-cold","h","J/kg","D02 Table 6",cold.EnthalpyJkg,-483491.635676,.00001);
            Compare("D02-T6-cold","s","J/(kg K)","D02 Table 6",cold.EntropyJkgK,-2611.95122589,.000001);
            var melt=ice.ReferenceTp(273.152519,101325);
            Compare("D02-T6-normal","u","J/kg","D02 Table 6",melt.InternalEnergyJkg,-333465.403393,.00001);
            Compare("D02-T6-normal","cp","J/(kg K)","D02 Table 6",melt.CpJkgK,2096.71391024,.000001);
            var preparedIce=new IceIh(IceConductivityModel.BonalesSlowFrozenCorrected2018);
            Compare("D15-corrected-cold","k","W/(m K)","Bonales 2018 corrigendum; arithmetic coefficient check",preparedIce.ConductivityWmK(233.15),2.7566,1e-12);
            Compare("D15-corrected-warm","k","W/(m K)","Bonales 2018 corrigendum; arithmetic coefficient check",preparedIce.ConductivityWmK(268.15),2.1406,1e-12);
            Reject("D15-no-extrapolation-to-melting",()=>preparedIce.ConductivityWmK(273.15));
            Reject("D15-no-extrapolation-to-minus50",()=>preparedIce.ConductivityWmK(223.15));
            Compare("D03-melt","p","Pa","D03 computer verification",PhaseEquilibrium.MeltingPressure(260),138.268113e6,1);
            Compare("D03-subl","p","Pa","D03 computer verification",PhaseEquilibrium.SublimationPressure(230),8.94735274,1e-7);
            Compare("D03-inverse","T","K","D03 bisection residual",PhaseEquilibrium.MeltingTemperature(PhaseEquilibrium.MeltingPressure(270)),270,1e-12);
            double[,] mus={{298.15,998,889.735100},{298.15,1200,1437.649467},{373.15,1000,307.883622},{433.15,1,14.538324},{433.15,1000,217.685358},{873.15,1,32.619287},{873.15,100,35.802262},{873.15,600,77.430195},{1173.15,1,44.217245},{1173.15,100,47.640433},{1173.15,400,64.154608}};
            for(int i=0;i<mus.GetLength(0);i++)Compare("D04-T4-"+i,"mu","Pa s","D04 Table 4 (mu2=1)",WaterTransport.ReferenceViscosityBackground(mus[i,0],mus[i,1]),mus[i,2]*1e-6,5.1e-13);
            double[,] ks={{298.15,0,18.4341883},{298.15,998,607.712868},{298.15,1200,799.038144},{873.15,0,79.1034659}};
            for(int i=0;i<4;i++)Compare("D12-T4-"+i,"k","W/(m K)","D12 Table 4 (k2=0)",WaterTransport.ReferenceConductivityBackground(ks[i,0],ks[i,1]),ks[i,2]*.001,5.1e-10);
            var gas=new HumidGas();
            var criticalDensityAwayFromCritical=water.ReferenceRhoT(900,322);
            Compare("rho-critical-axis-away-from-Tc","dp/drho","Pa m3/kg","central difference verifies nonanalytic composite limit at delta=1",1/criticalDensityAwayFromCritical.DensityPressureDerivative,
                (water.ReferenceRhoT(900,322.001).PressurePa-water.ReferenceRhoT(900,321.999).PressurePa)/.002,.01);
            // G8-10 Table 13 uses rounded A,rho inputs. Its extra digits are not a new measurement.
            double[,] gasCases={{200,.0000163479657,.107752281,.999999998,189712.231,4053.17966,1093.87397,291.394959},
                {300,1.14614216,.022394202,100000,83490.8383,296.711481,1026.81324,349.234196},
                {400,7.93354063,.174434709,1000000,577649.408,1068.91931,1235.52454,416.656820}};
            for(int i=0;i<3;i++)
            {
                var s=gas.ReferenceRhoT(gasCases[i,0],gasCases[i,1],gasCases[i,2]);string id="D06-T13-"+i;
                Compare(id,"p","Pa","D06 Table 13",s.PressurePa,gasCases[i,3],Math.Max(1e-7,gasCases[i,3]*2e-8));
                Compare(id,"h","J/kg","D06 Table 13 rounded inputs",s.EnthalpyJkg,gasCases[i,4],.01);
                Compare(id,"s","J/(kg K)","D06 Table 13 rounded inputs",s.EntropyJkgK,gasCases[i,5],.0001);
                Compare(id,"cp","J/(kg K)","D06 Table 13 rounded inputs",s.CpJkgK,gasCases[i,6],.0001);
                Compare(id,"sound","m/s","D06 Table 13 rounded inputs",s.SoundSpeedMs,gasCases[i,7],.00001);
                double[] mu={-109950.917,-5265.05193,-106748.981};
                Compare(id,"water chemical potential","J/kg","D06 Table 13 rounded inputs",gas.ReferenceWaterChemicalPotential(gasCases[i,0],gasCases[i,1],gasCases[i,2]),mu[i],.01);
            }
            Compare("D08-STP","D","m2/s","Massman 1998 Eq 1 Table 8",HumidGas.ReferenceDiffusivity(273.15,101325),.2178e-4,1e-20);
            Compare("D08-pressure","inverse pressure ratio","1","Massman Eq 1",HumidGas.ReferenceDiffusivity(300,80000)/HumidGas.ReferenceDiffusivity(300,101325),101325.0/80000,1e-14);
            for(int i=0;i<20;i++)
            {
                double t=275+4*i,p=101325;var s=water.Evaluate(t,p,WaterBranch.Liquid);
                var a=water.Evaluate(t-.001,p,WaterBranch.Liquid);var b=water.Evaluate(t+.001,p,WaterBranch.Liquid);
                Compare("identity-"+i,"dh/dT=cp","J/(kg K)","thermodynamic identity independent central difference",(b.EnthalpyJkg-a.EnthalpyJkg)/.002,s.CpJkgK,.005);
                Compare("identity-"+i,"h-u-p/rho","J/kg","thermodynamic identity",s.EnthalpyJkg-s.InternalEnergyJkg-p/s.DensityKgM3,0,1e-6);
                Compare("identity-"+i,"drho/dT","kg/(m3 K)","independent central difference",(b.DensityKgM3-a.DensityKgM3)/.002,s.DensityTemperatureDerivative,1e-7);
            }
            Compare("density-anomaly","rho277>rho274","1","D01 predicted density maximum",water.Evaluate(277.13,101325,WaterBranch.Liquid).DensityKgM3>water.Evaluate(274,101325,WaterBranch.Liquid).DensityKgM3?1:0,1,0);
            Compare("ice-expansion","V_ice>V_liquid at equal mass","1","D01/D02",.25/ice.Evaluate(273.1,101325).DensityKgM3>.25/water.Evaluate(273.16,101325,WaterBranch.Liquid).DensityKgM3?1:0,1,0);
            Compare("ambient-air","density near 1kg/m3","1","D06",gas.Evaluate(300,101325,.01).DensityKgM3>1?1:0,1,0);
            Reject("NaN-water",()=>water.Evaluate(double.NaN,101325,WaterBranch.Liquid));
            Reject("negative-pressure",()=>water.Evaluate(300,-1,WaterBranch.Liquid));
            Reject("wrong-liquid-branch",()=>water.Evaluate(500,101325,WaterBranch.Liquid));
            Reject("wrong-vapour-branch",()=>water.Evaluate(300,101325,WaterBranch.Vapour));
            Reject("undefined-branch",()=>water.Evaluate(300,101325,(WaterBranch)99));
            Reject("critical-region",()=>water.ReferenceRhoT(Water95.Tc,322));
            Reject("melted-ice",()=>ice.Evaluate(273.16,101325));
            Reject("gas-composition",()=>gas.Evaluate(300,101325,1.1));
            Reject("condensing-gas",()=>gas.Evaluate(300,101325,.1));
            Reject("missing-ice-k",()=>ice.ConductivityWmK(250));
            Reject("gas-transport-high-pressure",()=>gas.ViscosityPaS(300,200000,.01));
            Reject("Massman-hot-gas-diffusion",()=>gas.BinaryDiffusivityM2s(400,101325,.01));
            Compare("hot-gas-diffusion","D>0","1","ASHRAE2025 Table4 explicitly selected tabulated model",gas.BinaryDiffusivityM2s(400,101325,.01,GasDiffusionModel.MasonMonchickAshrae2025Tabulated)>0?1:0,1,0);
            Reject("steam-rich-diffusion",()=>gas.BinaryDiffusivityM2s(360,101325,.1));
            Reject("missing-duran-cp",()=>new DuranDataGap().CpJkgK(300));
            GasCompletionChecks();
            if(rows.All(r=>r.passed)){TableChecks();SurfaceChecks();}
            return new MaterialReport {utc=DateTime.UtcNow.ToString("o"),comparisons=rows.ToArray(),model_observations=observations.ToArray()};
        }
        public static double TableTolerance(int property,double expected)
        {
            if(property==1||property==2)return Math.Max(1,Math.Abs(expected)*1e-4);
            if(property==3)return Math.Max(.001,Math.Abs(expected)*1e-4);
            if(property==7)return Math.Max(2e-6,Math.Abs(expected)*1e-4);
            return Math.Max(1e-14,Math.Abs(expected)*1e-4);
        }
        private void TableChecks()
        {
            foreach(var phase in new[]{WaterBranch.Liquid,WaterBranch.Vapour})
            {
                var data=WaterPropertyTable.Generate(phase);tables.Add(data);
                var table=new WaterPropertyTable(RecordJson.Decode<WaterTableData>(RecordJson.Encode(data)));
                // Staggered deterministic interior probes: interpolation between nodes, not a
                // generator-to-itself node comparison. The oracle already passed official tables.
                for(int i=0;i<97;i++)
                {
                    float t=(float)(table.MinimumK+(table.MaximumK-table.MinimumK)*(i+.371)/97);
                    var expected=WaterPropertyTable.EvaluateReference(water,t,101325,phase);var actual=table.EvaluateFloat(t,101325);
                    for(int j=0;j<WaterPropertyTable.Width;j++)Compare("FP32-"+phase+"-"+i,WaterPropertyTable.Quantities[j],WaterPropertyTable.Units[j],"D01-verified oracle; Validation.md table error target",actual[j],expected[j],TableTolerance(j,expected[j]));
                }
                Reject("table-low-"+phase,()=>table.EvaluateFloat((float)table.MinimumK-.25f,101325));
                Reject("table-high-"+phase,()=>table.EvaluateFloat((float)table.MaximumK+.25f,101325));
                Reject("table-pressure-"+phase,()=>table.EvaluateFloat((float)table.MinimumK,101000));
                Reject("table-NaN-"+phase,()=>table.EvaluateFloat(float.NaN,101325));
                data.units[0]="g/cm3";Reject("table-units-"+phase,()=>new WaterPropertyTable(data));data.units[0]=WaterPropertyTable.Units[0];
                float saved=data.values[0];data.values[0]=float.NaN;Reject("table-corrupt-"+phase,()=>new WaterPropertyTable(data));data.values[0]=saved;
            }
        }
        public static string Execute(string projectRoot)
        {
            var checks=new MaterialChecks();var report=checks.Run();
            string output=Path.Combine(projectRoot,"ValidationRuns/local","s02-materials-"+DateTime.UtcNow.ToString("yyyyMMddTHHmmss")+"-"+Guid.NewGuid().ToString("N"));Directory.CreateDirectory(output);
            File.WriteAllBytes(Path.Combine(output,"comparisons.json"),RecordJson.Encode(report));
            if(report.comparisons.All(r=>r.passed))
            {Directory.CreateDirectory(Path.Combine(output,"tables"));foreach(var table in checks.tables)File.WriteAllBytes(Path.Combine(output,"tables",table.phase.ToLowerInvariant()+"-101325Pa.json"),RecordJson.Encode(table));foreach(var table in checks.surfaces)File.WriteAllBytes(Path.Combine(output,"tables",table.surface+"-TP.json"),RecordJson.Encode(table));}
            var csv=new StringBuilder("case_id,quantity,unit,source,expected,actual,absolute_error,tolerance,passed\n");
            foreach(var r in report.comparisons)csv.AppendLine(string.Join(",",new[]{r.case_id,r.quantity,r.unit,r.source,r.expected.ToString("R",CultureInfo.InvariantCulture),r.actual.ToString("R",CultureInfo.InvariantCulture),r.absolute_error.ToString("R",CultureInfo.InvariantCulture),r.tolerance.ToString("R",CultureInfo.InvariantCulture),r.passed.ToString()}));
            File.WriteAllText(Path.Combine(output,"comparisons.csv"),csv.ToString(),new UTF8Encoding(false));
            var sources=Directory.GetFiles(Path.Combine(projectRoot,"Assets/Scripts/ThreePhase3D/Materials"),"*.cs").Concat(new[]{
                "Assets/Scripts/ThreePhase3D/Verification/MaterialChecks.cs","Assets/Scripts/ThreePhase3D/Verification/MaterialGasChecks.cs","Assets/Scripts/ThreePhase3D/Verification/MaterialExtendedChecks.cs","Assets/Scripts/ThreePhase3D/Editor/MaterialVerificationMenu.cs",
                "Assets/Resources/ThreePhase3D/WaterPropertyLookup.compute","Assets/Resources/ThreePhase3D/MaterialSurfaceLookup.compute","Assets/Scripts/ThreePhase3D/Editor/MaterialSurfaceVerificationMenu.cs","Tools/ThreePhase3D/MaterialMain.cs","Tools/ThreePhase3D/RunMaterialChecks.ps1"
            }.Select(p=>Path.Combine(projectRoot,p))).OrderBy(p=>p,StringComparer.Ordinal);
            File.WriteAllText(Path.Combine(output,"source-hashes.txt"),"# SHA-256 with source CRLF normalised to LF; not a signed attestation\n"+string.Join("\n",sources.Select(p=>PhysicsSim.ThreePhase3D.Diagnostics.ProvenanceCapture.SourceHash(p)+"  "+p.Substring(projectRoot.Length+1).Replace('\\','/')))+"\n");
            File.WriteAllText(Path.Combine(output,"runner.txt"),"Managed executable SHA256: "+RecordJson.FileHash(typeof(MaterialChecks).Assembly.Location)+"\nManaged runtime: "+Environment.Version+"\nCompiler/reference environment: Unity 2022.3.46f1; see RunMaterialChecks.ps1\nGPU executed: false\nStage complete: false\n");
            File.WriteAllText(Path.Combine(output,"files.sha256"),string.Join("\n",Directory.GetFiles(output,"*",SearchOption.AllDirectories).OrderBy(p=>p,StringComparer.Ordinal).Select(p=>RecordJson.FileHash(p)+"  "+p.Substring(output.Length+1).Replace('\\','/')))+"\n");
            int failed=report.comparisons.Count(r=>!r.passed);
            Console.WriteLine("S02 CPU comparisons: "+report.comparisons.Length+"; failures: "+failed+"; GPU executed: false; stage complete: false.");
            Console.WriteLine("Additional model observations: "+report.model_observations.Length+"; these are not counted as passing numerical comparisons. Production mixture transport is limited to Y<=0.10 or pure vapour.");
            foreach(var r in report.comparisons.Where(r=>!r.passed))Console.WriteLine("FAIL "+r.case_id+" "+r.quantity+": actual="+r.actual.ToString("R")+" expected="+r.expected.ToString("R")+" tolerance="+r.tolerance);
            Console.WriteLine(output);
            if(failed>0)throw new InvalidOperationException("Material checks failed; retained output: "+output);
            return output;
        }
    }
}
