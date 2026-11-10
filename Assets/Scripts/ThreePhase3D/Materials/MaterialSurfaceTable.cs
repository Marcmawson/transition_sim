using System;
using System.Linq;
using System.Runtime.Serialization;

namespace PhysicsSim.ThreePhase3D.Materials
{
    public enum MaterialSurface { IceYen, GasDry, GasLean, GasRich, GasSteam, Liquid, Vapour }
    [DataContract] public sealed class MaterialSurfaceData
    {
        [DataMember] public string schema="S02-TP-FP32-1",surface,reference="IAPWS common water/ice reference; G8 air reference",generator="S02-Surface-1";
        [DataMember] public double minimum_k,step_k,minimum_pa=80000,step_pa=2000,water_mass_fraction;
        [DataMember] public int temperature_count,pressure_count=21;
        [DataMember] public string[] quantities,units;
        [DataMember] public float[] values;
    }
    // Physics-ID: MAT-MATERIAL-TP-TABLE
    // Implements: R013 R014 R015
        // Sources: D01 D02 D06 D18
    // Model: phase-separated T,p bilinear tables with a fixed, explicit gas composition
    // Source-Locators: CPU providers and their official reference checks
    // Inputs-SI: T [K], p [Pa], Y [kg/kg]
        // Outputs-SI: named SI property vector
    // State-Ownership: immutable copy
        // Conservation: common phase energy reference
    // Discretisation: FP32 bilinear interpolation; no interpolation across phase/composition
    // Validity: fixed generated domains, 80-120 kPa; ice stable through 273 K
    // Validation: V03 V04 V05 V32
        // Evidence: PENDING
        // Status: IMPLEMENTED_UNVERIFIED
    // Limitations: fixed Y surfaces are not a general composition table or a phase solver;
    // gas tables contain thermodynamics only, never unsupported steam-rich transport.
    public sealed class MaterialSurfaceTable
    {
        private readonly MaterialSurfaceData data;
        public readonly MaterialSurface Surface;
        public int Width=>data.quantities.Length;
        public MaterialSurfaceTable(MaterialSurfaceData input)
        {
            if(input==null||input.schema!="S02-TP-FP32-1"||input.generator!="S02-Surface-1"||
                input.reference!="IAPWS common water/ice reference; G8 air reference"||!Enum.TryParse(input.surface,out Surface)||!Enum.IsDefined(typeof(MaterialSurface),Surface))throw new ArgumentException("Surface schema/reference mismatch.");
            var expected=Descriptor(Surface);
            if(input.minimum_k!=expected.minimum_k||input.step_k!=expected.step_k||input.minimum_pa!=80000||input.step_pa!=2000||input.pressure_count!=21||input.temperature_count!=expected.temperature_count||input.water_mass_fraction!=expected.water_mass_fraction||input.quantities==null||input.units==null||!input.quantities.SequenceEqual(expected.quantities)||!input.units.SequenceEqual(expected.units))throw new ArgumentException("Surface axes/composition/units mismatch.");
            int length=checked(input.temperature_count*input.pressure_count*input.quantities.Length);
            if(input.values==null||input.values.Length!=length)throw new ArgumentException("Surface value shape mismatch.");
            foreach(float v in input.values)Guard.Finite(v);
            for(int i=0;i<input.temperature_count*input.pressure_count;i++)foreach(int j in new[]{0,4,5,6,8})if(input.values[i*input.quantities.Length+j]<=0)throw new ArgumentException("Nonpositive surface property.");
            data=new MaterialSurfaceData {surface=input.surface,minimum_k=input.minimum_k,step_k=input.step_k,temperature_count=input.temperature_count,water_mass_fraction=input.water_mass_fraction,quantities=(string[])input.quantities.Clone(),units=(string[])input.units.Clone(),values=(float[])input.values.Clone()};
        }
        public static MaterialSurfaceData Descriptor(MaterialSurface surface)
        {
            if(!Enum.IsDefined(typeof(MaterialSurface),surface))throw new ArgumentOutOfRangeException(nameof(surface));
            bool ice=surface==MaterialSurface.IceYen,liquid=surface==MaterialSurface.Liquid,vapour=surface==MaterialSurface.Vapour;
            double min=ice?200:liquid?274:380,max=ice?273:liquid?365:vapour?600:473,step=liquid?.125:.5;
            int width=ice?10:liquid||vapour?12:9;
            string[] quantities=WaterPropertyTable.Quantities.Take(width).ToArray(),units=WaterPropertyTable.Units.Take(width).ToArray();
            if(ice){quantities[9]="k";units[9]="W/(m K)";}
            return new MaterialSurfaceData {surface=surface.ToString(),minimum_k=min,step_k=step,temperature_count=(int)Math.Round((max-min)/step)+1,water_mass_fraction=surface==MaterialSurface.GasLean?.01:surface==MaterialSurface.GasRich?.9:surface==MaterialSurface.GasSteam?1:0,quantities=quantities,units=units};
        }
        public static double[] Reference(MaterialSurface surface,double t,double p,double y)
        {
            var d=Descriptor(surface);Guard.Range(t,d.minimum_k,d.minimum_k+(d.temperature_count-1)*d.step_k);Guard.Range(p,80000,120000);Guard.Range(y,d.water_mass_fraction,d.water_mass_fraction);
            if(surface==MaterialSurface.Liquid||surface==MaterialSurface.Vapour)return WaterPropertyTable.EvaluateReference(new Water95(),t,p,surface==MaterialSurface.Liquid?WaterBranch.Liquid:WaterBranch.Vapour);
            bool ice=surface==MaterialSurface.IceYen;var state=ice?new IceIh().Evaluate(t,p):new HumidGas().Evaluate(t,p,y);
            var row=new[]{state.DensityKgM3,state.EnthalpyJkg,state.InternalEnergyJkg,state.EntropyJkgK,state.CpJkgK,state.CvJkgK,state.SoundSpeedMs,state.DensityTemperatureDerivative,state.DensityPressureDerivative};
            return ice?row.Concat(new[]{new IceIh(IceConductivityModel.Yen1981PurePolycrystalline).ConductivityWmK(t)}).ToArray():row;
        }
        public static MaterialSurfaceData Generate(MaterialSurface surface)
        {
            var d=Descriptor(surface);int width=d.quantities.Length;d.values=new float[d.temperature_count*d.pressure_count*width];
            for(int p=0;p<d.pressure_count;p++)for(int t=0;t<d.temperature_count;t++)
            {var row=Reference(surface,d.minimum_k+t*d.step_k,d.minimum_pa+p*d.step_pa,d.water_mass_fraction);for(int j=0;j<width;j++)d.values[(p*d.temperature_count+t)*width+j]=(float)row[j];}
            return d;
        }
        public float[] Evaluate(float t,float p,float y)
        {
            Guard.Range(t,data.minimum_k,data.minimum_k+(data.temperature_count-1)*data.step_k);Guard.Range(p,80000,120000);Guard.Range(y,(float)data.water_mass_fraction,(float)data.water_mass_fraction);
            float tx=(t-(float)data.minimum_k)/(float)data.step_k,px=(p-80000)/2000;
            int ti=Math.Min(data.temperature_count-2,(int)tx),pi=Math.Min(19,(int)px);float tw=tx-ti,pw=px-pi;var result=new float[Width];
            for(int j=0;j<Width;j++)
            {int a=(pi*data.temperature_count+ti)*Width+j,b=a+data.temperature_count*Width;float low=data.values[a]+(data.values[a+Width]-data.values[a])*tw,high=data.values[b]+(data.values[b+Width]-data.values[b])*tw;result[j]=low+(high-low)*pw;}
            return result;
        }
    }
}
