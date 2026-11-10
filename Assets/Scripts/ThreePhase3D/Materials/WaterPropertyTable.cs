using System;
using System.Runtime.Serialization;

namespace PhysicsSim.ThreePhase3D.Materials
{
    [DataContract] public sealed class WaterTableData
    {
        [DataMember] public string schema="S02-ISOBAR-FP32-1", generator=Water95.Version;
        [DataMember] public string phase, enthalpy_reference="IAPWS liquid triple point u=s=0";
        [DataMember] public double pressure_pa, minimum_temperature_k, step_k;
        [DataMember] public int count;
        [DataMember] public string[] quantities, units;
        [DataMember] public float[] values;
    }
    // A deliberately bounded first GPU table: each phase has its own isobar.
    // Pressure is checked exactly rather than pretending a 1D table supplies pressure dependence.
    public sealed class WaterPropertyTable
    {
        public const int Width=12;
        public static readonly string[] Quantities={"rho","h","u","s","cp","cv","sound","drho_dT_p","drho_dp_T","mu","k","sigma"};
        public static readonly string[] Units={"kg/m3","J/kg","J/kg","J/(kg K)","J/(kg K)","J/(kg K)","m/s","kg/(m3 K)","kg/(m3 Pa)","Pa s","W/(m K)","N/m"};
        private readonly float[] values;
        public readonly double PressurePa,MinimumK,StepK;
        public readonly int Count;
        public readonly WaterBranch Branch;
        public double MaximumK=>MinimumK+(Count-1)*StepK;
        public WaterPropertyTable(WaterTableData data)
        {
            if(data==null || data.schema!="S02-ISOBAR-FP32-1" || data.generator!=Water95.Version ||
                data.enthalpy_reference!="IAPWS liquid triple point u=s=0" || data.count<2 || data.count>100000 ||
                data.quantities==null || data.units==null || data.quantities.Length!=Width || data.units.Length!=Width ||
                data.values==null || data.values.Length!=data.count*Width)throw new ArgumentException("Invalid water table schema/shape/reference.");
            Guard.Range(data.pressure_pa,101325,101325);Guard.Range(data.step_k,.01,1);
            for(int j=0;j<Width;j++)if(data.quantities[j]!=Quantities[j]||data.units[j]!=Units[j])throw new ArgumentException("Water table property order/units mismatch.");
            if(data.phase=="Liquid")Branch=WaterBranch.Liquid;else if(data.phase=="Vapour")Branch=WaterBranch.Vapour;else throw new ArgumentException("Unknown table branch.");
            PressurePa=data.pressure_pa;MinimumK=data.minimum_temperature_k;StepK=data.step_k;Count=data.count;
            double lo=Branch==WaterBranch.Liquid?274:374,hi=Branch==WaterBranch.Liquid?373:600;
            Guard.Range(MinimumK,lo,hi);Guard.Range(MaximumK,MinimumK,hi);
            values=(float[])data.values.Clone();foreach(float v in values)Guard.Finite(v);
            for(int i=0;i<Count;i++)foreach(int j in new[]{0,4,5,6,8,9,10,11})if(values[i*Width+j]<=0)throw new ArgumentException("Nonpositive property in water table.");
        }
        public static double[] EvaluateReference(Water95 water,double t,double p,WaterBranch phase)
        {
            var s=water.Evaluate(t,p,phase);
            return new[]{s.DensityKgM3,s.EnthalpyJkg,s.InternalEnergyJkg,s.EntropyJkgK,s.CpJkgK,s.CvJkgK,s.SoundSpeedMs,s.DensityTemperatureDerivative,s.DensityPressureDerivative,water.ViscosityPaS(s),water.ConductivityWmK(s),water.SurfaceTensionNm(t)};
        }
        public static WaterTableData Generate(WaterBranch phase)
        {
            if(phase!=WaterBranch.Liquid&&phase!=WaterBranch.Vapour)throw new ArgumentOutOfRangeException(nameof(phase));
            double lo=phase==WaterBranch.Liquid?274:374,hi=phase==WaterBranch.Liquid?373:600,step=phase==WaterBranch.Liquid?.125:.25;
            int count=(int)Math.Round((hi-lo)/step)+1;var data=new WaterTableData {phase=phase.ToString(),pressure_pa=101325,minimum_temperature_k=lo,step_k=step,count=count,quantities=(string[])Quantities.Clone(),units=(string[])Units.Clone(),values=new float[count*Width]};
            var water=new Water95();
            for(int i=0;i<count;i++) {double[] row=EvaluateReference(water,lo+i*step,101325,phase);for(int j=0;j<Width;j++)data.values[i*Width+j]=(float)row[j];}
            return data;
        }
        public float[] CopyValues()=>(float[])values.Clone();
        public float[] EvaluateFloat(float temperatureK,float pressurePa)
        {
            Guard.Range(temperatureK,MinimumK,MaximumK);Guard.Range(pressurePa,PressurePa,PressurePa);
            float x=(temperatureK-(float)MinimumK)/(float)StepK;int i=Math.Min(Count-2,(int)x);float weight=x-i;
            var result=new float[Width];for(int j=0;j<Width;j++) {float a=values[i*Width+j],b=values[(i+1)*Width+j];result[j]=a+(b-a)*weight;}return result;
        }
    }
}
