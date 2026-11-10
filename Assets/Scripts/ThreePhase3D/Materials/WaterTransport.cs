using System;

namespace PhysicsSim.ThreePhase3D.Materials
{
    public static class WaterTransport
    {
        // Physics-ID: MAT-WATER-TRANSPORT
        // Implements: R016
        // Model: R12-08 Eqs 10-22; R15-11 Eqs 10,16-22; R1-76(2014)
        // Sources: D04 D05 D12; Source-Locators: official release coefficient/check tables
        // Inputs-SI: T [K], rho [kg/m3], drho/dp [kg/(m3 Pa)], cp,cv [J/(kg K)]
        // Outputs-SI: mu [Pa s], k [W/(m K)], sigma [N/m]
        // State-Ownership: pure evaluations; Conservation: no inventory changes
        // Discretisation: FP64 direct correlation; Validity: restricted beaker domain
        // Validation: V05 V32
        // Source-Locators: Official release equations and tables listed above.
        // Conservation: No transfers; shared IAPWS reference convention.
        // Validity: Public provider bounds are narrower than homogeneous reference checks.
        // Evidence: PENDING
        // Status: IMPLEMENTED_UNVERIFIED
        // Limitations: S02 remains incomplete; see Docs/ThreePhase3D/S02-Handoff.md.
        private static void Validate(ThermodynamicState s)
        {if(s==null)throw new ArgumentNullException(nameof(s));Guard.Range(s.TemperatureK,273.16,600);Guard.Range(s.PressurePa,611.66,5e6);}
        public static double SurfaceTension(double t)
        {Guard.Range(t,273.16,Water95.Tc);double q=1-t/Water95.Tc;return .2358*Math.Pow(q,1.256)*(1-.625*q);}
        public static double SurfaceTensionDerivative(double t)
        {Guard.Range(t,273.16,Water95.Tc);double q=1-t/Water95.Tc;return -.2358/Water95.Tc*(1.256*Math.Pow(q,.256)*(1-.625*q)-.625*Math.Pow(q,1.256));}
        // Background terms exposed separately because the official releases supply checks with enhancement=0.
        public static double ReferenceViscosityBackground(double t,double rho)
        {
            Guard.Range(t,273.16,1173.15);Guard.Range(rho,0,1300);
            double tr=t/Water95.Tc,d=rho/322;
            double[] h={1.67752,2.20462,.6366564,-.241605};double den=0;
            for(int i=0;i<h.Length;i++)den+=h[i]/Math.Pow(tr,i);
            int[] ii={0,1,2,3,0,1,2,3,5,0,1,2,3,4,0,1,0,3,4,3,5},jj={0,0,0,0,1,1,1,1,1,2,2,2,2,2,3,3,4,4,5,6,6};
            double[] n={.520094,.0850895,-1.08374,-.289555,.222531,.999115,1.88797,1.26613,.120573,-.281378,-.906851,-.772479,-.489837,-.257040,.161913,.257399,-.0325372,.0698452,.00872102,-.00435673,-.000593264};
            double sum=0;for(int i=0;i<n.Length;i++)sum+=n[i]*Math.Pow(1/tr-1,ii[i])*Math.Pow(d-1,jj[i]);
            return 1e-6*100*Math.Sqrt(tr)/den*Math.Exp(d*sum);
        }
        private static double CorrelationLengthNm(ThermodynamicState s)
        {
            var reference=new Water95().ReferenceRhoT(1.5*Water95.Tc,s.DensityKgM3);
            double d=s.DensityKgM3/322;
            double susceptibility=22.064e6*d/322*(s.DensityPressureDerivative-reference.DensityPressureDerivative*1.5*Water95.Tc/s.TemperatureK);
            return susceptibility<=0?0:.13*Math.Pow(susceptibility/.06,.63/1.239);
        }
        public static double Viscosity(ThermodynamicState s)
        {
            Validate(s);double xi=CorrelationLengthNm(s),qc=1/1.9,qd=1/1.1,y=0;
            if(xi<=.3817016416)y=qc/5*xi*Math.Pow(qd*xi,5)*(1-qc*xi+Math.Pow(qc*xi,2)-765.0/504*Math.Pow(qd*xi,2));
            else
            {
                double q=qc*xi,phi=Math.Acos(1/Math.Sqrt(1+qd*qd*xi*xi));
                double w=Math.Sqrt(Math.Abs((q-1)/(q+1)))*Math.Tan(phi/2);
                double lw=q>1?Math.Log((1+w)/(1-w)):2*Math.Atan(w);
                y=Math.Sin(3*phi)/12-Math.Sin(2*phi)/(4*q)+(1-1.25*q*q)/(q*q)*Math.Sin(phi)-((1-1.5*q*q)*phi-Math.Pow(Math.Abs(q*q-1),1.5)*lw)/(q*q*q);
            }
            return ReferenceViscosityBackground(s.TemperatureK,s.DensityKgM3)*Math.Exp(.068*y);
        }
        public static double ReferenceConductivityBackground(double t,double rho)
        {
            Guard.Range(t,273.16,1173.15);Guard.Range(rho,0,1300);
            double tr=t/Water95.Tc,d=rho/322;
            double[] n0={.002443221,.01323095,.006770357,-.003454586,.0004096266};double den=0;
            for(int i=0;i<n0.Length;i++)den+=n0[i]/Math.Pow(tr,i);
            int[] ii={0,0,0,0,0,0,1,1,1,1,1,1,2,2,2,2,2,2,3,3,3,3,4,4,4,4,4,4},jj={0,1,2,3,4,5,0,1,2,3,4,5,0,1,2,3,4,5,0,1,2,3,0,1,2,3,4,5};
            double[] n={1.60397357,-.646013523,.111443906,.102997357,-.0504123634,.00609859258,2.33771842,-2.78843778,1.53616167,-.463045512,.0832827019,-.00719201245,2.19650529,-4.54580785,3.55777244,-1.40944978,.275418278,-.0205938816,-1.21051378,1.60812989,-.621178141,.0716373224,-2.7203370,4.57586331,-3.18369245,1.1168348,-.19268305,.012913842};
            double sum=0;for(int i=0;i<n.Length;i++)sum+=n[i]*Math.Pow(1/tr-1,ii[i])*Math.Pow(d-1,jj[i]);
            return .001*Math.Sqrt(tr)/den*Math.Exp(d*sum);
        }
        public static double Conductivity(ThermodynamicState s)
        {
            Validate(s);double y=CorrelationLengthNm(s)/.4,d=s.DensityKgM3/322,z=0;
            if(y>=1.2e-7)
            {
                double r=s.CvJkgK/s.CpJkgK;
                z=2/(Math.PI*y)*((1-r)*Math.Atan(y)+y*r-(1-Math.Exp(-1/(1/y+y*y/(3*d*d)))));
            }
            double enhancement=.001*177.8514*d*(s.CpJkgK/Water95.R)*(s.TemperatureK/Water95.Tc)/(Viscosity(s)*1e6)*z;
            return ReferenceConductivityBackground(s.TemperatureK,s.DensityKgM3)+enhancement;
        }
    }
}
