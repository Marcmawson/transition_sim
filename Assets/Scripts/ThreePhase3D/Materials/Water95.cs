using System;

namespace PhysicsSim.ThreePhase3D.Materials
{
    public sealed class Water95 : IWaterProperties
    {
        public const string Version="S02-Water95-1";
        public const double Tc=647.096, Rhoc=322, R=461.51805;
        // IAPWS R6-95(2018), Tables 1-2. Coefficients are formulation data;
        // implementation uses forward differentiation rather than hand-transcribed derivative tables.
        private static readonly double[] N1={.012533547935523,7.8957634722828,-8.7803203303561,.31802509345418,-.26145533859358,-.0078199751687981,.0088089493102134};
        private static readonly double[] D1={1,1,1,2,2,3,4}, T1={-.5,.875,1,.5,.75,.375,1};
        private static readonly double[] N2={-.66856572307965,.20433810950965,-.000066212605039687,-.19232721156002,-.25709043003438,.16074868486251,-.040092828925807,.00000039343422603254,-.0000075941377088144,.00056250979351888,-.000015608652257135,.0000000011537996422951,.00000036582165144204,-.0000000000013251180074668,-.00000000062639586912454,-.10793600908932,.017611491008752,.22132295167546,-.40247669763528,.58083399985759,.0049969146990806,-.031358700712549,-.74315929710341,.47807329915480,.020527940895948,-.13636435110343,.014180634400617,.0083326504880713,-.029052336009585,.038615085574206,-.020393486513704,-.0016554050063734,.0019955571979541,.00015870308324157,-.000016388568342530,.043613615723811,.034994005463765,-.076788197844621,.022446277332006,-.000062689710414685,-.00000000055711118565645,-.19905718354408,.31777497330738,-.11841182425981};
        private static readonly double[] D2={1,1,1,2,2,3,4,4,5,7,9,10,11,13,15,1,2,2,2,3,4,4,4,5,6,6,7,9,9,9,9,9,10,10,12,3,4,4,5,14,3,6,6,6};
        private static readonly double[] T2={4,6,12,1,5,4,2,13,9,3,4,11,4,13,1,7,1,9,10,10,3,7,10,10,6,10,10,1,2,3,4,8,6,9,8,16,22,23,23,10,50,44,46,50};
        private static readonly double[] C2={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,3,3,4,6,6,6,6};
        private static readonly double[] NI={.012436,.97315,1.27950,.96956,.24873}, GI={1.28728967,3.53734222,7.74073708,9.24437796,27.5075105};

        // Physics-ID: MAT-WATER-HELMHOLTZ
        // Implements: R013
        // Model: Specification.md section 5; IAPWS R6-95(2018) Eqs 4-6, Tables 1-3
        // Sources: D01
        // Inputs-SI: T [K], rho [kg/m3]; Outputs-SI: f and T/rho derivatives [SI]
        // State-Ownership: pure evaluation; Conservation: shared u=s=0 liquid triple-point reference
        // Discretisation: analytic forward derivatives, FP64; no field integration
        // Validity: evaluator algebra only; public providers apply narrower guards
        // Validation: V03 V32
        // Source-Locators: Official release equations and tables listed above.
        // Outputs-SI: Specific thermodynamic state or transport coefficient in SI.
        // Conservation: No transfers; shared IAPWS reference convention.
        // Evidence: PENDING
        // Status: IMPLEMENTED_UNVERIFIED
        // Limitations: S02 remains incomplete; see Docs/ThreePhase3D/S02-Handoff.md.
        internal static Jet2 Helmholtz(Jet2 t,Jet2 rho)
        {
            Jet2 d=rho/Rhoc, tau=Tc/t;
            Jet2 ideal=Jet2.Log(d)-8.3204464837497+6.6832105275932*tau+3.00632*Jet2.Log(tau);
            for(int i=0;i<NI.Length;i++)ideal+=NI[i]*Jet2.Log(1-Jet2.Exp(-GI[i]*tau));
            Jet2 residual=0;
            for(int i=0;i<N1.Length;i++)residual+=N1[i]*Jet2.Pow(d,D1[i])*Jet2.Pow(tau,T1[i]);
            for(int i=0;i<N2.Length;i++)residual+=N2[i]*Jet2.Pow(d,D2[i])*Jet2.Pow(tau,T2[i])*Jet2.Exp(-Jet2.Pow(d,C2[i]));
            double[] ng={-31.306260323435,31.546140237781,-2521.3154341695}, tg={0,1,4}, bg={150,150,250}, gg={1.21,1.21,1.25};
            for(int i=0;i<3;i++)residual+=ng[i]*Jet2.Pow(d,3)*Jet2.Pow(tau,tg[i])*Jet2.Exp(-20*(d-1)*(d-1)-bg[i]*(tau-gg[i])*(tau-gg[i]));
            Jet2 q=(d-1)*(d-1), theta=1-tau+.32*Jet2.Pow(q,1/(2*.3)), delta=theta*theta+.2*Jet2.Pow(q,3.5);
            residual+=-.14874640856724*d*Jet2.Pow(delta,.85)*Jet2.Exp(-28*q-700*(tau-1)*(tau-1));
            residual+=.31806110878444*d*Jet2.Pow(delta,.95)*Jet2.Exp(-32*q-800*(tau-1)*(tau-1));
            return R*t*(ideal+residual);
        }
        internal static ThermodynamicState FromHelmholtz(double t,double rho,Jet2 f)
        {
            double p=rho*rho*f.Y, dpdr=2*rho*f.Y+rho*rho*f.YY, dpdt=rho*rho*f.XY;
            double cv=-t*f.XX, cp=cv+t*dpdt*dpdt/(rho*rho*dpdr), u=f.V-t*f.X;
            return new ThermodynamicState(t,p,rho,u+p/rho,u,-f.X,cp,cv,Math.Sqrt(dpdr+t*dpdt*dpdt/(rho*rho*cv)),-dpdt/dpdr,1/dpdr);
        }
        // Raw homogeneous evaluator for official checks, not a stable phase selector.
        public ThermodynamicState ReferenceRhoT(double t,double rho)
        {
            Guard.Range(t,273.15,1273);Guard.Range(rho,1e-12,1300);
            if(Math.Abs(t-Tc)<.02 && Math.Abs(rho-Rhoc)<2)throw new ArgumentOutOfRangeException("Critical singular region excluded.");
            return FromHelmholtz(t,rho,Helmholtz(new Jet2(t,1),new Jet2(rho,0,1)));
        }
        public SaturationState Saturation(double t)
        {
            Guard.Range(t,273.16,625);
            double theta=1-t/Tc;
            double rl=Rhoc*(1+1.99274064*Math.Pow(theta,1.0/3)+1.09965342*Math.Pow(theta,2.0/3)-.510839303*Math.Pow(theta,5.0/3)-1.75493479*Math.Pow(theta,16.0/3)-45.5170352*Math.Pow(theta,43.0/3)-674694.45*Math.Pow(theta,110.0/3));
            double rv=Rhoc*Math.Exp(-2.0315024*Math.Pow(theta,1.0/3)-2.6830294*Math.Pow(theta,2.0/3)-5.38626492*Math.Pow(theta,4.0/3)-17.2991605*Math.Pow(theta,3)-44.7586581*Math.Pow(theta,18.5/3)-63.9201063*Math.Pow(theta,35.5/3));
            for(int i=0;i<30;i++)
            {
                // Ancillary liquid-density seeds can imply a small negative pressure near the
                // triple point. Evaluate Newton residuals directly; never expose such a trial as a state.
                Jet2 lf=Helmholtz(new Jet2(t,1),new Jet2(rl,0,1)),vf=Helmholtz(new Jet2(t,1),new Jet2(rv,0,1));
                double lp=rl*rl*lf.Y,vp=rv*rv*vf.Y;
                double fp=lp-vp,fg=lf.V+rl*lf.Y-vf.V-rv*vf.Y;
                // Dense-liquid pressure can oscillate at the cancellation floor.
                if(Math.Abs(fp)<Math.Max(1e-4,1e-10*vp)&&Math.Abs(fg)<2e-7)return new SaturationState(ReferenceRhoT(t,rl),ReferenceRhoT(t,rv));
                double ld=2*rl*lf.Y+rl*rl*lf.YY,vd=2*rv*vf.Y+rv*rv*vf.YY;
                double a=rl*ld,b=-rv*vd,c=ld,d=-vd,det=a*d-b*c;
                double dl=(-fp*d+b*fg)/det,dv=(-a*fg+c*fp)/det;
                if(Math.Abs(dl)>.1||Math.Abs(dv)>.1)throw new InvalidOperationException("Saturation Newton step left bracket neighbourhood.");
                rl*=Math.Exp(dl);rv*=Math.Exp(dv);
            }
            throw new InvalidOperationException("Saturation solve did not converge.");
        }
        public ThermodynamicState Evaluate(double t,double p,WaterBranch branch)
        {
            Guard.Range(t,273.16,600);Guard.Range(p,611.66,5e6);
            if(branch!=WaterBranch.Liquid&&branch!=WaterBranch.Vapour)throw new ArgumentOutOfRangeException(nameof(branch));
            var sat=Saturation(t);
            if((branch==WaterBranch.Liquid&&p<sat.PressurePa-2e-5)||(branch==WaterBranch.Vapour&&p>sat.PressurePa+2e-5))
                throw new ArgumentOutOfRangeException("Requested water branch is not stable at T,p.");
            double lo=branch==WaterBranch.Liquid?sat.Liquid.DensityKgM3:1e-12;
            double hi=branch==WaterBranch.Liquid?1300:sat.Vapour.DensityKgM3;
            double rho=branch==WaterBranch.Liquid?lo:Math.Min(hi,p/(R*t));
            for(int i=0;i<70;i++)
            {
                var s=ReferenceRhoT(t,rho);double e=s.PressurePa-p;
                if(Math.Abs(e)<Math.Max(3e-5,p*1e-11))return s;
                if(e>0)hi=rho;else lo=rho;
                double next=rho-e*s.DensityPressureDerivative;
                rho=next>lo&&next<hi?next:(lo+hi)/2;
            }
            throw new InvalidOperationException("Water density inversion did not converge.");
        }
        public double ViscosityPaS(ThermodynamicState state)=>WaterTransport.Viscosity(state);
        public double ConductivityWmK(ThermodynamicState state)=>WaterTransport.Conductivity(state);
        public double SurfaceTensionNm(double t)=>WaterTransport.SurfaceTension(t);
    }
}
