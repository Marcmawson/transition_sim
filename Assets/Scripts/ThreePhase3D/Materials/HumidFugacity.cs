using System;

namespace PhysicsSim.ThreePhase3D.Materials
{
    public static class HumidFugacity
    {
        public const double R=8.3144621;
        // Physics-ID: MAT-HUMID-G11-FUGACITY
        // Implements: R015
        // Model: IAPWS G11-15 Eqs2-11 Tables1-4
        // Sources: D07
        // Source-Locators: G11-15 Tables7-8 numerical verification
        // Inputs-SI: T [K], p [Pa], water mole fraction x [mol/mol]
        // Outputs-SI: fugacity [Pa], B [m3/mol], C [m6/mol2]
        // State-Ownership: stateless reference
        // Conservation: no phase/species transfer
        // Discretisation: explicit second/third virial FP64 algebra
        // Validity: 193-473 K, 0<p<=5 MPa; production composition must be stable
        // Validation: V05 V32
        // Evidence: PENDING
        // Status: IMPLEMENTED_UNVERIFIED
        // Limitations: ReferenceFugacity also accepts unstable Table8 check states;
        // never use it to replace a stable-phase selector or a kinetic interface law.
        public static double[] VirialCoefficients(double t)
        {
            Guard.Range(t,193,473);double tau=647.096/t,ta=132.6312/t,q=t/100;
            double[] nb={.012533547935523,7.8957634722828,-8.7803203303561,-.66856572307965,.20433810950965,-.000066212605039687,-.10793600908932};
            double[] tb={-.5,.875,1,4,6,12,7};
            double[] nc={.31802509345418,-.26145533859358,-.19232721156002,-.25709043003438,.017611491008752,.22132295167546,-.40247669763528};
            double[] tc={.5,.75,1,5,1,9,10};double bw=0,cw=0;
            for(int i=0;i<nb.Length;i++)bw+=nb[i]*Math.Pow(tau,tb[i]);
            for(int i=0;i<nc.Length;i++)cw+=nc[i]*Math.Pow(tau,tc[i]);
            for(int i=3;i<6;i++)cw-=nb[i]*Math.Pow(tau,tb[i]);
            double theta=1-tau+.32,delta=theta*theta+.2,deltaPrime=-2*theta*.32/.3-2*3.5*.2;
            double[] n={-.14874640856724,.31806110878444},b={.85,.95},c={28,32},d={700,800};
            for(int i=0;i<2;i++)
            {
                double e=n[i]*Math.Exp(-c[i]-d[i]*(tau-1)*(tau-1));
                bw+=e*Math.Pow(delta,b[i]);
                cw+=e*(b[i]*Math.Pow(delta,b[i]-1)*deltaPrime+2*c[i]*Math.Pow(delta,b[i]));
            }
            double scale=HumidGas.Mw/322;bw*=scale;cw*=2*scale*scale;
            double ba=(.118160747229+.713116392079*Math.Pow(ta,.33)-1.61824192067*Math.Pow(ta,1.01)-.101365037912*Math.Pow(ta,1.6)-.146629609713*Math.Pow(ta,3.6)+.0148287891978*Math.Pow(ta,3.5))/10447.7;
            double ca=2*(.0714140178971+.101365037912*Math.Pow(ta,1.6))/(10447.7*10447.7);
            double baw=1e-6*(66.5687*Math.Pow(q,-.237)-238.834*Math.Pow(q,-1.048)-176.755*Math.Pow(q,-3.183));
            double caaw=1e-6*(.000482737+.00105678/q-.00656394/(q*q)+.0294442/Math.Pow(q,3)-.0319317/Math.Pow(q,4));
            double caww=-1e-6*Math.Exp(-10.728876+34.7802/q-38.3383/(q*q)+33.406/Math.Pow(q,3));
            return new[]{ba,baw,bw,ca,caaw,caww,cw};
        }
        public static double[] Auxiliary(double t,double x)
        {
            Guard.Range(x,0,1);var v=VirialCoefficients(t);double a=1-x;
            double bm=a*a*v[0]+2*a*x*v[1]+x*x*v[2];
            double bp=-2*a*v[0]+2*(1-2*x)*v[1]+2*x*v[2];
            double cm=a*a*a*v[3]+3*a*a*x*v[4]+3*a*x*x*v[5]+x*x*x*v[6];
            double cp=-3*a*a*v[3]+3*(a*a-2*a*x)*v[4]+3*(2*a*x-x*x)*v[5]+3*x*x*v[6];
            return new[]{bm+a*bp,cm+a*cp-bm*bm-2*a*bm*bp};
        }
        public static double ReferenceFugacity(double t,double p,double x)
        {Guard.Range(p,double.Epsilon,5e6);var v=Auxiliary(t,x);double z=p/(R*t);return x*p*Math.Exp(v[0]*z+.5*v[1]*z*z);}
        public static double StableFugacity(double t,double p,double waterMassFraction)
        {new HumidGas().Evaluate(t,p,waterMassFraction);return ReferenceFugacity(t,p,HumidGas.WaterMoleFraction(waterMassFraction));}
    }
}
