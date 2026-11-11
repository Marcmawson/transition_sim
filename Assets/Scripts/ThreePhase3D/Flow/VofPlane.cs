using System;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Flow
{
    internal readonly struct VofPlane
    {
        private readonly D3 normal;
        private readonly double intercept;
        private VofPlane(D3 normal, double intercept) { this.normal = normal; this.intercept = intercept; }

        internal static VofPlane Reconstruct(D3 normal, double fraction)
        {
            double scale = Math.Abs(normal.X) + Math.Abs(normal.Y) + Math.Abs(normal.Z);
            normal = scale == 0 ? new D3(1, 0, 0) : new D3(normal.X / scale, normal.Y / scale, normal.Z / scale);
            double lo = Math.Min(0, normal.X) + Math.Min(0, normal.Y) + Math.Min(0, normal.Z);
            double hi = lo + 1;
            for (int i = 0; i < 48; i++)
            {
                double mid = .5 * (lo + hi);
                if (Fraction(normal, mid) < fraction) lo = mid;
                else hi = mid;
            }
            return new VofPlane(normal, .5 * (lo + hi));
        }

        internal double Swept(int axis, bool positive, double width)
        {
            var scaled = new D3(axis == 0 ? normal.X * width : normal.X,
                axis == 1 ? normal.Y * width : normal.Y, axis == 2 ? normal.Z * width : normal.Z);
            return width * Fraction(scaled, intercept - (positive ? normal[axis] * (1 - width) : 0));
        }

        // After reflecting negative axes, the cut fraction is the CDF of a
        // sum of three scaled unit uniforms. Integrate the piecewise-quadratic
        // two-axis CDF over the shortest axis using exact Simpson intervals.
        internal static double Fraction(D3 n, double intercept)
        {
            double t = intercept - Math.Min(0, n.X) - Math.Min(0, n.Y) - Math.Min(0, n.Z);
            double a = Math.Abs(n.X), b = Math.Abs(n.Y), c = Math.Abs(n.Z);
            if (a > b) Swap(ref a, ref b);
            if (b > c) Swap(ref b, ref c);
            if (a > b) Swap(ref a, ref b);
            if (t <= 0) return 0;
            if (t >= a + b + c) return 1;
            if (a == 0) return Two(t, b, c);

            double p = Unit((t - b - c) / a), q = Unit((t - c) / a);
            double r = Unit((t - b) / a), s = Unit(t / a);
            return Integrate(0, p, t, a, b, c) + Integrate(p, q, t, a, b, c)
                + Integrate(q, r, t, a, b, c) + Integrate(r, s, t, a, b, c);
        }

        private static double Integrate(double lo, double hi, double t, double a, double b, double c)
        {
            if (lo == hi) return 0;
            return (hi - lo) / 6 * (Two(t - a * lo, b, c) + 4 * Two(t - a * .5 * (lo + hi), b, c)
                + Two(t - a * hi, b, c));
        }

        private static double Two(double t, double b, double c)
        {
            if (t <= 0) return 0;
            if (t >= b + c) return 1;
            if (t < b) return .5 * (t / b) * (t / c);
            if (t <= c) return (t - .5 * b) / c;
            double tail = b + c - t;
            return 1 - .5 * (tail / b) * (tail / c);
        }

        private static double Unit(double value) => Math.Max(0, Math.Min(1, value));
        private static void Swap(ref double a, ref double b) { double old = a; a = b; b = old; }
    }
}
