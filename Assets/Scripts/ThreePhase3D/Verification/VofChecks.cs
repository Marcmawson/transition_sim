using System;
using System.Collections.Generic;
using System.Linq;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;

namespace PhysicsSim.ThreePhase3D.Verification
{
    public sealed class VofChecks
    {
        public delegate VofAdvance Advance(GeometricVof3D vof, VofFields state, double[][] velocity, double dt);
        private readonly Advance advance;
        public VofChecks(Advance advance = null)
        {
            this.advance = advance ?? ((vof, state, velocity, dt) => vof.Advance(state, velocity, dt));
        }

        private const double LiquidDensity = 997, GasDensity = 1.2;
        private readonly List<FlowCheckRow> rows = new List<FlowCheckRow>();
        public IReadOnlyList<FlowCheckRow> Rows => rows;
        public bool Passed => rows.Count > 0 && rows.All(row => row.Passed);

        public void Run()
        {
            Check("PLIC", "axis-aligned cut", VofPlane.Fraction(new D3(1, 0, 0), .3), .3, 1e-14);
            Check("PLIC", "diagonal tetrahedral cut", VofPlane.Fraction(new D3(1, 1, 1), .6), .036, 1e-14);
            Check("PLIC", "negative orientation complement", VofPlane.Fraction(new D3(-1, -1, -1), -.6), .964, 1e-14);
            Check("PLIC", "two-axis near-degenerate cut", VofPlane.Fraction(new D3(1, 1, 1e-18), .4), .08, 1e-14);
            var plane = VofPlane.Reconstruct(new D3(-1, .3, .5), .37);
            Check("PLIC", "swept full-cell reconstructs fraction", plane.Swept(0, true, 1), .37, 1e-13);
            PressureFreeMomentum();
            double coarse = Translate(12), fine = Translate(24);
            Check("translation", "fine/coarse shape error ratio", fine / coarse, 0, .85);
            DeformAndReverse();
        }

        private void PressureFreeMomentum()
        {
            const string id = "pressure-free momentum";
            var grid = new Grid3D(4, 1, 1, new D3(1, 1, 1));
            var vof = new GeometricVof3D(grid, LiquidDensity, GasDensity);
            double[] alpha = { 1, 0, 0, 1 };
            var transport = Uniform(4, .25, 0, 0);
            var state = Fields(grid, alpha, transport);
            state.FaceVelocity[0][0] = 3;
            state.FaceVelocity[0][1] = -2;
            var step = advance(vof, state, transport, 1);
            Check(id, "liquid transfers across first face [m3]", step.LiquidTransferM3[0][0], .25, 1e-14);
            Check(id, "gas transfers across second face [m3]", step.GasTransferM3[0][1], .25, 1e-14);
            double dualInitial = .5 * (LiquidDensity + GasDensity);
            double incoming = .25 * LiquidDensity;
            double outgoing = .125 * (LiquidDensity + GasDensity);
            Check(id, "first face updated momentum [kg m/s]", step.MomentumKgMPerS[0][0],
                3 * dualInitial + incoming * state.FaceVelocity[0][3] - outgoing * 3, 1e-10);
            Check(id, "mass and momentum share dual transfer [kg]", step.DualMassErrorKg, 0, 1e-10);
            try
            {
                advance(vof, state, transport, 3);
                Check(id, "CFL rejection", 0, 1, 0);
            }
            catch (ArgumentException) { Check(id, "CFL rejection", 1, 1, 0); }
            Check(id, "accepted state preserved", state.LiquidFraction.Sum(), 2, 0);
        }

        private double Translate(int n)
        {
            string id = "translation-" + n;
            var grid = new Grid3D(n, n, n, new D3(1.0 / n, 1.0 / n, 1.0 / n));
            var vof = new GeometricVof3D(grid, LiquidDensity, GasDensity);
            var u = Uniform(vof.CellCount, .3, -.2, .1);
            const double elapsed = .5;
            var centre = new D3(.39, .46, .49);
            var initial = Fields(grid, Sphere(vof, centre, .22), u);
            int steps = (int)Math.Ceiling(elapsed * .6 * n / .35);
            var state = initial;
            double dual = 0, reconcile = 0, limiter = 0;
            for (int s = 0; s < steps; s++)
            {
                var result = advance(vof, state, u, elapsed / steps);
                dual = Math.Max(dual, result.DualMassErrorKg);
                reconcile += result.RoundoffVolumeM3;
                limiter += result.LimitedVolumeM3;
                state = result.Fields;
            }
            var expected = Sphere(vof, centre + new D3(.3, -.2, .1) * elapsed, .22);
            double error = ShapeError(state.LiquidFraction, expected);
            Check(id, "relative L1 shape error", error, 0, n == 12 ? .40 : .24);
            Check(id, "uniform staggered velocity preservation [m/s]",
                Enumerable.Range(0, 3).Max(axis => state.FaceVelocity[axis].Zip(u[axis], (x, y) => Math.Abs(x - y)).Max()), 0, 2e-11);
            Ledger(id, grid, initial, state, dual, reconcile, limiter);
            return error;
        }

        private void DeformAndReverse()
        {
            const string id = "deform-reverse";
            const int n = 16, steps = 12;
            const double dt = .025;
            var grid = new Grid3D(n, n, n, new D3(1.0 / n, 1.0 / n, 1.0 / n));
            var vof = new GeometricVof3D(grid, LiquidDensity, GasDensity);
            var transport = Uniform(vof.CellCount, 0, 0, 0);
            for (int k = 0; k < n; k++)
            for (int j = 0; j < n; j++)
            for (int i = 0; i < n; i++)
            {
                int c = vof.CellIndex(i, j, k);
                // A periodic shear composition with a discrete divergence of zero.
                // Each component is independent of its own coordinate.
                transport[0][c] = .22 * Math.Sin(2 * Math.PI * (j + .5) / n);
                transport[1][c] = .18 * Math.Sin(2 * Math.PI * (k + .5) / n);
                transport[2][c] = .15 * Math.Sin(2 * Math.PI * (i + .5) / n);
            }
            var initial = Fields(grid, Sphere(vof, new D3(.43, .40, .46), .23), transport);
            var state = initial;
            double dual = 0, reconcile = 0, limiter = 0, midpoint = 0;
            var backward = transport.Select(axis => axis.Select(x => -x).ToArray()).ToArray();
            for (int s = 0; s < 2 * steps; s++)
            {
                var result = advance(vof, state, s < steps ? transport : backward, dt);
                dual = Math.Max(dual, result.DualMassErrorKg);
                reconcile += result.RoundoffVolumeM3;
                limiter += result.LimitedVolumeM3;
                state = result.Fields;
                if (s == steps - 1) midpoint = ShapeError(state.LiquidFraction, initial.LiquidFraction);
            }
            double returned = ShapeError(state.LiquidFraction, initial.LiquidFraction);
            Check(id, "reversal error smaller than forward deformation", returned < midpoint ? 1 : 0, 1, 0);
            Check(id, "relative L1 reversal error", returned, 0, .4);
            Ledger(id, grid, initial, state, dual, reconcile, limiter);
        }

        private void Ledger(string id, Grid3D grid, VofFields start, VofFields end, double dual, double reconcile, double limiter)
        {
            double v = grid.CellVolume;
            Check(id, "minimum liquid fraction bound", Math.Max(0, -end.LiquidFraction.Min()), 0, 0);
            Check(id, "maximum liquid fraction bound", Math.Max(0, end.LiquidFraction.Max() - 1), 0, 0);
            Check(id, "liquid volume drift [m3]", end.LiquidFraction.Sum() * v - start.LiquidFraction.Sum() * v, 0, 1e-12);
            Check(id, "mixture mass drift [kg]", (end.LiquidFraction.Sum() - start.LiquidFraction.Sum()) * v * (LiquidDensity - GasDensity), 0, 1e-9);
            Check(id, "vapour mass drift [kg]", end.VapourKg.Sum() - start.VapourKg.Sum(), 0, 1e-12);
            Check(id, "liquid energy drift [J]", end.LiquidEnergyJ.Sum() - start.LiquidEnergyJ.Sum(), 0, 1e-9);
            Check(id, "gas energy drift [J]", end.GasEnergyJ.Sum() - start.GasEnergyJ.Sum(), 0, 1e-9);
            Check(id, "max dual mass error [kg]", dual, 0, 1e-11);
            Check(id, "cumulative absolute volume reconciliation [m3]", reconcile, 0, 1e-12);
            Check(id, "cumulative limited geometric transfer [m3]", limiter, 0, .03);
            for (int a = 0; a < 3; a++)
                Check(id, "global momentum drift axis " + a + " [kg m/s]", Momentum(grid, end, a) - Momentum(grid, start, a), 0, 1e-9);
        }

        private static double Momentum(Grid3D grid, VofFields state, int axis)
        {
            var shape = grid.Cells;
            double total = 0;
            for (int k = 0; k < shape.Z; k++)
            for (int j = 0; j < shape.Y; j++)
            for (int i = 0; i < shape.X; i++)
            {
                int cell = (k * shape.Y + j) * shape.X + i;
                int neighbour = (((k + (axis == 2 ? 1 : 0)) % shape.Z) * shape.Y +
                    (j + (axis == 1 ? 1 : 0)) % shape.Y) * shape.X + (i + (axis == 0 ? 1 : 0)) % shape.X;
                double liquid = .5 * (state.LiquidFraction[cell] + state.LiquidFraction[neighbour]);
                total += grid.CellVolume * (LiquidDensity * liquid + GasDensity * (1 - liquid)) * state.FaceVelocity[axis][cell];
            }
            return total;
        }

        internal static VofFields Fields(Grid3D grid, double[] alpha, double[][] velocity)
        {
            double volume = grid.CellVolume;
            return new VofFields((double[])alpha.Clone(), velocity.Select(axis => (double[])axis.Clone()).ToArray(),
                alpha.Select((x, i) => x * volume * LiquidDensity * (1 + .02 * (i % 7))).ToArray(),
                alpha.Select((x, i) => (1 - x) * volume * GasDensity * (.5 + .01 * (i % 9))).ToArray(),
                alpha.Select((x, i) => (1 - x) * volume * GasDensity * (.1 + .01 * (i % 5))).ToArray());
        }

        internal static double[][] Uniform(int size, double x, double y, double z) =>
            new[] { Enumerable.Repeat(x, size).ToArray(), Enumerable.Repeat(y, size).ToArray(), Enumerable.Repeat(z, size).ToArray() };

        private static double ShapeError(double[] state, double[] expected) =>
            state.Zip(expected, (x, y) => Math.Abs(x - y)).Sum() / expected.Sum();

        internal static double[] Sphere(GeometricVof3D vof, D3 centre, double radius)
        {
            var grid = vof.Grid;
            var alpha = new double[vof.CellCount];
            double halfDiagonal = grid.Spacing.Length * .5;
            const int samples = 8;
            for (int k = 0; k < grid.Cells.Z; k++)
            for (int j = 0; j < grid.Cells.Y; j++)
            for (int i = 0; i < grid.Cells.X; i++)
            {
                int cell = vof.CellIndex(i, j, k);
                var lower = grid.Lower(i, j, k) - centre;
                double d = (lower + grid.Spacing * .5).Length;
                if (d >= radius + halfDiagonal) continue;
                if (d <= radius - halfDiagonal) { alpha[cell] = 1; continue; }
                int count = 0;
                for (int c = 0; c < samples; c++)
                for (int b = 0; b < samples; b++)
                for (int a = 0; a < samples; a++)
                {
                    var p = lower + new D3((a + .5) * grid.Spacing.X / samples,
                        (b + .5) * grid.Spacing.Y / samples, (c + .5) * grid.Spacing.Z / samples);
                    if (D3.Dot(p, p) < radius * radius) count++;
                }
                alpha[cell] = (double)count / (samples * samples * samples);
            }
            return alpha;
        }

        private void Check(string id, string quantity, double actual, double expected, double tolerance)
        {
            rows.Add(new FlowCheckRow { Case = id, Quantity = quantity, Actual = actual, Expected = expected,
                Tolerance = tolerance, Passed = SiUnits.IsFinite(actual) && Math.Abs(actual - expected) <= tolerance });
        }
    }
}
