using System;
using System.Collections.Generic;
using System.Linq;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Geometry;

namespace PhysicsSim.ThreePhase3D.Verification
{
    public sealed class VofChecks
    {
        public delegate VofAdvance Advance(GeometricVof3D vof, VofFields state, double[][] velocity, double dt);
        public delegate CutCellTransportResult CutAdvance(VariableCoefficientProjection plan, CutCellInventories state,
            double[] velocity, double dt, Func<int, CutCellInflow> inflow);
        public delegate CutCellMomentumResult CutMomentumAdvance(VariableCoefficientProjection plan, CutCellInventories before,
            CutCellTransportResult after, double[] velocity, double liquidDensity, double gasDensity);
        private readonly CutMomentumAdvance cutMomentumAdvance;
        private readonly Advance advance;
        private readonly CutAdvance cutAdvance, cutClipAdvance;
        private readonly PeriodicVofProjection.PressureSolve pressureSolve;
        public VofChecks(Advance advance = null) : this(advance, null) { }

        public VofChecks(Advance advance, PeriodicVofProjection.PressureSolve pressureSolve, CutAdvance cutAdvance = null, CutMomentumAdvance cutMomentumAdvance = null, CutAdvance cutClipAdvance = null)
        {
            this.cutAdvance = cutAdvance;
            this.cutClipAdvance = cutClipAdvance;
            this.cutMomentumAdvance = cutMomentumAdvance;
            this.pressureSolve = pressureSolve ?? FlowCpuReference.Project;
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
            CoupledPressure();
            CutCellBoundaryTransport();
            ClosedCutCirculation();
            BeakerCutTransport();
        }

        private void CoupledPressure()
        {
            const string id = "coupled-periodic-pressure";
            const int n = 6;
            var grid = new Grid3D(n, n, n, new D3(1.0/n, 1.0/n, 1.0/n));
            var vof = new GeometricVof3D(grid, LiquidDensity, GasDensity);
            var coupled = new PeriodicVofProjection(grid, LiquidDensity, GasDensity);
            var velocity = Uniform(vof.CellCount, 0, 0, 0);
            for (int k = 0; k < n; k++)
            for (int j = 0; j < n; j++)
            for (int i = 0; i < n; i++)
            {
                int c = vof.CellIndex(i, j, k);
                velocity[0][c] = .12 * Math.Sin(2 * Math.PI * j / n);
                velocity[1][c] = .10 * Math.Sin(2 * Math.PI * i / n);
            }
            var start = Fields(grid, Sphere(vof, new D3(.44, .43, .48), .24), velocity);
            var first = coupled.Advance(start, .04, 4096, (fields, u, dt) => advance(vof, fields, u, dt), pressureSolve);
            Check(id, "transport generates a nontrivial pressure correction", first.Pressure.Iterations > 0 ? 1 : 0, 1, 0);
            Check(id, "pressure relative residual", first.Pressure.ResidualL2 / first.Pressure.RhsL2, 0, 1e-6);
            Check(id, "flux reconciliation [Courant]", first.MaxFluxCorrectionCourant, 0, 1e-6);
            var second = coupled.Advance(first.Fields, .04, 4096, (fields, u, dt) => advance(vof, fields, u, dt), pressureSolve);
            Check(id, "second transport conserves liquid volume [m3]",
                (second.Fields.LiquidFraction.Sum() - start.LiquidFraction.Sum()) * grid.CellVolume, 0, 1e-12);
        }

        private void CutCellBoundaryTransport()
        {
            const string id = "cut-cell open boundary";
            var grid = new Grid3D(4, 1, 1, new D3(1,1,1));
            var slab = SurfacePrimitives.Box(new D3(-1,-1,-1), new D3(5,.4,2));
            var geometry = CutCellGeometry.Build(grid, slab);
            var clipped = new CutCellVofGeometry(geometry);
            int sample = grid.Cells.Index(1,0,0);
            var plane = clipped.Reconstruct(sample, new D3(0,1,0), .3);
            Check(id, "clipped liquid inventory [m3]", clipped.LiquidVolume(sample, plane), .3, 1e-10);
            Check(id, "clipped liquid sweep [m3]", clipped.Sweep(sample, 0, true, .06, plane), .03, 1e-10);
            var open = new FlowBoundary(FlowBoundaryKind.OpenPressure);
            var wall = new FlowBoundary(FlowBoundaryKind.NoSlip);
            var plan = new VariableCoefficientProjection(geometry,
                new BoundaryConditions3D(open,open,wall,wall,wall,wall), _ => 1000, _ => 0);
            var transport = new CutCellVofTransport(plan);
            double[] liquid = plan.Volumes.Select((v,i) => v * (.2 + .15 * i)).ToArray();
            double[] gas = plan.Volumes.Zip(liquid, (v,l) => v-l).ToArray();
            var state = new CutCellInventories(liquid, gas, liquid.Select(v => 6*v).ToArray(),
                gas.Select(v => 3*v).ToArray(), gas.Select(v => .1*v).ToArray());
            var u = new double[plan.FaceCount];
            for (int i = 0; i <= 4; i++)
            {
                int face = plan.FaceIndex(0,i,0,0);
                u[face] = .06 / plan.Faces[face].Area;
            }
            var result = TransportCut(id, plan, transport, state,u,1,_ => new CutCellInflow(0,0,4,.2,new D3(u[plan.FaceIndex(0,0,0,0)],.2,0)));
            int inlet = plan.FaceIndex(0,0,0,0);
            Check(id, "ambient gas inlet [m3]", result.GasFaceM3[inlet], .06, 1e-12);
            for (int field = 0; field < 5; field++)
                Check(id, "field ledger " + field, result.Fields.Planes[field].Sum() -
                    state.Planes[field].Sum() - result.BoundaryInflow[field], 0, 1e-10);
            Check(id, "roundoff accounting [m3]", result.RoundoffM3, 0, 1e-10);
            var updatedMomentum = TransportCutMomentum(id, plan, transport, state, result, u);
            Check(id, "staggered phase mass closes [kg]", updatedMomentum.MaxDualMassErrorKg, 0, 1e-10);
            Check(id, "transverse inflow momentum reaches wall ledger [kg m/s]",
                updatedMomentum.WallImpulseKgMPerS[1], -GasDensity * .06 * .2, 1e-12);
            Check(id, "uniform velocity with open-boundary mass transfer [m/s]",
                updatedMomentum.VelocityMPerS.Zip(u, (a,b) => Math.Abs(a-b)).Max(), 0, 2e-10);
            double beforeMomentum = state.LiquidM3.Zip(state.GasM3, (l,g) => LiquidDensity*l+GasDensity*g).Sum() * u[inlet];
            Check(id, "staggered momentum follows boundary ledger [kg m/s]", updatedMomentum.MomentumKgMPerS.Sum() -
                beforeMomentum - updatedMomentum.BoundaryMomentumKgMPerS[0] - updatedMomentum.WallImpulseKgMPerS[0], 0, 1e-9);
            var pressure = pressureSolve(plan, u.Select(v => (float)v).ToArray(), 1, 4096);
            var projected = transport.AdvanceProjected(state, pressure, _ => new CutCellInflow(0,0,4,.2));
            Check(id, "S04 projection feeds cut-cell scalar transport [m3]",
                projected.Fields.LiquidM3.Sum() - state.LiquidM3.Sum() - projected.BoundaryInflow[0], 0, 1e-10);
            Check(id, "pressure flux reconciliation [m3]", projected.PressureFluxCorrectionM3, 0, 1e-6);
            var coupled = transport.AdvanceCoupled(state, pressure, LiquidDensity, GasDensity,
                _ => new CutCellInflow(0,0,4,.2,new D3(u[inlet],0,0)), 4096, pressureSolve);
            Check(id, "advected cut-cell density feeds the next pressure solve", coupled.Pressure.Diagnostics.Converged ? 1 : 0, 1, 0);
            u[plan.FaceIndex(1,0,0,0)] = .1;
            bool rejected = false;
            try { transport.Advance(state,u,1,_ => new CutCellInflow(0,0,4,.2)); }
            catch (ArgumentException) { rejected = true; }
            Check(id, "closed wall rejects normal transport", rejected ? 1 : 0, 1, 0);
        }

        private void ClosedCutCirculation()
        {
            const string id = "cut-cell closed circulation";
            var grid = new Grid3D(2, 1, 2, new D3(1,1,1));
            var slab = SurfacePrimitives.Box(new D3(-1,-1,-1), new D3(3,.4,3));
            var plan = new VariableCoefficientProjection(CutCellGeometry.Build(grid, slab),
                BoundaryConditions3D.Closed, _ => 1000, _ => 0);
            var solver = new CutCellVofTransport(plan);
            var liquid = plan.Volumes.Select((v,i) => v * (.2 + .15 * i)).ToArray();
            var gas = plan.Volumes.Zip(liquid, (v,l) => v-l).ToArray();
            var state = new CutCellInventories(liquid, gas, liquid.Select(v => 6*v).ToArray(),
                gas.Select(v => 3*v).ToArray(), gas.Select(v => .1*v).ToArray());
            var u = new double[plan.FaceCount];
            Set(0,1,0,0, .06); Set(2,1,0,1, .06); Set(0,1,0,1,-.06); Set(2,0,0,1,-.06);
            var transport = TransportCut(id, plan, solver, state,u,1);
            var momentum = TransportCutMomentum(id, plan, solver, state, transport, u);
            Check(id, "liquid volume closes [m3]", transport.Fields.LiquidM3.Sum() - liquid.Sum(), 0, 1e-10);
            Check(id, "closed boundary ledger [m3]", transport.BoundaryInflow.Select(Math.Abs).Max(), 0, 0);
            Check(id, "staggered mass closes [kg]", momentum.MaxDualMassErrorKg, 0, 1e-10);
            Check(id, "staggered velocity evolves under transported momentum",
                momentum.VelocityMPerS.Zip(u,(a,b) => Math.Abs(a-b)).Max() > 1e-4 ? 1 : 0, 1, 0);

            void Set(int axis, int i, int j, int k, double q)
            {
                int face = plan.FaceIndex(axis,i,j,k);
                u[face] = q / plan.Faces[face].Area;
            }
        }

        private void BeakerCutTransport()
        {
            const string id = "beaker cut-cell transport";
            var frame = new RigidFrame(new D3(-.08,-.025,-.08), new D3(1,0,0), new D3(0,1,0), new D3(0,0,1));
            var grid = new Grid3D(16, 13, 16, new D3(.01,.01,.01), frame);
            var beaker = ApparatusSurface.Beaker(new GeometryInputs
            {
                innerRadiusM = .033, wallThicknessM = .031, baseThicknessM = .031, cavityHeightM = .06, externalBaseHeightM = 0
            }, 24);
            var geometry = CutCellGeometry.Build(grid, beaker);
            var plan = new VariableCoefficientProjection(geometry, BoundaryConditions3D.Closed, _ => LiquidDensity, _ => 0);
            var loop = FindLoop();
            Check(id, "resolved circulation crosses a curved cut cell", loop != null ? 1 : 0, 1, 0);
            if (loop == null) return;
            var liquid = plan.Volumes.Select((v,i) => v * (.2 + .1 * (i % 4))).ToArray();
            var gas = plan.Volumes.Zip(liquid, (v,l) => v-l).ToArray();
            var state = new CutCellInventories(liquid, gas, liquid.Select(v => 6*v).ToArray(),
                gas.Select(v => 3*v).ToArray(), gas.Select(v => .1*v).ToArray());
            var u = new double[plan.FaceCount];
            for (int s = 0; s < 4; s++)
                u[loop[s]] = (s < 2 ? 1 : -1) * 1e-9 / plan.Faces[loop[s]].Area;
            var transport = new CutCellVofTransport(plan);
            var result = TransportCut(id, plan, transport, state,u,1);
            var momentum = TransportCutMomentum(id, plan, transport, state, result, u);
            Check(id, "phase volume closes through curved cut cells [m3]", result.Fields.LiquidM3.Sum() - liquid.Sum(), 0, 1e-13);
            Check(id, "staggered mass closes [kg]", momentum.MaxDualMassErrorKg, 0, 1e-12);

            int[] FindLoop()
            {
                for (int j = 0; j < grid.Cells.Y; j++)
                for (int k = 4; k < 11; k++)
                for (int i = 4; i < 11; i++)
                {
                    var cells = new[] { grid.Cells.Index(i,j,k), grid.Cells.Index(i+1,j,k),
                        grid.Cells.Index(i+1,j,k+1), grid.Cells.Index(i,j,k+1) };
                    if (cells.Any(c => geometry.FluidVolumeM3[c] < .15 * grid.CellVolume) ||
                        cells.All(c => geometry.FluidVolumeM3[c] >= .999 * grid.CellVolume) ||
                        cells.Select(c => plan.GroupByCell[c]).Distinct().Count() != 4) continue;
                    var faces = new[] { plan.FaceIndex(0,i+1,j,k), plan.FaceIndex(2,i+1,j,k+1),
                        plan.FaceIndex(0,i+1,j,k+1), plan.FaceIndex(2,i,j,k+1) };
                    if (faces.All(f => plan.Faces[f].Active != 0)) return faces;
                }
                return null;
            }
        }

        private CutCellMomentumResult TransportCutMomentum(string id, VariableCoefficientProjection plan,
            CutCellVofTransport reference, CutCellInventories before, CutCellTransportResult after, double[] velocity)
        {
            var expected = reference.AdvanceMomentum(before, after, velocity, LiquidDensity, GasDensity);
            if (cutMomentumAdvance == null) return expected;
            var actual = cutMomentumAdvance(plan, before, after, velocity, LiquidDensity, GasDensity);
            Compare("velocity [m/s]", actual.VelocityMPerS, expected.VelocityMPerS);
            Compare("staggered momentum [kg m/s]", actual.MomentumKgMPerS, expected.MomentumKgMPerS);
            Compare("boundary momentum [kg m/s]", actual.BoundaryMomentumKgMPerS, expected.BoundaryMomentumKgMPerS);
            Compare("wall impulse [kg m/s]", actual.WallImpulseKgMPerS, expected.WallImpulseKgMPerS);
            return actual;

            void Compare(string quantity, double[] a, double[] b) =>
                Check(id, "GPU vs CPU " + quantity, a.Zip(b, (x,y) => Math.Abs(x-y)).Max(),
                    0, 1e-13 + 1e-10 * b.Select(Math.Abs).Max());
        }

        private CutCellTransportResult TransportCut(string id, VariableCoefficientProjection plan, CutCellVofTransport reference,
            CutCellInventories state, double[] velocity, double dt, Func<int, CutCellInflow> inflow = null)
        {
            var expected = reference.Advance(state, velocity, dt, inflow);
            if (cutClipAdvance != null)
            {
                var clipped = cutClipAdvance(plan, state, velocity, dt, inflow);
                double faceTolerance = 2e-6 * plan.Grid.CellVolume;
                Check(id, "GPU PLIC liquid flux [m3]", clipped.LiquidFaceM3.Zip(expected.LiquidFaceM3,
                    (x,y) => Math.Abs(x-y)).Max(), 0, faceTolerance);
                Check(id, "GPU PLIC gas flux [m3]", clipped.GasFaceM3.Zip(expected.GasFaceM3,
                    (x,y) => Math.Abs(x-y)).Max(), 0, faceTolerance);
                Check(id, "GPU PLIC liquid inventory ledger [m3]", clipped.Fields.LiquidM3.Sum() -
                    state.LiquidM3.Sum() - clipped.BoundaryInflow[0], 0, 1e-10 * plan.Grid.CellVolume);
            }
            if (cutAdvance == null) return expected;
            var actual = cutAdvance(plan, state, velocity, dt, inflow);
            for (int plane = 0; plane < 5; plane++)
                Compare("inventory " + plane, actual.Fields.Planes[plane], expected.Fields.Planes[plane]);
            Compare("liquid face flux", actual.LiquidFaceM3, expected.LiquidFaceM3);
            Compare("gas face flux", actual.GasFaceM3, expected.GasFaceM3);
            Compare("open boundary ledger", actual.BoundaryInflow, expected.BoundaryInflow);
            return actual;

            void Compare(string quantity, double[] a, double[] b)
            {
                double scale = Math.Max(1e-15, b.Select(Math.Abs).Max());
                Check(id, "GPU vs CPU " + quantity, a.Zip(b, (x,y) => Math.Abs(x-y)).Max(), 0, 1e-10 * scale);
            }
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
