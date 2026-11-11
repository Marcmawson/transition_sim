using System;
using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Geometry;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // Scalar reference path for the S04 pressure groups. Inventories are
    // extensive; open-boundary inputs use the same signed face ledger.
    public sealed class CutCellVofTransport
    {
        private const double Tolerance = 1e-10;
        private readonly VariableCoefficientProjection plan;
        private readonly CutCellVofGeometry geometry;
        private readonly CutCellMomentum momentum;
        private readonly int[] lowCell, highCell;
        private readonly int[] parent, parentFace;
        private readonly List<int> order = new List<int>();

        public CutCellVofTransport(VariableCoefficientProjection plan)
        {
            this.plan = plan ?? throw new ArgumentNullException(nameof(plan));
            geometry = new CutCellVofGeometry(plan.Geometry);
            momentum = new CutCellMomentum(plan);
            lowCell = new int[plan.FaceCount];
            highCell = new int[plan.FaceCount];
            for (int face = 0; face < plan.FaceCount; face++)
            {
                var metric = plan.Faces[face];
                if (metric.Active == 0) continue;
                int a = metric.Axis;
                plan.Grid.Shape((GridLocation)(a + 1)).Coordinates(face - plan.FaceOffsets[a], out int i, out int j, out int k);
                lowCell[face] = metric.Low < 0 ? -1 : plan.Grid.Cells.Index(i - (a == 0 ? 1 : 0),
                    j - (a == 1 ? 1 : 0), k - (a == 2 ? 1 : 0));
                highCell[face] = metric.High < 0 ? -1 : plan.Grid.Cells.Index(i, j, k);
            }
            parent = new int[plan.GroupCount];
            parentFace = new int[plan.GroupCount];
            for (int c = 0; c < plan.GroupCount; c++) { parent[c] = -2; parentFace[c] = -1; }
            // Seed all open groups before building the forest. Each such group
            // can close its own residual through a declared open face.
            for (int f = 0; f < plan.FaceCount; f++)
            {
                var metric = plan.Faces[f];
                if (metric.Active == 0 || (metric.Low >= 0 && metric.High >= 0)) continue;
                int group = Math.Max(metric.Low, metric.High);
                if (parent[group] != -2) continue;
                parent[group] = -1;
                parentFace[group] = f;
                order.Add(group);
            }
            int cursor = 0;
            Visit();
            for (int c = 0; c < plan.GroupCount; c++)
            {
                if (parent[c] != -2) continue;
                parent[c] = -1;
                order.Add(c);
                Visit();
            }

            void Visit()
            {
                for (; cursor < order.Count; cursor++)
                {
                    int group = order[cursor];
                    for (int entry = plan.Rows[group]; entry < plan.Rows[group + 1]; entry++)
                    {
                        int signed = plan.Incidence[entry], f = Math.Abs(signed) - 1;
                        var metric = plan.Faces[f];
                        int neighbour = signed > 0 ? metric.High : metric.Low;
                        if (neighbour < 0 || parent[neighbour] != -2) continue;
                        parent[neighbour] = group;
                        parentFace[neighbour] = f;
                        order.Add(neighbour);
                    }
                }
            }
        }

        public CutCellCoupledStep AdvanceCoupled(CutCellInventories state, FlowStep transportingFlow,
            double liquidDensity, double gasDensity, Func<int, CutCellInflow> inflow = null, int maxIterations = 4096) =>
            AdvanceCoupled(state, transportingFlow, liquidDensity, gasDensity, inflow, maxIterations, FlowCpuReference.Project);

        internal CutCellCoupledStep AdvanceCoupled(CutCellInventories state, FlowStep transportingFlow,
            double liquidDensity, double gasDensity, Func<int, CutCellInflow> inflow, int maxIterations,
            PeriodicVofProjection.PressureSolve project)
        {
            var transported = AdvanceProjected(state, transportingFlow, inflow);
            var previousVelocity = Array.ConvertAll(transportingFlow.VelocityMPerS, value => (double)value);
            var advected = AdvanceMomentum(state, transported, previousVelocity, liquidDensity, gasDensity);
            var nextPlan = new VariableCoefficientProjection(plan.Geometry, plan.Boundaries, cell =>
            {
                int group = plan.GroupByCell[cell];
                return (liquidDensity * transported.Fields.LiquidM3[group] +
                    gasDensity * transported.Fields.GasM3[group]) / plan.Volumes[group];
            }, _ => 0);
            var tentative = Array.ConvertAll(advected.VelocityMPerS, value => (float)value);
            var pressure = project(nextPlan, tentative, transportingFlow.Diagnostics.Dt, maxIterations);
            return new CutCellCoupledStep(transported, advected, pressure);
        }

        public CutCellMomentumResult AdvanceMomentum(CutCellInventories before, CutCellTransportResult transported,
            double[] velocity, double liquidDensity, double gasDensity)
        {
            if (before == null || transported == null || transported.Plan != plan)
                throw new ArgumentException("Momentum requires the scalar transfer from the same cut-cell topology.");
            Validate(before.Planes);
            return momentum.Advance(before, transported, velocity, liquidDensity, gasDensity);
        }

        public CutCellTransportResult AdvanceProjected(CutCellInventories state, FlowStep pressure,
            Func<int, CutCellInflow> inflow = null)
        {
            if (pressure == null || !pressure.Diagnostics.Converged || pressure.VelocityMPerS.Length != plan.FaceCount)
                throw new ArgumentException("A converged compatible pressure result is required.");
            double dt = pressure.Diagnostics.Dt, maximum = 0, absolute = 0;
            var velocity = new double[plan.FaceCount];
            var divergence = new double[plan.GroupCount];
            for (int f = 0; f < velocity.Length; f++)
            {
                velocity[f] = pressure.VelocityMPerS[f];
                var metric = plan.Faces[f];
                if (metric.Active == 0) continue;
                double q = velocity[f] * metric.Area * dt;
                if (metric.Low >= 0) divergence[metric.Low] += q;
                if (metric.High >= 0) divergence[metric.High] -= q;
            }
            for (int index = order.Count - 1; index >= 0; index--)
            {
                int group = order[index], f = parentFace[group];
                if (f < 0) continue;
                var metric = plan.Faces[f];
                double delta = metric.Low == group ? -divergence[group] : divergence[group];
                velocity[f] += delta / (metric.Area * dt);
                double volume = parent[group] < 0 ? plan.Volumes[group] :
                    Math.Min(plan.Volumes[group], plan.Volumes[parent[group]]);
                maximum = Math.Max(maximum, Math.Abs(delta) / volume);
                absolute += Math.Abs(delta);
                if (parent[group] >= 0) divergence[parent[group]] += divergence[group];
            }
            if (maximum > 1e-6)
                throw new InvalidOperationException("Cut-cell pressure flux correction exceeds 1e-6 Courant units.");
            return Advance(state, velocity, dt, inflow, absolute, null);
        }

        public CutCellTransportResult Advance(CutCellInventories state, double[] faceVelocity, double dt,
            Func<int, CutCellInflow> inflow = null) => Advance(state, faceVelocity, dt, inflow, 0, null);

        internal CutCellTransportResult Advance(CutCellInventories state, double[] faceVelocity, double dt,
            Func<int, CutCellInflow> inflow, double pressureCorrection, Func<CutCellScalarWork, CutCellScalarOutput> apply)
        {
            if (state == null || faceVelocity == null || faceVelocity.Length != plan.FaceCount ||
                !SiUnits.IsFinite(dt) || dt <= 0) throw new ArgumentException("Invalid cut-cell transport inputs.");
            var fields = state.Planes;
            Validate(fields);
            int groups = plan.GroupCount, faces = plan.FaceCount;
            var swept = new double[faces];
            var correction = new double[faces];
            var divergence = new double[groups];
            var outgoing = new double[groups];
            var planes = new ClipPlane?[plan.Grid.Cells.Count];
            var reservoir = new CutCellInflow[faces];
            double roundoff = 0;

            for (int f = 0; f < faces; f++)
            {
                var metric = plan.Faces[f];
                double u = faceVelocity[f];
                if (!SiUnits.IsFinite(u) || (metric.Active == 0 && u != 0))
                    throw new ArgumentException("A blocked face has nonzero or nonfinite velocity.");
                if (u == 0) continue;
                double q = u * metric.Area * dt;
                if (!SiUnits.IsFinite(q)) throw new ArgumentException("Nonfinite swept volume.");
                if (q == 0) continue;
                swept[f] = q;
                if (metric.Low >= 0) divergence[metric.Low] += q;
                if (metric.High >= 0) divergence[metric.High] -= q;
                int donor = q > 0 ? metric.Low : metric.High;
                if (donor >= 0) outgoing[donor] += Math.Abs(q);
                else
                {
                    if (inflow == null) throw new ArgumentException("An open inflow requires a reservoir state.");
                    reservoir[f] = inflow(f);
                    reservoir[f].Validate();
                }
            }
            for (int c = 0; c < groups; c++)
            {
                double volume = plan.Volumes[c];
                if (Math.Abs(divergence[c]) > Tolerance * volume || outgoing[c] > .5 * volume)
                    throw new ArgumentException("Cut-cell transport requires closed group volume balance and outgoing CFL at most 0.5.");
            }
            for (int f = 0; f < faces; f++)
            {
                double q = swept[f];
                if (q == 0) continue;
                var metric = plan.Faces[f];
                int donor = q > 0 ? metric.Low : metric.High;
                if (donor < 0) continue;
                int cell = q > 0 ? lowCell[f] : highCell[f];
                double widthVolume = Math.Abs(q), volume = plan.Geometry.FluidVolumeM3[cell];
                if (widthVolume > .5 * volume)
                    throw new ArgumentException("Cut-face transfer exceeds half its donor member's accessible volume.");
                double alpha = fields[0][donor] / plan.Volumes[donor];
                if (alpha == 0 || alpha == 1) continue;
                if (!planes[cell].HasValue)
                    planes[cell] = geometry.Reconstruct(cell, Normal(cell, donor, fields[0]), alpha * volume);
                double raw = geometry.Sweep(cell, metric.Axis, q > 0, widthVolume, planes[cell].Value);
                if (raw < -Tolerance * volume || raw > widthVolume + Tolerance * volume)
                    throw new InvalidOperationException("Cut PLIC liquid exceeds the accessible swept slab.");
                double geometric = Math.Max(0, Math.Min(widthVolume, raw));
                roundoff += Math.Abs(geometric - raw);
                double delta = geometric - alpha * widthVolume;
                correction[f] = delta;
            }

            var work = new CutCellScalarWork(plan, fields, swept, correction, outgoing, reservoir);
            var output = apply == null ? ApplyReference(work) : apply(work);
            var next = output.Fields;
            roundoff += output.RoundoffM3;
            for (int c = 0; c < groups; c++)
            {
                double volume = plan.Volumes[c];
                if (next[0][c] < -Tolerance * volume || next[1][c] < -Tolerance * volume ||
                    Math.Abs(next[0][c] + next[1][c] - volume) > 4 * Tolerance * volume)
                    throw new InvalidOperationException("Cut-cell phase volume closure failed.");
                double liquid = next[1][c] <= 0 ? volume : Math.Max(0, Math.Min(volume, next[0][c]));
                double gas = volume - liquid;
                roundoff += Math.Abs(liquid - next[0][c]) + Math.Abs(gas - next[1][c]);
                next[0][c] = liquid;
                next[1][c] = gas;
                // Keep scalar conservation explicit. Reject an inconsistent
                // empty phase rather than silently removing its inventory.
            }
            Validate(next);
            return new CutCellTransportResult(plan, new CutCellInventories(next), output.Liquid, output.Gas,
                output.Boundary, reservoir, roundoff, output.LimitedM3, pressureCorrection);
        }

        private static CutCellScalarOutput ApplyReference(CutCellScalarWork work)
        {
            var plan = work.Plan;
            var fields = work.Fields;
            var swept = work.Swept;
            var correction = work.Correction;
            var outgoing = work.Outgoing;
            var increase = new double[plan.GroupCount];
            var decrease = new double[plan.GroupCount];
            for (int f = 0; f < plan.FaceCount; f++)
            {
                if (swept[f] == 0) continue;
                var face = plan.Faces[f];
                int donor = swept[f] > 0 ? face.Low : face.High;
                if (donor < 0) continue;
                increase[donor] += Math.Max(0, correction[f]);
                decrease[donor] += Math.Max(0, -correction[f]);
            }
            var reservoir = work.Reservoir;
            int faces = plan.FaceCount;
            var next = new double[5][];
            for (int field = 0; field < 5; field++) next[field] = (double[])fields[field].Clone();
            var boundary = new double[5];
            var liquidFace = new double[faces];
            var gasFace = new double[faces];
            double limited = 0, roundoff = 0;
            for (int f = 0; f < faces; f++)
            {
                double q = swept[f];
                if (q == 0) continue;
                var metric = plan.Faces[f];
                int donor = q > 0 ? metric.Low : metric.High;
                double l, g, el, eg, vapour;
                if (donor >= 0)
                {
                    double delta = correction[f], volume = plan.Volumes[donor];
                    double fraction = fields[0][donor] / volume;
                    double capacity = (delta < 0 ? 1 - fraction : fraction) * (volume - outgoing[donor]);
                    double demand = delta < 0 ? decrease[donor] : increase[donor];
                    double accepted = demand > capacity ? delta * (capacity / demand) : delta;
                    limited += Math.Abs(delta - accepted);
                    double proposed = fraction * Math.Abs(q) + accepted;
                    if (proposed < -Tolerance * volume || proposed > Math.Abs(q) + Tolerance * volume)
                        throw new InvalidOperationException("Cut-cell limiter exceeded the swept volume.");
                    double bounded = Math.Max(0, Math.Min(Math.Abs(q), proposed));
                    roundoff += Math.Abs(bounded - proposed);
                    l = Math.Sign(q) * bounded;
                    g = q - l;
                    el = Scale(l, fields[0][donor], fields[2][donor]);
                    eg = Scale(g, fields[1][donor], fields[3][donor]);
                    vapour = Scale(g, fields[1][donor], fields[4][donor]);
                }
                else
                {
                    var source = reservoir[f];
                    l = q * source.LiquidFraction;
                    g = q - l;
                    el = l * source.LiquidEnergyDensity;
                    eg = g * source.GasEnergyDensity;
                    vapour = g * source.VapourDensity;
                }
                liquidFace[f] = l;
                gasFace[f] = g;
                Transfer(0, l); Transfer(1, g); Transfer(2, el); Transfer(3, eg); Transfer(4, vapour);

                void Transfer(int field, double amount)
                {
                    if (metric.Low >= 0) next[field][metric.Low] -= amount;
                    else boundary[field] += amount;
                    if (metric.High >= 0) next[field][metric.High] += amount;
                    else boundary[field] -= amount;
                }
            }

            return new CutCellScalarOutput(next, liquidFace, gasFace, boundary, roundoff, limited);
        }

        private D3 Normal(int cell, int group, double[] liquid)
        {
            plan.Grid.Cells.Coordinates(cell, out int i, out int j, out int k);
            double alpha = liquid[group] / plan.Volumes[group];
            return new D3(Difference(0), Difference(1), Difference(2));

            double Difference(int axis) => (SampleAt(axis, -1) - SampleAt(axis, 1)) / plan.Grid.Spacing[axis];

            // The pressure topology masks the solid and out-of-domain faces.
            double SampleAt(int axis, int sign)
            {
                int face = plan.FaceIndex(axis, i + (axis == 0 && sign > 0 ? 1 : 0),
                    j + (axis == 1 && sign > 0 ? 1 : 0), k + (axis == 2 && sign > 0 ? 1 : 0));
                var metric = plan.Faces[face];
                int other = sign < 0 ? metric.Low : metric.High;
                return metric.Active == 0 || other < 0 ? alpha : liquid[other] / plan.Volumes[other];
            }

        }

        private static double Scale(double transfer, double volume, double inventory)
        {
            if (transfer == 0) return 0;
            if (volume <= 0) throw new InvalidOperationException("A transfer originates in an empty cut-cell phase.");
            return inventory * (transfer / volume);
        }

        private void Validate(double[][] fields)
        {
            foreach (double[] field in fields)
                if (field == null || field.Length != plan.GroupCount || !Array.TrueForAll(field, SiUnits.IsFinite))
                    throw new ArgumentException("Invalid cut-cell inventory shape or value.");
            for (int c = 0; c < plan.GroupCount; c++)
            {
                if (fields[0][c] < 0 || fields[1][c] < 0 || fields[4][c] < 0 ||
                    fields[0][c] > plan.Volumes[c] || fields[1][c] > plan.Volumes[c] ||
                    Math.Abs(fields[0][c] + fields[1][c] - plan.Volumes[c]) > Tolerance * plan.Volumes[c] ||
                    (fields[0][c] == 0 && fields[2][c] != 0) ||
                    (fields[1][c] == 0 && (fields[3][c] != 0 || fields[4][c] != 0)))
                    throw new ArgumentException("Invalid cut-cell phase inventory.");
            }
        }
    }

    internal sealed class CutCellScalarWork
    {
        internal readonly VariableCoefficientProjection Plan;
        internal readonly double[][] Fields;
        internal readonly double[] Swept, Correction, Outgoing;
        internal readonly CutCellInflow[] Reservoir;
        internal CutCellScalarWork(VariableCoefficientProjection plan, double[][] fields, double[] swept,
            double[] correction, double[] outgoing, CutCellInflow[] reservoir)
        {
            Plan = plan; Fields = fields; Swept = swept; Correction = correction;
            Outgoing = outgoing; Reservoir = reservoir;
        }
    }

    internal sealed class CutCellScalarOutput
    {
        internal readonly double[][] Fields;
        internal readonly double[] Liquid, Gas, Boundary;
        internal readonly double RoundoffM3, LimitedM3;
        internal CutCellScalarOutput(double[][] fields, double[] liquid, double[] gas,
            double[] boundary, double roundoff, double limited)
        {
            Fields = fields; Liquid = liquid; Gas = gas; Boundary = boundary;
            RoundoffM3 = roundoff; LimitedM3 = limited;
        }
    }

    public sealed class CutCellCoupledStep
    {
        public readonly CutCellTransportResult Transport;
        public readonly CutCellMomentumResult Momentum;
        public readonly FlowStep Pressure;
        internal CutCellCoupledStep(CutCellTransportResult transport, CutCellMomentumResult momentum, FlowStep pressure)
        {
            Transport = transport; Momentum = momentum; Pressure = pressure;
        }
    }

    public sealed class CutCellInventories
    {
        internal readonly double[][] Planes;
        public double[] LiquidM3 => Planes[0];
        public double[] GasM3 => Planes[1];
        public double[] LiquidEnergyJ => Planes[2];
        public double[] GasEnergyJ => Planes[3];
        public double[] VapourKg => Planes[4];
        public CutCellInventories(double[] liquid, double[] gas, double[] liquidEnergy, double[] gasEnergy, double[] vapour)
            : this(new[] { liquid, gas, liquidEnergy, gasEnergy, vapour }) { }
        internal CutCellInventories(double[][] planes) { Planes = planes; }
    }

    public readonly struct CutCellInflow
    {
        public readonly double LiquidFraction, LiquidEnergyDensity, GasEnergyDensity, VapourDensity;
        public readonly D3 VelocityMPerS;
        public CutCellInflow(double fraction, double liquidEnergyDensity, double gasEnergyDensity, double vapourDensity, D3 velocityMPerS = default)
        {
            LiquidFraction = fraction; LiquidEnergyDensity = liquidEnergyDensity;
            GasEnergyDensity = gasEnergyDensity; VapourDensity = vapourDensity; VelocityMPerS = velocityMPerS;
        }
        internal void Validate()
        {
            if (!SiUnits.IsFinite(LiquidFraction) || LiquidFraction < 0 || LiquidFraction > 1 ||
                !SiUnits.IsFinite(LiquidEnergyDensity) || !SiUnits.IsFinite(GasEnergyDensity) ||
                !SiUnits.IsFinite(VapourDensity) || VapourDensity < 0)
                throw new ArgumentException("Invalid open-boundary reservoir.");
        }
    }

    public sealed class CutCellTransportResult
    {
        internal readonly VariableCoefficientProjection Plan;
        internal readonly CutCellInflow[] Reservoir;
        public readonly CutCellInventories Fields;
        public readonly double[] LiquidFaceM3, GasFaceM3, BoundaryInflow;
        public readonly double RoundoffM3, LimitedM3, PressureFluxCorrectionM3;
        internal CutCellTransportResult(VariableCoefficientProjection plan, CutCellInventories fields, double[] liquid, double[] gas,
            double[] boundary, CutCellInflow[] reservoir, double roundoff, double limited, double pressureCorrection)
        {
            Plan = plan; Reservoir = reservoir;
            Fields = fields; LiquidFaceM3 = liquid; GasFaceM3 = gas; BoundaryInflow = boundary;
            RoundoffM3 = roundoff; LimitedM3 = limited; PressureFluxCorrectionM3 = pressureCorrection;
        }
    }
}
