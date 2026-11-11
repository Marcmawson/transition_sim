using System;
using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.PhaseChange;

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

        private readonly Func<int, WettingContact> contactAtCell;
        private readonly HashSet<InterfacePatch> wettablePatches;
        private readonly Dictionary<int, List<InterfacePatch>> trackedPatches;
        private readonly WettingProfile trackedWetting;
        private readonly PlanarWallCapillarity capillarity;

        public static CutCellVofTransport WithPlanarWallCapillarity(VariableCoefficientProjection plan,
            int firstFluidY, double surfaceTensionNPerM, double contactAngleDegrees = 90) =>
            new CutCellVofTransport(plan, null, null,
                new PlanarWallCapillarity(plan ?? throw new ArgumentNullException(nameof(plan)),
                    firstFluidY, surfaceTensionNPerM, contactAngleDegrees));

        public static CutCellVofTransport WithPlanarWallWetting(VariableCoefficientProjection plan,
            int firstFluidY, double surfaceTensionNPerM, WettingProfile profile)
        {
            if (plan == null) throw new ArgumentNullException(nameof(plan));
            if (profile == null) throw new ArgumentNullException(nameof(profile));
            return new CutCellVofTransport(plan, null, profile,
                new PlanarWallCapillarity(plan, firstFluidY, surfaceTensionNPerM, profile.EquilibriumAngleDegrees));
        }

        public static CutCellVofTransport WithTrackedWetting(VariableCoefficientProjection plan, WettingProfile profile) =>
            new CutCellVofTransport(plan, null, profile ?? throw new ArgumentNullException(nameof(profile)));

        public CutCellVofTransport(VariableCoefficientProjection plan) : this(plan, null) { }

        // The caller supplies the active contact cells and wall-relative speeds for the accepted step.
        public CutCellVofTransport(VariableCoefficientProjection plan, Func<int, WettingContact> contactAtCell)
            : this(plan, contactAtCell, null) { }

        private CutCellVofTransport(VariableCoefficientProjection plan, Func<int, WettingContact> contactAtCell,
            WettingProfile trackedWetting, PlanarWallCapillarity capillarity = null)
        {
            this.plan = plan ?? throw new ArgumentNullException(nameof(plan));
            this.contactAtCell = contactAtCell;
            if (contactAtCell != null)
            {
                wettablePatches = new HashSet<InterfacePatch>();
                foreach (var patch in plan.Geometry.Patches)
                    if (patch.Role == SolidRole.Fixed && patch.FluidExposedAreaM2 > 0)
                        wettablePatches.Add(patch);
            }
            this.trackedWetting = trackedWetting;
            this.capillarity = capillarity;
            if (trackedWetting != null)
            {
                trackedPatches = new Dictionary<int, List<InterfacePatch>>();
                foreach (var patch in plan.Geometry.Patches)
                {
                    // The exposed polygon is known exactly only for wholly exposed patches.
                    if (patch.Role != SolidRole.Fixed || patch.ContactAreaM2 != 0) continue;
                    if (!trackedPatches.TryGetValue(patch.Cell, out var patches))
                        trackedPatches.Add(patch.Cell, patches = new List<InterfacePatch>());
                    patches.Add(patch);
                }
            }
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

        public CutCellCoupledStep AdvanceCoupledWithWetting(CutCellInventories state, FlowStep transportingFlow,
            double liquidDensity, double gasDensity, double liquidViscosityPaS, double gasViscosityPaS,
            Func<int, CutCellInflow> inflow = null, int maxIterations = 4096)
        {
            return AdvanceCoupledWithWetting(state, transportingFlow, liquidDensity, gasDensity,
                liquidViscosityPaS, gasViscosityPaS, inflow, maxIterations, FlowCpuReference.Project);
        }

        internal CutCellCoupledStep AdvanceCoupledWithWetting(CutCellInventories state, FlowStep transportingFlow,
            double liquidDensity, double gasDensity, double liquidViscosityPaS, double gasViscosityPaS,
            Func<int, CutCellInflow> inflow, int maxIterations, PeriodicVofProjection.PressureSolve project,
            Func<CutCellScalarWork, CutCellScalarOutput> apply = null, CutCellGeometryBatch clip = null,
            Func<CutCellInventories, CutCellTransportResult, double[], double, double, CutCellMomentumResult> remap = null,
            Func<ContactTractionWork, double[]> traction = null)
        {
            if (trackedPatches == null) throw new InvalidOperationException("Tracked wetting is required.");
            if (!SiUnits.IsFinite(liquidViscosityPaS) || !SiUnits.IsFinite(gasViscosityPaS) ||
                liquidViscosityPaS <= 0 || gasViscosityPaS <= 0)
                throw new ArgumentException("Both dynamic viscosities must be finite and positive.");
            return AdvanceCoupled(state, transportingFlow, liquidDensity, gasDensity, inflow, maxIterations,
                project, apply, clip, (before, after, velocity, rhoL, rhoG) =>
                {
                    var advected = (remap ?? AdvanceMomentum)(before, after, velocity, rhoL, rhoG);
                    var contacts = new List<InterfacePatch>();
                    foreach (int cell in trackedPatches.Keys)
                    {
                        int group = plan.GroupByCell[cell];
                        if (group < 0 || after.Fields.LiquidM3[group] <= 0 || after.Fields.GasM3[group] <= 0) continue;
                        TrackContact(cell, group, after.Fields.LiquidM3[group],
                            EstimateNormal(cell, group, after.Fields.LiquidM3), advected.VelocityMPerS, contacts.Add);
                    }
                    return momentum.ApplyContactTraction(advected, after.Fields, contacts, trackedWetting,
                        rhoL, rhoG, liquidViscosityPaS, gasViscosityPaS, transportingFlow.Diagnostics.Dt, traction);
                });
        }

        internal CutCellCoupledStep AdvanceCoupled(CutCellInventories state, FlowStep transportingFlow,
            double liquidDensity, double gasDensity, Func<int, CutCellInflow> inflow, int maxIterations,
            PeriodicVofProjection.PressureSolve project,
            Func<CutCellScalarWork, CutCellScalarOutput> apply = null, CutCellGeometryBatch clip = null,
            Func<CutCellInventories, CutCellTransportResult, double[], double, double, CutCellMomentumResult> remap = null)
        {
            var transported = AdvanceProjected(state, transportingFlow, inflow, apply, clip);
            var previousVelocity = Array.ConvertAll(transportingFlow.VelocityMPerS, value => (double)value);
            var advected = remap == null ? AdvanceMomentum(state, transported, previousVelocity, liquidDensity, gasDensity) :
                remap(state, transported, previousVelocity, liquidDensity, gasDensity);
            var nextPlan = new VariableCoefficientProjection(plan.Geometry, plan.Boundaries, cell =>
            {
                int group = plan.GroupByCell[cell];
                return (liquidDensity * transported.Fields.LiquidM3[group] +
                    gasDensity * transported.Fields.GasM3[group]) / plan.Volumes[group];
            }, _ => 0);
            var tentative = Array.ConvertAll(advected.VelocityMPerS, value => (float)value);
            Dictionary<int, double> angles = null;
            if (capillarity != null && trackedPatches != null)
            {
                angles = new Dictionary<int, double>();
                var liquid = transported.Fields.LiquidM3;
                foreach (int cell in trackedPatches.Keys)
                {
                    plan.Grid.Cells.Coordinates(cell, out _, out int j, out _);
                    int group = plan.GroupByCell[cell];
                    if (j != capillarity.FirstFluidY || group < 0 || liquid[group] <= 0 ||
                        transported.Fields.GasM3[group] <= 0) continue;
                    var normal = EstimateNormal(cell, group, liquid);
                    var contact = TrackContact(cell, group, liquid[group], normal, advected.VelocityMPerS);
                    if (contact == null) continue;
                    angles.Add(cell, contact.ContactAngleDegrees(normal));
                }
            }
            capillarity?.Apply(nextPlan, transported.Fields, lowCell, highCell, liquidDensity, gasDensity,
                transportingFlow.Diagnostics.Dt, tentative, angles);
            var pressure = project(nextPlan, tentative, transportingFlow.Diagnostics.Dt, maxIterations);
            return new CutCellCoupledStep(transported, advected, pressure, angles?.Count ?? 0);
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
            Func<int, CutCellInflow> inflow = null) => AdvanceProjected(state, pressure, inflow, null, null);

        private CutCellTransportResult AdvanceProjected(CutCellInventories state, FlowStep pressure,
            Func<int, CutCellInflow> inflow, Func<CutCellScalarWork, CutCellScalarOutput> apply, CutCellGeometryBatch clip)
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
            return Advance(state, velocity, dt, inflow, absolute, apply, clip);
        }

        public CutCellTransportResult Advance(CutCellInventories state, double[] faceVelocity, double dt,
            Func<int, CutCellInflow> inflow = null) => Advance(state, faceVelocity, dt, inflow, 0, null);

        internal CutCellTransportResult Advance(CutCellInventories state, double[] faceVelocity, double dt,
            Func<int, CutCellInflow> inflow, double pressureCorrection, Func<CutCellScalarWork, CutCellScalarOutput> apply, CutCellGeometryBatch clip = null)
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
            var normals = trackedPatches == null ? null : new D3?[planes.Length];
            var reservoir = new CutCellInflow[faces];
            var jobs = clip == null ? null : new List<CutCellClipJob>();
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
                if (jobs != null)
                {
                    jobs.Add(new CutCellClipJob(f, cell, metric.Axis, q > 0, DonorNormal(cell, donor, fields[0], faceVelocity, normals),
                        alpha, volume, widthVolume));
                    continue;
                }
                if (!planes[cell].HasValue)
                    planes[cell] = geometry.Reconstruct(cell, DonorNormal(cell, donor, fields[0], faceVelocity, normals), alpha * volume);
                Record(f, geometry.Sweep(cell, metric.Axis, q > 0, widthVolume, planes[cell].Value),
                    widthVolume, alpha, Tolerance * volume);
            }
            if (jobs != null && jobs.Count > 0)
            {
                var values = clip(geometry, jobs);
                if (values == null || values.Length != jobs.Count)
                    throw new InvalidOperationException("GPU clipping returned an incompatible face list.");
                for (int n = 0; n < jobs.Count; n++)
                {
                    var job = jobs[n];
                    Record(job.Face, values[n], job.SweptVolume, job.Alpha,
                        2e-6 * plan.Grid.CellVolume);
                }
            }

            void Record(int f, double raw, double width, double alpha, double tolerance)
            {
                if (!SiUnits.IsFinite(raw) || raw < -tolerance || raw > width + tolerance)
                    throw new InvalidOperationException("Cut PLIC liquid exceeds the accessible swept slab.");
                double geometric = Math.Max(0, Math.Min(width, raw));
                roundoff += Math.Abs(geometric - raw);
                correction[f] = geometric - alpha * width;
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

        private D3 DonorNormal(int cell, int group, double[] liquid, double[] velocity, D3?[] cache)
        {
            if (cache != null && cache[cell].HasValue) return cache[cell].Value;
            var normal = EstimateNormal(cell, group, liquid);
            var contact = contactAtCell?.Invoke(cell);
            if (contact != null && (contact.Wall.Cell != cell || !wettablePatches.Contains(contact.Wall)))
                throw new ArgumentException("Contact requires an exposed fixed-wall patch of the donor cell.");
            if (cache != null)
                contact = TrackContact(cell, group, liquid[group], normal, velocity);
            if (contact != null) normal = contact.InterfaceNormal(normal);
            if (cache != null) cache[cell] = normal;
            return normal;
        }

        internal WettingContact TrackContact(int cell, int group, double liquidVolume, D3 normal, double[] velocity,
            Action<InterfacePatch> onIntersection = null)
        {
            if (normal.Length == 0 || !trackedPatches.TryGetValue(cell, out var patches)) return null;
            var plane = geometry.Reconstruct(cell, normal, liquidVolume / plan.Volumes[group] * plan.Geometry.FluidVolumeM3[cell]);
            var spacing = plan.Grid.Spacing;
            double tolerance = 1e-11 * Math.Min(spacing.X, Math.Min(spacing.Y, spacing.Z));
            InterfacePatch selected = null;
            foreach (var patch in patches)
            {
                bool negative = false, positive = false;
                foreach (var point in patch.Polygon)
                {
                    double distance = plane.Distance(point);
                    negative |= distance < -tolerance;
                    positive |= distance > tolerance;
                }
                if (!negative || !positive) continue;
                onIntersection?.Invoke(patch);
                if (selected == null || patch.AreaM2 > selected.AreaM2) selected = patch;
            }
            if (selected == null) return null;
            D3 tangent = plane.N - selected.NormalLocal * D3.Dot(plane.N, selected.NormalLocal);
            if (tangent.Length < 1e-12) return null;

            plan.Grid.Cells.Coordinates(cell, out int i, out int j, out int k);
            var sampled = new D3(Component(0), Component(1), Component(2));
            return new WettingContact(trackedWetting, selected, D3.Dot(sampled, tangent.Unit()));

            double Component(int axis)
            {
                int lower = plan.FaceIndex(axis, i, j, k);
                int upper = plan.FaceIndex(axis, i + (axis == 0 ? 1 : 0),
                    j + (axis == 1 ? 1 : 0), k + (axis == 2 ? 1 : 0));
                double a = plan.Faces[lower].Area, b = plan.Faces[upper].Area;
                return a + b == 0 ? 0 : (a * velocity[lower] + b * velocity[upper]) / (a + b);
            }
        }

        internal D3 EstimateNormal(int cell, int group, double[] liquid)
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

    internal delegate double[] CutCellGeometryBatch(CutCellVofGeometry geometry, IReadOnlyList<CutCellClipJob> jobs);

    internal readonly struct CutCellClipJob
    {
        internal readonly int Face, Cell, Axis;
        internal readonly bool Positive;
        internal readonly D3 Normal;
        internal readonly double Alpha, FluidVolume, SweptVolume;
        internal CutCellClipJob(int face, int cell, int axis, bool positive, D3 normal,
            double alpha, double fluidVolume, double sweptVolume)
        {
            Face = face; Cell = cell; Axis = axis; Positive = positive; Normal = normal;
            Alpha = alpha; FluidVolume = fluidVolume; SweptVolume = sweptVolume;
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
        public readonly int CapillaryWettingContacts;
        internal CutCellCoupledStep(CutCellTransportResult transport, CutCellMomentumResult momentum, FlowStep pressure,
            int capillaryWettingContacts = 0)
        {
            Transport = transport; Momentum = momentum; Pressure = pressure;
            CapillaryWettingContacts = capillaryWettingContacts;
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
