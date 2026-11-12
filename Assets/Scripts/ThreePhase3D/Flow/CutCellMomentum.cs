using System;
using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.PhaseChange;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // Each pressure group partitions its mass among its active staggered faces
    // of a component, proportional to aperture. A primal mass transfer removes
    // donor contributions using those weights and deposits their mixed momentum
    // at the recipient faces. Missing component DOFs exchange impulse with the wall.
    internal sealed class CutCellMomentum
    {
        private readonly VariableCoefficientProjection plan;
        private readonly double[,] areaSum;
        private CutCellGeometry geometryWithMoments;

        internal CutCellMomentum(VariableCoefficientProjection plan)
        {
            this.plan = plan;
            areaSum = new double[3, plan.GroupCount];
            foreach (var face in plan.Faces)
            {
                if (face.Active == 0) continue;
                if (face.Low >= 0) areaSum[face.Axis, face.Low] += face.Area;
                if (face.High >= 0) areaSum[face.Axis, face.High] += face.Area;
            }
        }

        // Interior MAC stresses on the full Cartesian fluid rows of the
        // planar reference. Cell normals and edge shears share a symmetric
        // strain/force pair, so the bulk impulse is dissipative and conservative.
        internal CutCellMomentumResult ApplyPlanarBulkViscosity(CutCellMomentumResult input,
            CutCellInventories state, double rhoL, double rhoG, double muL, double muG, double dt)
        {
            var grid = plan.Grid;
            var cells = grid.Cells;
            var mass = new double[plan.FaceCount];
            var viscosity = new double[plan.GroupCount];
            var impulse = new double[plan.FaceCount];
            var weights = new double[plan.FaceCount];
            for (int group = 0; group < viscosity.Length; group++)
            {
                double alpha = state.LiquidM3[group] / plan.Volumes[group];
                viscosity[group] = alpha * muL + (1 - alpha) * muG;
            }
            for (int f = 0; f < mass.Length; f++)
                if (plan.Faces[f].Active != 0)
                    mass[f] = FaceMass(state, plan.Faces[f], rhoL, rhoG);

            double scale = dt * grid.CellVolume;
            for (int k = 0; k < cells.Z; k++)
            for (int j = 0; j < cells.Y; j++)
            for (int i = 0; i < cells.X; i++)
            {
                int group = plan.GroupByCell[cells.Index(i, j, k)];
                if (group < 0) continue;
                for (int a = 0; a < 3; a++)
                {
                    int ai = i + (a == 0 ? 1 : 0), aj = j + (a == 1 ? 1 : 0), ak = k + (a == 2 ? 1 : 0);
                    double inverseA = 1 / grid.Spacing[a];
                    Scatter(2 * scale * viscosity[group], plan.FaceIndex(a, i, j, k),
                        plan.FaceIndex(a, ai, aj, ak), inverseA);

                    for (int b = a + 1; b < 3; b++)
                    {
                        int bi = b == 0 ? 1 : 0, bj = b == 1 ? 1 : 0, bk = b == 2 ? 1 : 0;
                        if (!cells.Interior(ai + bi, aj + bj, ak + bk)) continue;
                        int groupA = plan.GroupByCell[cells.Index(ai, aj, ak)];
                        int groupB = plan.GroupByCell[cells.Index(i + bi, j + bj, k + bk)];
                        int groupAB = plan.GroupByCell[cells.Index(ai + bi, aj + bj, ak + bk)];
                        if (groupA < 0 || groupB < 0 || groupAB < 0) continue;

                        double edgeViscosity = 4 / (1 / viscosity[group] + 1 / viscosity[groupA] +
                            1 / viscosity[groupB] + 1 / viscosity[groupAB]);
                        Scatter(scale * edgeViscosity, plan.FaceIndex(a, ai, aj, ak),
                            plan.FaceIndex(a, ai + bi, aj + bj, ak + bk), 1 / grid.Spacing[b],
                            plan.FaceIndex(b, i + bi, j + bj, k + bk),
                            plan.FaceIndex(b, ai + bi, aj + bj, ak + bk), inverseA);
                    }
                }
            }

            void Scatter(double factor, int low, int high, double inverse, int otherLow = -1,
                int otherHigh = -1, double otherInverse = 0)
            {
                if (mass[low] == 0 || mass[high] == 0 ||
                    (otherLow >= 0 && (mass[otherLow] == 0 || mass[otherHigh] == 0))) return;
                double strain = (input.VelocityMPerS[high] - input.VelocityMPerS[low]) * inverse;
                if (otherLow >= 0)
                    strain += (input.VelocityMPerS[otherHigh] - input.VelocityMPerS[otherLow]) * otherInverse;
                double stressImpulse = factor * strain;
                double rowScale = 2 * factor * (inverse + otherInverse);
                AddPair(low, high, inverse);
                if (otherLow >= 0) AddPair(otherLow, otherHigh, otherInverse);

                void AddPair(int minus, int plus, double derivative)
                {
                    double delta = stressImpulse * derivative;
                    impulse[minus] += delta;
                    impulse[plus] -= delta;
                    weights[minus] += rowScale * derivative;
                    weights[plus] += rowScale * derivative;
                }
            }

            var velocity = (double[])input.VelocityMPerS.Clone();
            var momentum = (double[])input.MomentumKgMPerS.Clone();
            double energyLoss = 0;
            for (int f = 0; f < mass.Length; f++)
            {
                if (weights[f] == 0) continue;
                // Absolute row sums bound the mass-normalised stress operator.
                if (!(weights[f] <= mass[f]) || !SiUnits.IsFinite(impulse[f]))
                    throw new InvalidOperationException("Timestep exceeds the explicit bulk viscosity bound.");
                momentum[f] += impulse[f];
                velocity[f] = momentum[f] / mass[f];
                energyLoss -= .5 * impulse[f] * (input.VelocityMPerS[f] + velocity[f]);
            }
            return new CutCellMomentumResult(velocity, momentum, input.BoundaryMomentumKgMPerS,
                input.WallImpulseKgMPerS, input.MaxDualMassErrorKg, input.ContactTractionPatches, energyLoss);
        }

        internal double[][] GatherGroupMomentum(CutCellInventories state, float[] velocity, double rhoL, double rhoG)
        {
            if (velocity == null || velocity.Length != plan.FaceCount)
                throw new ArgumentException("Velocity does not match the old staggered grid.", nameof(velocity));
            var result = new[] { new double[plan.GroupCount], new double[plan.GroupCount], new double[plan.GroupCount] };
            for (int f = 0; f < plan.FaceCount; f++)
            {
                var face = plan.Faces[f];
                double u = velocity[f];
                if (!SiUnits.IsFinite(u) || (face.Active == 0 && u != 0))
                    throw new ArgumentException("Nonfinite velocity or nonzero blocked-face velocity.", nameof(velocity));
                if (face.Active == 0) continue;
                if (face.Low >= 0) result[face.Axis][face.Low] += PartMass(state, face, face.Low, rhoL, rhoG) * u;
                if (face.High >= 0) result[face.Axis][face.High] += PartMass(state, face, face.High, rhoL, rhoG) * u;
            }
            return result;
        }

        internal CutCellMomentumResult ScatterGroupMomentum(CutCellInventories state, double[][] groupMomentum,
            double rhoL, double rhoG)
        {
            for (int axis = 0; axis < 3; axis++)
            for (int group = 0; group < plan.GroupCount; group++)
                if (areaSum[axis, group] == 0 && groupMomentum[axis][group] != 0)
                    throw new InvalidOperationException("Remapped momentum has no active staggered face.");
            var velocity = new double[plan.FaceCount];
            var momentum = new double[plan.FaceCount];
            for (int f = 0; f < plan.FaceCount; f++)
            {
                var face = plan.Faces[f];
                if (face.Active == 0) continue;
                double mass = FaceMass(state, face, rhoL, rhoG);
                momentum[f] = Portion(face.Low) + Portion(face.High);
                velocity[f] = momentum[f] / mass;
                if (!(mass > 0) || !SiUnits.IsFinite(velocity[f]))
                    throw new InvalidOperationException("Invalid remapped staggered mass or velocity.");

                double Portion(int group) => group < 0 ? 0 :
                    groupMomentum[face.Axis][group] * (face.Area / areaSum[face.Axis, group]);
            }
            return new CutCellMomentumResult(velocity, momentum, new double[3], new double[3], 0);
        }

        private double FaceMass(CutCellInventories state, VariableCoefficientProjection.Face face, double rhoL, double rhoG) =>
            PartMass(state, face, face.Low, rhoL, rhoG) + PartMass(state, face, face.High, rhoL, rhoG);

        private double PartMass(CutCellInventories state, VariableCoefficientProjection.Face face,
            int group, double rhoL, double rhoG) => group < 0 ? 0 :
            (rhoL * state.LiquidM3[group] + rhoG * state.GasM3[group]) * (face.Area / areaSum[face.Axis, group]);

        internal CutCellMomentumResult ApplyContactTraction(CutCellMomentumResult input,
            CutCellInventories state, List<InterfacePatch> contacts, WettingProfile profile,
            double rhoL, double rhoG, double muL, double muG, double dt,
            Func<ContactTractionWork, double[]> dispatch = null, bool planarNoSlip = false)
        {
            if (contacts.Count == 0 && !planarNoSlip) return input;
            var geometry = geometryWithMoments ??= plan.Geometry.WithFluidCentroidMoments();
            var tensors = new D3[checked(3 * plan.GroupCount)];
            var samples = new D3[plan.GroupCount];
            var coefficients = new double[plan.GroupCount];
            var explicitWeights = new double[plan.FaceCount];
            var wall = (double[])input.WallImpulseKgMPerS.Clone();
            if (planarNoSlip)
            {
                var slipPatches = new HashSet<InterfacePatch>(contacts);
                foreach (var patch in plan.Geometry.Patches)
                    if (patch.Role == SolidRole.Fixed && patch.FluidExposedAreaM2 > 0 &&
                        plan.GroupByCell[patch.Cell] >= 0)
                        AccumulatePatch(patch, slipPatches.Contains(patch));
            }
            else
            {
                foreach (var patch in contacts) AccumulatePatch(patch, true);
            }

            void AccumulatePatch(InterfacePatch patch, bool slip)
            {
                int group = plan.GroupByCell[patch.Cell];
                plan.Grid.Cells.Coordinates(patch.Cell, out int i, out int j, out int k);
                var centroid = plan.Grid.Lower(i, j, k) + plan.Grid.Spacing * .5 +
                    geometry.FluidCentroidOffset(patch.Cell);
                double distance = D3.Dot(centroid - patch.CentroidLocalM, patch.NormalLocal);
                double alpha = state.LiquidM3[group] / plan.Volumes[group];
                double viscosity = alpha * muL + (1 - alpha) * muG;
                if (!(distance > 0))
                    throw new InvalidOperationException("Contact traction requires a resolved fluid centroid.");
                double coefficient = dt * patch.FluidExposedAreaM2 * viscosity *
                    (slip ? profile.WallGradientPerVelocity(distance) : 1 / distance);
                var normal = patch.NormalLocal;
                coefficients[group] += coefficient;
                tensors[3 * group] += (normal * normal.X - new D3(1, 0, 0)) * coefficient;
                tensors[3 * group + 1] += (normal * normal.Y - new D3(0, 1, 0)) * coefficient;
                tensors[3 * group + 2] += (normal * normal.Z - new D3(0, 0, 1)) * coefficient;
            }
            for (int group = 0; group < samples.Length; group++)
            {
                if (coefficients[group] == 0) continue;
                var sampled = new double[3];
                for (int entry = plan.Rows[group]; entry < plan.Rows[group + 1]; entry++)
                {
                    int f = Math.Abs(plan.Incidence[entry]) - 1;
                    var face = plan.Faces[f];
                    double weight = Weight(face, group);
                    sampled[face.Axis] += weight * input.VelocityMPerS[f];
                    explicitWeights[f] += weight * coefficients[group];
                }
                samples[group] = new D3(sampled[0], sampled[1], sampled[2]);
            }
            var work = new ContactTractionWork(plan, tensors, samples, areaSum);
            var impulses = dispatch == null ? work.ReferenceImpulses() : dispatch(work);
            if (impulses == null || impulses.Length != plan.FaceCount)
                throw new InvalidOperationException("Invalid contact traction result size.");

            var velocity = (double[])input.VelocityMPerS.Clone();
            var momentum = (double[])input.MomentumKgMPerS.Clone();
            for (int f = 0; f < impulses.Length; f++)
            {
                if (!SiUnits.IsFinite(impulses[f]) || (explicitWeights[f] == 0 && impulses[f] != 0))
                    throw new InvalidOperationException("Invalid impulse on a contact traction face.");
                if (explicitWeights[f] == 0) continue;
                var face = plan.Faces[f];
                double mass = FaceMass(state, face, rhoL, rhoG);
                // Sampling and scatter are adjoints. The weighted Jensen bound
                // limits the largest mass-normalised drag eigenvalue to one.
                if (explicitWeights[f] > mass)
                    throw new InvalidOperationException("Timestep exceeds the explicit contact traction bound.");
                wall[face.Axis] += impulses[f];
                momentum[f] += impulses[f];
                velocity[f] = momentum[f] / mass;
            }
            return new CutCellMomentumResult(velocity, momentum, input.BoundaryMomentumKgMPerS, wall,
                input.MaxDualMassErrorKg, contacts.Count);

            double Weight(VariableCoefficientProjection.Face face, int group) => face.Area / areaSum[face.Axis, group];
        }

        internal CutCellMomentumResult Advance(CutCellInventories before, CutCellTransportResult after,
            double[] velocity, double liquidDensity, double gasDensity,
            IReadOnlyList<(int Donor, int Recipient, double MassKg)> phaseTransfers = null)
        {
            if (velocity == null || velocity.Length != plan.FaceCount ||
                !SiUnits.IsFinite(liquidDensity) || !SiUnits.IsFinite(gasDensity) || liquidDensity <= 0 || gasDensity <= 0)
                throw new ArgumentException("Invalid cut-cell momentum inputs.");
            var mass = new double[plan.FaceCount];
            var momentum = new double[plan.FaceCount];
            var wall = new double[3];
            var exterior = new double[3];
            var result = new double[plan.FaceCount];
            for (int f = 0; f < mass.Length; f++)
            {
                var face = plan.Faces[f];
                if (!SiUnits.IsFinite(velocity[f]) || (face.Active == 0 && velocity[f] != 0))
                    throw new ArgumentException("Invalid velocity on a cut-cell momentum face.");
                if (face.Active == 0) continue;
                mass[f] = DualMass(before, face);
                momentum[f] = mass[f] * velocity[f];
            }

            for (int f = 0; f < mass.Length; f++)
            {
                double amount = liquidDensity * after.LiquidFaceM3[f] + gasDensity * after.GasFaceM3[f];
                if (amount == 0) continue;
                var face = plan.Faces[f];
                int donor = amount > 0 ? face.Low : face.High;
                int recipient = amount > 0 ? face.High : face.Low;
                Transfer(donor, recipient, Math.Abs(amount), after.Reservoir[f].VelocityMPerS);
            }
            if (phaseTransfers != null)
                foreach (var phase in phaseTransfers)
                    Transfer(phase.Donor, phase.Recipient, phase.MassKg, default);

            void Transfer(int donor, int recipient, double amount, D3 inflowVelocity)
            {
                for (int axis = 0; axis < 3; axis++)
                {
                    double impulse = donor < 0 ? amount * inflowVelocity[axis] :
                        Distribute(donor, axis, amount, 0, true);
                    if (donor < 0) exterior[axis] += impulse;
                    if (recipient < 0) exterior[axis] -= impulse;
                    else if (areaSum[axis, recipient] == 0) wall[axis] -= impulse;
                    else Distribute(recipient, axis, amount, impulse, false);
                }
            }

            double maxMassError = 0;
            for (int f = 0; f < mass.Length; f++)
            {
                var face = plan.Faces[f];
                if (face.Active == 0) continue;
                double expected = DualMass(after.Fields, face);
                maxMassError = Math.Max(maxMassError, Math.Abs(expected - mass[f]));
                if (!(expected > 0) || Math.Abs(expected - mass[f]) > 1e-9 * expected)
                    throw new InvalidOperationException("Cut-cell staggered mass does not close.");
                result[f] = momentum[f] / expected;
                if (!SiUnits.IsFinite(result[f])) throw new InvalidOperationException("Nonfinite cut-cell momentum.");
            }
            return new CutCellMomentumResult(result, momentum, exterior, wall, maxMassError);

            double DualMass(CutCellInventories state, VariableCoefficientProjection.Face face) =>
                FaceMass(state, face, liquidDensity, gasDensity);

            double Distribute(int group, int axis, double total, double incomingMomentum, bool withdraw)
            {
                double sum = areaSum[axis, group];
                if (sum == 0) return 0;
                double outgoingMomentum = 0;
                for (int entry = plan.Rows[group]; entry < plan.Rows[group + 1]; entry++)
                {
                    int index = Math.Abs(plan.Incidence[entry]) - 1;
                    var face = plan.Faces[index];
                    if (face.Axis != axis) continue;
                    double weight = face.Area / sum;
                    double portion = total * weight;
                    if (withdraw)
                    {
                        double impulse = portion * velocity[index];
                        mass[index] -= portion;
                        momentum[index] -= impulse;
                        outgoingMomentum += impulse;
                    }
                    else
                    {
                        mass[index] += portion;
                        momentum[index] += incomingMomentum * weight;
                    }
                }
                return outgoingMomentum;
            }
        }
    }

    internal sealed class ContactTractionWork
    {
        internal readonly VariableCoefficientProjection Plan;
        internal readonly D3[] Tensors, Samples;
        private readonly double[,] areaSum;

        internal ContactTractionWork(VariableCoefficientProjection plan, D3[] tensors, D3[] samples, double[,] areaSum)
        {
            Plan = plan;
            Tensors = tensors;
            Samples = samples;
            this.areaSum = areaSum;
        }

        internal D3 WeightedRow(int faceIndex, int group)
        {
            var face = Plan.Faces[faceIndex];
            return face.Active == 0 || group < 0 ? default :
                Tensors[3 * group + face.Axis] * (face.Area / areaSum[face.Axis, group]);
        }

        internal double[] ReferenceImpulses()
        {
            var impulses = new double[Plan.FaceCount];
            for (int f = 0; f < impulses.Length; f++)
            {
                var face = Plan.Faces[f];
                if (face.Active == 0) continue;
                impulses[f] = Contribution(face.Low) + Contribution(face.High);
                double Contribution(int group) => group < 0 ? 0 : D3.Dot(WeightedRow(f, group), Samples[group]);
            }
            return impulses;
        }
    }

    public sealed class CutCellMomentumResult
    {
        public readonly double[] VelocityMPerS, MomentumKgMPerS, BoundaryMomentumKgMPerS, WallImpulseKgMPerS;
        public readonly double MaxDualMassErrorKg;
        public readonly int ContactTractionPatches;
        public readonly double BulkViscousEnergyLossJ;
        internal CutCellMomentumResult(double[] velocity, double[] momentum, double[] boundary, double[] wall,
            double error, int contactTractionPatches = 0, double bulkViscousEnergyLossJ = 0)
        {
            VelocityMPerS = velocity; MomentumKgMPerS = momentum;
            ContactTractionPatches = contactTractionPatches;
            BulkViscousEnergyLossJ = bulkViscousEnergyLossJ;
            BoundaryMomentumKgMPerS = boundary; WallImpulseKgMPerS = wall; MaxDualMassErrorKg = error;
        }
    }
}
