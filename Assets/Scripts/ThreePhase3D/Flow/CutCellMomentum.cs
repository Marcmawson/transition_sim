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

        internal CutCellMomentumResult ApplyContactTraction(CutCellMomentumResult input,
            CutCellInventories state, List<InterfacePatch> contacts, WettingProfile profile,
            double rhoL, double rhoG, double muL, double muG, double dt)
        {
            if (contacts.Count == 0) return input;
            var geometry = geometryWithMoments ??= plan.Geometry.WithFluidCentroidMoments();
            var impulses = new double[plan.FaceCount];
            var explicitWeights = new double[plan.FaceCount];
            var wall = (double[])input.WallImpulseKgMPerS.Clone();
            foreach (var patch in contacts)
            {
                int group = plan.GroupByCell[patch.Cell];
                plan.Grid.Cells.Coordinates(patch.Cell, out int i, out int j, out int k);
                var centroid = plan.Grid.Lower(i, j, k) + plan.Grid.Spacing * .5 +
                    geometry.FluidCentroidOffset(patch.Cell);
                double distance = D3.Dot(centroid - patch.CentroidLocalM, patch.NormalLocal);
                double alpha = state.LiquidM3[group] / plan.Volumes[group];
                double viscosity = alpha * muL + (1 - alpha) * muG;
                var sampled = new double[3];
                for (int entry = plan.Rows[group]; entry < plan.Rows[group + 1]; entry++)
                {
                    int f = Math.Abs(plan.Incidence[entry]) - 1;
                    var face = plan.Faces[f];
                    sampled[face.Axis] += Weight(face, group) * input.VelocityMPerS[f];
                }
                var traction = profile.WallTractionPa(new D3(sampled[0], sampled[1], sampled[2]),
                    patch.NormalLocal, distance, viscosity);
                var impulse = traction * (dt * patch.FluidExposedAreaM2);
                double coefficient = dt * patch.FluidExposedAreaM2 * viscosity *
                    profile.WallGradientPerVelocity(distance);
                for (int entry = plan.Rows[group]; entry < plan.Rows[group + 1]; entry++)
                {
                    int f = Math.Abs(plan.Incidence[entry]) - 1;
                    var face = plan.Faces[f];
                    double weight = Weight(face, group);
                    double contribution = weight * impulse[face.Axis];
                    impulses[f] += contribution;
                    wall[face.Axis] += contribution;
                    explicitWeights[f] += weight * coefficient;
                }
            }

            var velocity = (double[])input.VelocityMPerS.Clone();
            var momentum = (double[])input.MomentumKgMPerS.Clone();
            for (int f = 0; f < impulses.Length; f++)
            {
                if (explicitWeights[f] == 0) continue;
                var face = plan.Faces[f];
                double mass = Part(face, face.Low) + Part(face, face.High);
                // Sampling and scatter are adjoints. The weighted Jensen bound
                // limits the largest mass-normalised drag eigenvalue to one.
                if (explicitWeights[f] > mass)
                    throw new InvalidOperationException("Timestep exceeds the explicit contact traction bound.");
                momentum[f] += impulses[f];
                velocity[f] = momentum[f] / mass;
            }
            return new CutCellMomentumResult(velocity, momentum, input.BoundaryMomentumKgMPerS, wall,
                input.MaxDualMassErrorKg, contacts.Count);

            double Weight(VariableCoefficientProjection.Face face, int group) => face.Area / areaSum[face.Axis, group];

            double Part(VariableCoefficientProjection.Face face, int group) => group < 0 ? 0 :
                (rhoL * state.LiquidM3[group] + rhoG * state.GasM3[group]) * Weight(face, group);
        }

        internal CutCellMomentumResult Advance(CutCellInventories before, CutCellTransportResult after,
            double[] velocity, double liquidDensity, double gasDensity)
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
                amount = Math.Abs(amount);
                for (int axis = 0; axis < 3; axis++)
                {
                    double impulse = donor < 0 ? amount * after.Reservoir[f].VelocityMPerS[axis] :
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
                Part(state, face, face.Low) + Part(state, face, face.High);

            double Part(CutCellInventories state, VariableCoefficientProjection.Face face, int group)
            {
                if (group < 0) return 0;
                double cellMass = liquidDensity * state.LiquidM3[group] + gasDensity * state.GasM3[group];
                return cellMass * (face.Area / areaSum[face.Axis, group]);
            }

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

    public sealed class CutCellMomentumResult
    {
        public readonly double[] VelocityMPerS, MomentumKgMPerS, BoundaryMomentumKgMPerS, WallImpulseKgMPerS;
        public readonly double MaxDualMassErrorKg;
        public readonly int ContactTractionPatches;
        internal CutCellMomentumResult(double[] velocity, double[] momentum, double[] boundary, double[] wall,
            double error, int contactTractionPatches = 0)
        {
            VelocityMPerS = velocity; MomentumKgMPerS = momentum;
            ContactTractionPatches = contactTractionPatches;
            BoundaryMomentumKgMPerS = boundary; WallImpulseKgMPerS = wall; MaxDualMassErrorKg = error;
        }
    }
}
