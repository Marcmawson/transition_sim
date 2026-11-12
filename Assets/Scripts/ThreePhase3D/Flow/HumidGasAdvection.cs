using System;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // First-order donor transport for gas-only groups. The outlet ledger is signed.
    internal static class HumidGasAdvection
    {
        internal static (D3 WallReaction, D3 OpenBoundaryReaction) ApplyPressure(
            VariableCoefficientProjection plan, FlowStep flow,
            (double DryAirKg, double VapourKg, double EnthalpyJ, D3 MomentumKgMPerS)[] gas)
        {
            if (plan.HasEmbeddedSolid || plan.IsPeriodic ||
                !flow.Diagnostics.Converged || flow.GroupPressurePa.Length != plan.GroupCount || gas.Length != plan.GroupCount)
                throw new ArgumentException("Gas pressure impulses require a compatible Cartesian box projection.");

            var impulse = new D3[gas.Length];
            D3 wallReaction = default, openReaction = default;
            var cells = plan.Grid.Cells;
            for (int k = 0; k < cells.Z; k++)
            for (int j = 0; j < cells.Y; j++)
            for (int i = 0; i < cells.X; i++)
            {
                int group = plan.PressureGroup(i, j, k);
                for (int axis = 0; axis < 3; axis++)
                for (int side = 0; side < 2; side++)
                {
                    int index = plan.FaceIndex(axis, i + (axis == 0 ? side : 0),
                        j + (axis == 1 ? side : 0), k + (axis == 2 ? side : 0));
                    var face = plan.Faces[index];
                    double pressure = flow.GroupPressurePa[group];
                    bool open = face.Active != 0 && (face.Low < 0 || face.High < 0);
                    if (open) pressure = face.BoundaryPressure;
                    else if (face.Active != 0)
                        pressure = 0.5 * (flow.GroupPressurePa[face.Low] + flow.GroupPressurePa[face.High]);
                    // A rigid wall uses zero normal pressure gradient.
                    double value = (side == 0 ? 1 : -1) * pressure * plan.Grid.FaceArea(axis) * flow.Diagnostics.Dt;
                    var forceImpulse = new D3(axis == 0 ? value : 0, axis == 1 ? value : 0, axis == 2 ? value : 0);
                    impulse[group] += forceImpulse;
                    if (open) openReaction -= forceImpulse;
                    else if (face.Active == 0) wallReaction -= forceImpulse;
                }
            }
            for (int group = 0; group < gas.Length; group++)
                gas[group].MomentumKgMPerS += impulse[group];
            return (wallReaction, openReaction);
        }

        internal static float[] PredictFaces(VariableCoefficientProjection plan,
            (double DryAirKg, double VapourKg, double EnthalpyJ, D3 MomentumKgMPerS)[] gas)
        {
            if (gas.Length != plan.GroupCount)
                throw new ArgumentException("Gas inventories must match the pressure groups.", nameof(gas));
            var cellVelocity = new D3[gas.Length];
            for (int group = 0; group < gas.Length; group++)
            {
                double mass = gas[group].DryAirKg + gas[group].VapourKg;
                if (!(mass > 0) || !SiUnits.IsFinite(mass))
                    throw new ArgumentException("Face prediction requires positive finite gas mass.", nameof(gas));
                cellVelocity[group] = gas[group].MomentumKgMPerS / mass;
            }

            var predicted = new float[plan.FaceCount];
            for (int index = 0; index < predicted.Length; index++)
            {
                var face = plan.Faces[index];
                if (face.Active == 0) continue;
                double velocity = face.Low < 0 ? cellVelocity[face.High][face.Axis] :
                    face.High < 0 ? cellVelocity[face.Low][face.Axis] :
                    0.5 * (cellVelocity[face.Low][face.Axis] + cellVelocity[face.High][face.Axis]);
                predicted[index] = (float)velocity;
                if (!SiUnits.IsFinite(predicted[index]))
                    throw new ArgumentException("Gas face velocity exceeds the supported range.", nameof(gas));
            }
            return predicted;
        }

        internal static ((double DryAirKg, double VapourKg, double EnthalpyJ, D3 MomentumKgMPerS)[] Gas,
            (double DryAirKg, double VapourKg, double EnthalpyJ, D3 MomentumKgMPerS) Outflow) Advance(
            VariableCoefficientProjection plan, FlowStep flow, double[] gasVolumes,
            (double DryAirKg, double VapourKg, double EnthalpyJ, D3 MomentumKgMPerS)[] gas,
            (double DryAirKg, double VapourKg, double EnthalpyJ, D3 MomentumKgMPerS)? reservoirPerM3 = null)
        {
            if (!flow.Diagnostics.Converged || flow.VelocityMPerS.Length != plan.FaceCount ||
                gas.Length != plan.GroupCount || gasVolumes.Length != plan.GroupCount)
                throw new ArgumentException("Gas advection requires a compatible converged projection and gas inventories.");

            var incoming = new (double DryAirKg, double VapourKg, double EnthalpyJ, D3 MomentumKgMPerS)[gas.Length];
            var fractions = new double[gas.Length];
            (double DryAirKg, double VapourKg, double EnthalpyJ, D3 MomentumKgMPerS) outlet = default;
            for (int index = 0; index < plan.FaceCount; index++)
            {
                var face = plan.Faces[index];
                if (face.Active == 0 || face.Low == face.High) continue;
                double volume = face.Area * (double)flow.VelocityMPerS[index] * flow.Diagnostics.Dt;
                if (volume == 0) continue;
                int donor = volume > 0 ? face.Low : face.High;
                int receiver = volume > 0 ? face.High : face.Low;
                if (donor < 0)
                {
                    if (!reservoirPerM3.HasValue)
                        throw new InvalidOperationException("Gas advection requires a prescribed reservoir for boundary inflow.");
                    var reservoir = reservoirPerM3.Value;
                    if (!(reservoir.DryAirKg > 0 && reservoir.VapourKg >= 0) ||
                        !SiUnits.IsFinite(reservoir.DryAirKg) || !SiUnits.IsFinite(reservoir.VapourKg) ||
                        !SiUnits.IsFinite(reservoir.EnthalpyJ))
                        throw new ArgumentException("Invalid gas reservoir concentrations.");
                    incoming[receiver] = Add(incoming[receiver], Scale(reservoir, Math.Abs(volume)));
                    outlet = Add(outlet, Scale(reservoir, -Math.Abs(volume)));
                    continue;
                }
                double fraction = Math.Abs(volume) / gasVolumes[donor];
                fractions[donor] += fraction;
                if (!(fractions[donor] <= 1))
                    throw new InvalidOperationException("Projected outflow exceeds the donor gas volume. Reduce the timestep.");
                var transfer = Scale(gas[donor], fraction);
                if (receiver < 0) outlet = Add(outlet, transfer);
                else incoming[receiver] = Add(incoming[receiver], transfer);
            }

            for (int group = 0; group < gas.Length; group++)
            {
                var state = gas[group];
                if (!SiUnits.IsFinite(gasVolumes[group]) || gasVolumes[group] <= 0 ||
                    !SiUnits.IsFinite(state.DryAirKg) || state.DryAirKg <= 0 ||
                    !SiUnits.IsFinite(state.VapourKg) || state.VapourKg < 0 || !SiUnits.IsFinite(state.EnthalpyJ))
                    throw new ArgumentException("Invalid gas donor inventory or volume.");
                incoming[group] = Add(incoming[group], Scale(state, 1 - fractions[group]));
            }
            return (incoming, outlet);
        }

        private static (double, double, double, D3) Scale((double Air, double Vapour, double Enthalpy, D3 Momentum) gas, double fraction) =>
            (gas.Air * fraction, gas.Vapour * fraction, gas.Enthalpy * fraction, gas.Momentum * fraction);

        private static (double, double, double, D3) Add((double Air, double Vapour, double Enthalpy, D3 Momentum) a,
            (double Air, double Vapour, double Enthalpy, D3 Momentum) b) =>
            (a.Air + b.Air, a.Vapour + b.Vapour, a.Enthalpy + b.Enthalpy, a.Momentum + b.Momentum);
    }
}
