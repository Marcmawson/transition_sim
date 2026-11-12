using System;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // First-order donor transport for gas-only groups. The outlet ledger is signed.
    internal static class HumidGasAdvection
    {
        internal static ((double DryAirKg, double VapourKg, double EnthalpyJ)[] Gas,
            (double DryAirKg, double VapourKg, double EnthalpyJ) Outflow) Advance(
            VariableCoefficientProjection plan, FlowStep flow, double[] gasVolumes,
            (double DryAirKg, double VapourKg, double EnthalpyJ)[] gas,
            (double DryAirKg, double VapourKg, double EnthalpyJ)? reservoirPerM3 = null)
        {
            if (!flow.Diagnostics.Converged || flow.VelocityMPerS.Length != plan.FaceCount ||
                gas.Length != plan.GroupCount || gasVolumes.Length != plan.GroupCount)
                throw new ArgumentException("Gas advection requires a compatible converged projection and gas inventories.");

            var incoming = new (double DryAirKg, double VapourKg, double EnthalpyJ)[gas.Length];
            var fractions = new double[gas.Length];
            (double DryAirKg, double VapourKg, double EnthalpyJ) outlet = default;
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

        private static (double, double, double) Scale((double Air, double Vapour, double Enthalpy) gas, double fraction) =>
            (gas.Air * fraction, gas.Vapour * fraction, gas.Enthalpy * fraction);

        private static (double, double, double) Add((double Air, double Vapour, double Enthalpy) a,
            (double Air, double Vapour, double Enthalpy) b) =>
            (a.Air + b.Air, a.Vapour + b.Vapour, a.Enthalpy + b.Enthalpy);
    }
}
