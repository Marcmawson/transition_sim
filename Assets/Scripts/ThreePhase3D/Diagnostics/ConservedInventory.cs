using System;
using System.Collections.Generic;
using System.Runtime.Serialization;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Diagnostics
{
    public static class StableReduction
    {
        // Physics-ID: DIAG-COMPENSATED-SUM
        // Implements: R042 R048
        // Model: Specification.md sections 12.2 and 14; diagnostic reduction only.
        // Sources: project algebraic invariant; Neumaier-style compensated double sum.
        // Source-Locators: independent cancellation tests, not an asserted physical closure.
        // Inputs-SI: finite contributions, all with one common declared unit.
        // Outputs-SI: sum in that same unit.
        // State-Ownership: local accumulator only.
        // Conservation: compensation retains low-order contributions; no inventory correction.
        // Discretisation: sequential compensated FP64 accumulation of GPU partials/body totals.
        // Validity: finite FP64 sum; overflowing/nonfinite inputs fail explicitly.
        // Validation: V02 V34
        // Evidence: PENDING
        // Status: IMPLEMENTED_UNVERIFIED
        // Limitations: GPU partial-reduction error remains a separate S03/V35 obligation.
        public static double Sum(IEnumerable<double> values)
        {
            var sum = new Accumulator();
            foreach (double value in values) sum.Add(value);
            return sum.Value;
        }

        public struct Accumulator
        {
            double sum, correction;
            public double Value
            {
                get { double result = sum + correction; SiUnits.RequireFinite(result, "reduction result"); return result; }
            }
            public void Add(double value)
            {
                SiUnits.RequireFinite(value, nameof(value));
                double next = sum + value;
                SiUnits.RequireFinite(next, "reduction intermediate");
                correction += Math.Abs(sum) >= Math.Abs(value) ? (sum - next) + value : (value - next) + sum;
                sum = next;
                SiUnits.RequireFinite(correction, "reduction compensation");
            }
        }
    }

    [DataContract]
    public sealed class InventoryState
    {
        [DataMember] public double iceKg;
        [DataMember] public double liquidKg;
        [DataMember] public double vapourKg;
        [DataMember] public double parcelWaterKg;
        [DataMember] public double dryAirKg;
        [DataMember] public double internalEnergyJ;
        [DataMember] public double kineticEnergyJ;
        [DataMember] public double gravitationalEnergyJ;
        [DataMember] public double solidInternalEnergyJ;
        [DataMember] public double interfaceEnergyJ;
        [DataMember] public double momentumXKgMPerS;
        [DataMember] public double momentumYKgMPerS;
        [DataMember] public double momentumZKgMPerS;
        [DataMember] public double angularMomentumXKgM2PerS;
        [DataMember] public double angularMomentumYKgM2PerS;
        [DataMember] public double angularMomentumZKgM2PerS;
        public const int Channels = 16;
        public double WaterKg => StableReduction.Sum(new[] { iceKg, liquidKg, vapourKg, parcelWaterKg });
        public double TotalEnergyJ => StableReduction.Sum(new[] { internalEnergyJ, kineticEnergyJ, gravitationalEnergyJ, solidInternalEnergyJ, interfaceEnergyJ });
        public InventoryState Copy() => FromArray(ToArray());
        public double[] ToArray() => new[] { iceKg, liquidKg, vapourKg, parcelWaterKg, dryAirKg,
            internalEnergyJ, kineticEnergyJ, gravitationalEnergyJ, solidInternalEnergyJ, interfaceEnergyJ,
            momentumXKgMPerS, momentumYKgMPerS, momentumZKgMPerS,
            angularMomentumXKgM2PerS, angularMomentumYKgM2PerS, angularMomentumZKgM2PerS };
        public static InventoryState FromArray(double[] x)
        {
            if (x == null || x.Length != Channels) throw new ArgumentException("Inventory channel count mismatch.");
            return new InventoryState { iceKg = x[0], liquidKg = x[1], vapourKg = x[2], parcelWaterKg = x[3], dryAirKg = x[4],
                internalEnergyJ = x[5], kineticEnergyJ = x[6], gravitationalEnergyJ = x[7], solidInternalEnergyJ = x[8], interfaceEnergyJ = x[9],
                momentumXKgMPerS = x[10], momentumYKgMPerS = x[11], momentumZKgMPerS = x[12],
                angularMomentumXKgM2PerS = x[13], angularMomentumYKgM2PerS = x[14], angularMomentumZKgM2PerS = x[15] };
        }
        public void ValidatePhysicalInventory()
        {
            double[] x = ToArray();
            for (int i = 0; i < x.Length; i++) SiUnits.RequireFinite(x[i], "inventory channel " + i);
            for (int i = 0; i < 5; i++) SiUnits.RequireNonnegative(x[i], "mass channel " + i);
            SiUnits.RequireNonnegative(kineticEnergyJ, "kinetic energy J");
            // Internal/gravitational energies and angular/linear momenta can have signed references.
        }
        // Physics-ID: DIAG-COMBINED-INVENTORY
        // Implements: R042 R059
        // Model: Specification.md sections 5 and 14; sum every owned inventory once.
        // Sources: project inventory definition; no material-property approximation.
        // Source-Locators: Specification.md section 14, included water/energy representations.
        // Inputs-SI: phase/dry-air mass [kg], energy [J], linear/angular momentum [kg m/s, kg m2/s].
        // Outputs-SI: combined inventory in the same units.
        // State-Ownership: new aggregate only; source states are read-only.
        // Conservation: separate phase mass is retained; no phase mass is counted as volume.
        // Discretisation: compensated FP64 accumulation per channel.
        // Validity: input inventories have common energy and angular-origin references.
        // Validation: V02 V34
        // Evidence: PENDING
        // Status: IMPLEMENTED_UNVERIFIED
        // Limitations: owner disjointness is the caller's responsibility; field reduction is later work.
        public static InventoryState Combine(IEnumerable<InventoryState> states)
        {
            var sums = new StableReduction.Accumulator[Channels];
            foreach (var state in states)
            {
                state.ValidatePhysicalInventory(); double[] values = state.ToArray();
                for (int i = 0; i < Channels; i++) sums[i].Add(values[i]);
            }
            var result = new double[Channels];
            for (int i = 0; i < Channels; i++) result[i] = sums[i].Value;
            return FromArray(result);
        }
    }

    [DataContract]
    public sealed class ConservationTolerance
    {
        // Absolute test scales, NOT universal solver tolerances; every benchmark records its own.
        [DataMember] public double massKg = 1e-12;
        [DataMember] public double energyJ = 1e-9;
        [DataMember] public double momentumKgMPerS = 1e-12;
        [DataMember] public double angularMomentumKgM2PerS = 1e-12;
        public void Validate()
        { SiUnits.RequireNonnegative(massKg, nameof(massKg)); SiUnits.RequireNonnegative(energyJ, nameof(energyJ));
          SiUnits.RequireNonnegative(momentumKgMPerS, nameof(momentumKgMPerS)); SiUnits.RequireNonnegative(angularMomentumKgM2PerS, nameof(angularMomentumKgM2PerS)); }

        public bool Matches(InventoryState a, InventoryState b)
        {
            Validate(); a.ValidatePhysicalInventory(); b.ValidatePhysicalInventory();
            if (Math.Abs(a.WaterKg - b.WaterKg) > massKg || Math.Abs(a.dryAirKg - b.dryAirKg) > massKg || Math.Abs(a.TotalEnergyJ - b.TotalEnergyJ) > energyJ) return false;
            double[] x = a.ToArray(), y = b.ToArray();
            for (int i = 10; i < x.Length; i++) if (Math.Abs(x[i] - y[i]) > (i < 13 ? momentumKgMPerS : angularMomentumKgM2PerS)) return false;
            return true;
        }
    }

    public sealed class ConservedInventory
    {
        public string Id { get; }
        StableReduction.Accumulator[] channels;
        public ConservedInventory(string id, InventoryState initial)
        {
            if (string.IsNullOrWhiteSpace(id)) throw new ArgumentException("Inventory ID required.");
            initial.ValidatePhysicalInventory(); Id = id; channels = new StableReduction.Accumulator[InventoryState.Channels];
            var values = initial.ToArray(); for (int i = 0; i < channels.Length; i++) channels[i].Add(values[i]);
        }
        public InventoryState Snapshot
        {
            get { var values = new double[channels.Length]; for (int i = 0; i < channels.Length; i++) values[i] = channels[i].Value; return InventoryState.FromArray(values); }
        }

        // Physics-ID: DIAG-PAIRED-INVENTORY-TRANSFER
        // Implements: R021 R042 R059
        // Model: Specification.md sections 5, 6.4 and 14; sum of internal transfers vanishes.
        // Sources: project conservative inventory contract; no phase-rate closure supplied.
        // Source-Locators: Specification.md 14, R_M and R_E definitions.
        // Inputs-SI: loss/gain mass [kg], storage energies [J], momentum [kg m/s], angular momentum [kg m2/s].
        // Outputs-SI: donor/receiver inventories in the same units, updated atomically.
        // State-Ownership: the two specified inventories only; candidates checked before commit.
        // Conservation: total water, dry air, total energy, linear/angular momentum must match.
        // Discretisation: compensated FP64 arithmetic; no clamping, no latent energy added here.
        // Validity: same thermodynamic/gravitational/angular-origin conventions on both sides.
        // Validation: V02 V34 (bookkeeping only, not a representation/phase solver).
        // Evidence: PENDING
        // Status: IMPLEMENTED_UNVERIFIED
        // Limitations: callers must derive correct phase enthalpy/work and geometry transfers.
        public static void Transfer(ConservedInventory donor, ConservedInventory receiver,
            InventoryState loss, InventoryState gain, ConservationTolerance tolerance)
        {
            if (donor == null || receiver == null || ReferenceEquals(donor, receiver) || donor.Id == receiver.Id) throw new ArgumentException("Distinct inventory owners and IDs required.");
            tolerance.Validate(); loss.ValidatePhysicalInventory(); gain.ValidatePhysicalInventory();
            RequireMatched(loss.WaterKg, gain.WaterKg, tolerance.massKg, "water mass");
            RequireMatched(loss.dryAirKg, gain.dryAirKg, tolerance.massKg, "dry air mass");
            RequireMatched(loss.TotalEnergyJ, gain.TotalEnergyJ, tolerance.energyJ, "total storage energy");
            var a = loss.ToArray(); var b = gain.ToArray();
            for (int i = 10; i < InventoryState.Channels; i++) RequireMatched(a[i], b[i], i < 13 ? tolerance.momentumKgMPerS : tolerance.angularMomentumKgM2PerS, "momentum channel " + i);
            var nextDonor = (StableReduction.Accumulator[])donor.channels.Clone();
            var nextReceiver = (StableReduction.Accumulator[])receiver.channels.Clone();
            for (int i = 0; i < nextDonor.Length; i++) { nextDonor[i].Add(-a[i]); nextReceiver[i].Add(b[i]); }
            ValidateCandidate(nextDonor); ValidateCandidate(nextReceiver);
            donor.channels = nextDonor; receiver.channels = nextReceiver;
        }
        static void ValidateCandidate(StableReduction.Accumulator[] candidate)
        { var values = new double[candidate.Length]; for (int i = 0; i < candidate.Length; i++) values[i] = candidate[i].Value; InventoryState.FromArray(values).ValidatePhysicalInventory(); }
        static void RequireMatched(double a, double b, double tolerance, string channel)
        { if (Math.Abs(a - b) > tolerance) throw new InvalidOperationException("Unmatched internal transfer: " + channel + "."); }
    }
}
