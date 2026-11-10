using System;
using System.Collections.Generic;
using System.Runtime.Serialization;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Diagnostics
{
    public enum ExternalChannel { HeaterAbsorbed, OpenBoundary, SupportConduction, Radiation, MechanicalWork, Injection, NumericalCorrection }
    public enum FluxDirection { IntoDomain, OutOfDomain }

    [DataContract]
    public sealed class ExternalTotals
    {
        [DataMember] public string channel;
        [DataMember] public double waterInKg, dryAirInKg, energyInJ, waterOutKg, dryAirOutKg, energyOutJ;
    }

    [DataContract]
    public sealed class LedgerResidual
    {
        [DataMember] public double waterKg;
        [DataMember] public double dryAirKg;
        [DataMember] public double energyJ;
        [DataMember] public bool passed;
    }

    public sealed class ConservationLedger
    {
        readonly InventoryState initial;
        readonly Dictionary<ExternalChannel, StableReduction.Accumulator[]> channels = new Dictionary<ExternalChannel, StableReduction.Accumulator[]>();
        readonly ConservationTolerance tolerance;
        bool numericalCorrection;
        public ConservationLedger(InventoryState initial, ConservationTolerance tolerance)
        {
            initial.ValidatePhysicalInventory(); tolerance.Validate(); this.initial = initial.Copy();
            this.tolerance = new ConservationTolerance { massKg = tolerance.massKg, energyJ = tolerance.energyJ,
                momentumKgMPerS = tolerance.momentumKgMPerS, angularMomentumKgM2PerS = tolerance.angularMomentumKgM2PerS };
            foreach (ExternalChannel channel in Enum.GetValues(typeof(ExternalChannel))) channels.Add(channel, new StableReduction.Accumulator[6]);
        }
        // Physics-ID: DIAG-EXTERNAL-INVENTORY-FLUX
        // Implements: R042 R057
        // Model: Specification.md section 14, external source and boundary terms.
        // Sources: project control-volume energy/mass convention.
        // Source-Locators: Specification.md section 14; EvidenceProtocol.md section 4.
        // Inputs-SI: integrated accepted boundary water/dry-air mass [kg], energy [J], direction [1].
        // Outputs-SI: cumulative per-channel input/output mass and energy [kg, J].
        // State-Ownership: external ledger only, not physical inventories.
        // Conservation: absorbed heater is a subset of energy input, counted exactly once.
        // Discretisation: compensated FP64 sums of already time-integrated accepted fluxes.
        // Validity: advected energy may be signed by thermodynamic reference; heat/work is nonnegative with explicit direction.
        // Validation: V02 V33
        // Evidence: PENDING
        // Status: IMPLEMENTED_UNVERIFIED
        // Limitations: caller must include pressure work with the chosen boundary enthalpy convention.
        public void RecordExternal(ExternalChannel channel, FluxDirection direction, double waterKg, double dryAirKg, double energyJ)
        {
            if (!channels.ContainsKey(channel) || !Enum.IsDefined(typeof(FluxDirection), direction)) throw new ArgumentException("Unknown external channel/direction.");
            SiUnits.RequireNonnegative(waterKg, nameof(waterKg)); SiUnits.RequireNonnegative(dryAirKg, nameof(dryAirKg)); SiUnits.RequireFinite(energyJ, nameof(energyJ));
            bool transportsMass = channel == ExternalChannel.OpenBoundary || channel == ExternalChannel.Injection || channel == ExternalChannel.NumericalCorrection;
            if (!transportsMass && (waterKg != 0 || dryAirKg != 0 || energyJ < 0)) throw new ArgumentException("Heat/work channels carry nonnegative energy and no mass; use an explicit direction.");
            if (channel == ExternalChannel.HeaterAbsorbed && (waterKg != 0 || dryAirKg != 0 || energyJ < 0 || direction != FluxDirection.IntoDomain))
                throw new ArgumentException("Absorbed heater channel is nonnegative incoming heat only.");
            // Boundary energy can be signed with the selected internal-energy/enthalpy reference.
            // Direction is the mass/flux normal convention, not a requirement that enthalpy be positive.
            int offset = direction == FluxDirection.IntoDomain ? 0 : 3;
            var next = (StableReduction.Accumulator[])channels[channel].Clone();
            next[offset].Add(waterKg); next[offset + 1].Add(dryAirKg); next[offset + 2].Add(energyJ);
            channels[channel] = next;
            if (channel == ExternalChannel.NumericalCorrection && (waterKg != 0 || dryAirKg != 0 || energyJ != 0)) numericalCorrection = true;
        }
        public double WaterInKg => Total(0);
        public double WaterOutKg => Total(3);
        public double DryAirInKg => Total(1);
        public double DryAirOutKg => Total(4);
        // HeaterAbsorbedJ is a labelled subset of EnergyInJ, never added to it twice.
        public double EnergyInJ => Total(2);
        public double EnergyOutJ => Total(5);
        public double HeaterAbsorbedJ => channels[ExternalChannel.HeaterAbsorbed][2].Value;

        public ExternalTotals[] ExternalChannelTotals()
        {
            var result = new List<ExternalTotals>();
            foreach (ExternalChannel channel in Enum.GetValues(typeof(ExternalChannel)))
            {
                var c = channels[channel];
                result.Add(new ExternalTotals { channel = channel.ToString(), waterInKg = c[0].Value, dryAirInKg = c[1].Value, energyInJ = c[2].Value,
                    waterOutKg = c[3].Value, dryAirOutKg = c[4].Value, energyOutJ = c[5].Value });
            }
            return result.ToArray();
        }

        // Physics-ID: DIAG-INDEPENDENT-CONSERVATION-AUDIT
        // Implements: R042 R056
        // Model: Specification.md section 14: R_M=M-M0-Min+Mout; R_E=E-E0-Ein+Eout.
        // Sources: project control-volume balance; storage convention documented in manifest.
        // Source-Locators: Specification.md section 14, water and total-energy residual definitions.
        // Inputs-SI: independently reduced CURRENT mass [kg], total storage energy [J].
        // Outputs-SI: residuals [kg, J] and tolerance/failure flags.
        // State-Ownership: none; audit never adjusts the current state.
        // Conservation: internal transfers are absent from external sums; humid inflow water counts.
        // Discretisation: compensated FP64 reductions; absolute benchmark tolerances.
        // Validity: every external heat/work/advected-energy channel must be recorded once.
        // Validation: V02 V34 V33
        // Evidence: PENDING
        // Status: IMPLEMENTED_UNVERIFIED
        // Limitations: a self-reported wrong boundary flux needs independent flux verification later.
        public LedgerResidual Audit(InventoryState independentlyReducedCurrent)
        {
            independentlyReducedCurrent.ValidatePhysicalInventory();
            var result = new LedgerResidual {
                waterKg = StableReduction.Sum(new[] { independentlyReducedCurrent.WaterKg, -initial.WaterKg, -WaterInKg, WaterOutKg }),
                dryAirKg = StableReduction.Sum(new[] { independentlyReducedCurrent.dryAirKg, -initial.dryAirKg, -DryAirInKg, DryAirOutKg }),
                energyJ = StableReduction.Sum(new[] { independentlyReducedCurrent.TotalEnergyJ, -initial.TotalEnergyJ, -EnergyInJ, EnergyOutJ })
            };
            result.passed = !numericalCorrection && Math.Abs(result.waterKg) <= tolerance.massKg && Math.Abs(result.dryAirKg) <= tolerance.massKg && Math.Abs(result.energyJ) <= tolerance.energyJ;
            return result;
        }

        double Total(int channel)
        {
            var values = new List<double>();
            foreach (ExternalChannel key in Enum.GetValues(typeof(ExternalChannel))) values.Add(channels[key][channel].Value);
            return StableReduction.Sum(values);
        }
    }
}
