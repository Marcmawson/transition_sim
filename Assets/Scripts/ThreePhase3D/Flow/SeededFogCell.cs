using System;
using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // Each parcel represents identical seeded droplets sharing one gas inventory.
    internal static class SeededFogCell
    {
        internal static ((D3 Position, double RadiusM, double TemperatureK, int Multiplicity)[] Parcels,
            (double DryAirKg, double VapourKg, double EnthalpyJ)[] Gas, D3[] ParcelVelocity, D3[] GasMomentum)
            AdvanceMovingGrid(VariableCoefficientProjection plan,
                (D3 Position, double RadiusM, double TemperatureK, int Multiplicity)[] parcels,
                (double DryAirKg, double VapourKg, double EnthalpyJ)[] gas, D3[] parcelVelocity, D3[] gasMomentum,
                double pressurePa, double conductivity, double diffusivity, double viscosity, double dt)
        {
            if (parcelVelocity.Length != parcels.Length || gasMomentum.Length != gas.Length)
                throw new ArgumentException("Fog velocity and gas momentum must match their inventories.");
            var exchange = AdvanceGrid(plan, parcels, gas, default, pressurePa, conductivity, diffusivity, dt);
            var velocity = new D3[parcels.Length];
            var momentum = (D3[])gasMomentum.Clone();
            for (int index = 0; index < parcels.Length; index++)
            {
                var parcel = parcels[index];
                int group = DropletGasCellExchange.PressureGroupAt(plan, parcel.Position);
                var donor = gas[group];
                double mass = donor.DryAirKg + donor.VapourKg;
                double volume = DropletGasCellExchange.GasVolumeM3(donor.DryAirKg, donor.VapourKg, donor.EnthalpyJ, pressurePa);
                double vapourGain = DropletGasCellExchange.Mass(parcel.RadiusM) -
                    DropletGasCellExchange.Mass(exchange.Parcels[index].RadiusM);
                var motion = StokesDroplet.Advance(parcel.Position, parcelVelocity[index], gasMomentum[group] / mass,
                    default, parcel.RadiusM, DropletGasCellExchange.LiquidDensity, mass / volume, viscosity, dt, vapourGain);
                DropletGasCellExchange.PressureGroupAt(plan, motion.Position);
                exchange.Parcels[index].Position = motion.Position;
                velocity[index] = motion.Velocity;
                momentum[group] += motion.GasMomentumGain * parcel.Multiplicity;
            }
            return (exchange.Parcels, exchange.Gas, velocity, momentum);
        }

        internal static ((D3 Position, double RadiusM, double TemperatureK, int Multiplicity)[] Parcels,
            (double DryAirKg, double VapourKg, double EnthalpyJ)[] Gas) AdvanceGrid(
            VariableCoefficientProjection plan, (D3 Position, double RadiusM, double TemperatureK, int Multiplicity)[] parcels,
            (double DryAirKg, double VapourKg, double EnthalpyJ)[] gas, D3 carrierVelocity,
            double pressurePa, double conductivity, double diffusivity, double dt)
        {
            if (gas.Length != plan.GroupCount || !SiUnits.IsFinite(dt) || dt <= 0)
                throw new ArgumentException("Fog transport requires a matching gas grid and positive finite timestep.");
            var groups = new List<int>[gas.Length];
            var moved = ((D3 Position, double RadiusM, double TemperatureK, int Multiplicity)[])parcels.Clone();
            for (int index = 0; index < parcels.Length; index++)
            {
                int group = DropletGasCellExchange.PressureGroupAt(plan, parcels[index].Position);
                moved[index].Position += carrierVelocity * dt;
                DropletGasCellExchange.PressureGroupAt(plan, moved[index].Position);
                (groups[group] ??= new List<int>()).Add(index);
            }

            var updatedGas = ((double DryAirKg, double VapourKg, double EnthalpyJ)[])gas.Clone();
            for (int group = 0; group < groups.Length; group++)
            {
                var indices = groups[group];
                if (indices == null) continue;
                var local = new (double RadiusM, double TemperatureK, int Multiplicity)[indices.Count];
                for (int slot = 0; slot < indices.Count; slot++)
                {
                    var parcel = parcels[indices[slot]];
                    local[slot] = (parcel.RadiusM, parcel.TemperatureK, parcel.Multiplicity);
                }
                var state = gas[group];
                var exchange = Advance(local, state.DryAirKg, state.VapourKg, state.EnthalpyJ,
                    pressurePa, conductivity, diffusivity, dt);
                updatedGas[group] = (state.DryAirKg, exchange.VapourKg, exchange.GasEnthalpyJ);
                for (int slot = 0; slot < indices.Count; slot++)
                {
                    int index = indices[slot];
                    var parcel = exchange.Parcels[slot];
                    moved[index] = (moved[index].Position, parcel.RadiusM, parcel.TemperatureK, parcel.Multiplicity);
                }
            }
            return (moved, updatedGas);
        }

        internal static ((double RadiusM, double TemperatureK, int Multiplicity)[] Parcels,
            double VapourKg, double GasEnthalpyJ) Advance(
            (double RadiusM, double TemperatureK, int Multiplicity)[] parcels,
            double dryAirKg, double vapourKg, double gasEnthalpyJ, double pressurePa,
            double conductivity, double diffusivity, double dt)
        {
            long seedCount = 0;
            foreach (var parcel in parcels)
            {
                if (parcel.Multiplicity <= 0 || !(parcel.RadiusM > 0))
                    throw new ArgumentException("Fog parcels require a positive seed radius and multiplicity.", nameof(parcels));
                seedCount += parcel.Multiplicity;
            }
            if (seedCount == 0) return (Array.Empty<(double, double, int)>(), vapourKg, gasEnthalpyJ);

            // Each seed sees the same starting gas state. The per-seed share
            // depends on physical count, so splitting a parcel does not change the update.
            double seedAir = dryAirKg / seedCount, seedVapour = vapourKg / seedCount;
            double seedEnthalpy = gasEnthalpyJ / seedCount;
            var updated = new (double RadiusM, double TemperatureK, int Multiplicity)[parcels.Length];
            double vapourChange = 0, enthalpyChange = 0;
            for (int index = 0; index < parcels.Length; index++)
            {
                var parcel = parcels[index];
                var exchange = DropletGasCellExchange.Advance(parcel.RadiusM, parcel.TemperatureK,
                    seedAir, seedVapour, seedEnthalpy, pressurePa, conductivity, diffusivity, dt, allowCondensation: true);
                updated[index] = (exchange.RadiusM, exchange.LiquidK, parcel.Multiplicity);
                vapourChange += (exchange.VapourKg - seedVapour) * parcel.Multiplicity;
                enthalpyChange += (exchange.GasEnthalpyJ - seedEnthalpy) * parcel.Multiplicity;
            }
            double updatedVapour = vapourKg + vapourChange;
            double updatedEnthalpy = gasEnthalpyJ + enthalpyChange;
            if (updatedVapour < 0)
                throw new InvalidOperationException("Fog condensation would overdraw the cell vapour inventory.");
            DropletGasCellExchange.GasVolumeM3(dryAirKg, updatedVapour, updatedEnthalpy, pressurePa);
            return (updated, updatedVapour, updatedEnthalpy);
        }
    }
}
