using System;
using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // Each parcel represents identical seeded droplets sharing one gas inventory.
    internal static class SeededFogCell
    {
        // Independent-droplet extinction in the visible geometric-optics limit.
        internal static double VisibleExtinctionPerM(
            (double RadiusM, double TemperatureK, int Multiplicity)[] parcels, double volumeM3)
        {
            if (!(volumeM3 > 0) || !SiUnits.IsFinite(volumeM3))
                throw new ArgumentOutOfRangeException(nameof(volumeM3));
            double crossSection = 0;
            foreach (var parcel in parcels)
            {
                if (!(parcel.RadiusM >= 5e-6) || !SiUnits.IsFinite(parcel.RadiusM) || parcel.Multiplicity <= 0)
                    throw new ArgumentException("Visible geometric-optics extinction requires radii of at least 5 micrometres and positive multiplicities.");
                crossSection += 2 * Math.PI * parcel.RadiusM * parcel.RadiusM * parcel.Multiplicity;
            }
            double extinction = crossSection / volumeM3;
            if (!SiUnits.IsFinite(extinction)) throw new ArgumentException("Fog extinction exceeds the supported range.");
            return extinction;
        }

        internal static (CutCellInventories Fields, CutCellMomentumResult Momentum) ReinsertWithFaceMomentum(
            VariableCoefficientProjection plan, CutCellInventories state, double[] displacement,
            float[] faceVelocity, double gasDensity, double radiusM, double temperatureK, int multiplicity, D3 velocity)
        {
            if (!(gasDensity > 0) || !SiUnits.IsFinite(gasDensity))
                throw new ArgumentOutOfRangeException(nameof(gasDensity));
            var reinserted = ReinsertResolvedLiquid(plan, state, displacement, radiusM, temperatureK, multiplicity, velocity);
            var remap = new CutCellMomentum(plan);
            double liquidDensity = DropletGasCellExchange.LiquidDensity;
            var momentum = remap.GatherGroupMomentum(state, faceVelocity, liquidDensity, gasDensity);
            for (int axis = 0; axis < 3; axis++)
            for (int group = 0; group < plan.GroupCount; group++)
                momentum[axis][group] += reinserted.LiquidMomentum[group][axis];
            return (reinserted.Fields, remap.ScatterGroupMomentum(reinserted.Fields, momentum, liquidDensity, gasDensity));
        }

        // Deposit unresolved occupancy in the pressure group containing the centre.
        // Mixed groups and footprints that would fill the group require a different remap.
        internal static double[] SelectParcelFootprint(VariableCoefficientProjection plan,
            CutCellInventories state, D3 position, double radiusM, int multiplicity)
        {
            if (!(radiusM > 0) || !SiUnits.IsFinite(radiusM) || multiplicity <= 0 ||
                state.LiquidM3.Length != plan.GroupCount)
                throw new ArgumentException("Parcel footprint requires a valid parcel and matching VOF groups.");
            int group = DropletGasCellExchange.PressureGroupAt(plan, position);
            double volume = DropletGasCellExchange.Mass(radiusM) * multiplicity / DropletGasCellExchange.LiquidDensity;
            if (state.LiquidM3[group] != 0 || !(volume > 0 && volume < plan.Volumes[group]))
                throw new InvalidOperationException("Parcel footprint requires a gas-only group with sufficient volume.");
            var displacement = new double[plan.GroupCount];
            displacement[group] = volume;
            return displacement;
        }

        // Reinsert a parcel into its reserved footprint. The caller transfers
        // the returned momentum into the resolved momentum inventory.
        internal static (CutCellInventories Fields, D3[] LiquidMomentum) ReinsertResolvedLiquid(
            VariableCoefficientProjection plan, CutCellInventories state, double[] displacement,
            double radiusM, double temperatureK, int multiplicity, D3 velocity)
        {
            var parcel = ToResolvedLiquid(radiusM, temperatureK, multiplicity, velocity);
            if (displacement.Length != plan.GroupCount || state.LiquidM3.Length != plan.GroupCount)
                throw new ArgumentException("Parcel footprint must match the pressure groups.");
            double reserved = 0;
            for (int group = 0; group < displacement.Length; group++)
            {
                double occupied = displacement[group];
                if (!(occupied >= 0 && occupied <= plan.Volumes[group]) ||
                    Math.Abs(state.LiquidM3[group] + state.GasM3[group] + occupied - plan.Volumes[group]) >
                        1e-10 * plan.Volumes[group])
                    throw new ArgumentException("Parcel footprint does not close the geometric volume.");
                reserved += occupied;
            }
            if (Math.Abs(reserved - parcel.LiquidM3) > 1e-12 * parcel.LiquidM3)
                throw new ArgumentException("Parcel liquid volume must match its reserved footprint.");

            var liquid = (double[])state.LiquidM3.Clone();
            var energy = (double[])state.LiquidEnergyJ.Clone();
            var momentum = new D3[liquid.Length];
            for (int group = 0; group < liquid.Length; group++)
            {
                double fraction = displacement[group] / reserved;
                liquid[group] += displacement[group];
                energy[group] += parcel.EnthalpyJ * fraction;
                momentum[group] = parcel.Momentum * fraction;
            }
            return (new CutCellInventories(liquid, (double[])state.GasM3.Clone(), energy,
                (double[])state.GasEnergyJ.Clone(), (double[])state.VapourKg.Clone()), momentum);
        }

        // Anchors identify bulk liquid. Components touching an open liquid face
        // remain resolved so their inventory can be handled by the boundary flux.
        internal static List<int[]> DetachedLiquidGroups(VariableCoefficientProjection plan, CutCellInventories fields,
            bool[] anchors, double maxVolumeM3)
        {
            if (plan.IsPeriodic || anchors.Length != plan.GroupCount || !(maxVolumeM3 > 0) ||
                !SiUnits.IsFinite(maxVolumeM3))
                throw new ArgumentException("Invalid detached-liquid topology or threshold.");
            var areas = new CutCellVofTransport(plan).ReconstructLiquidFaceAreas(fields);
            var visited = new bool[plan.GroupCount];
            var detached = new List<int[]>();
            for (int group = 0; group < visited.Length; group++)
            {
                if (visited[group] || fields.LiquidM3[group] == 0) continue;
                var component = new List<int> { group };
                visited[group] = true;
                bool attached = false;
                double volume = 0;
                for (int cursor = 0; cursor < component.Count; cursor++)
                {
                    int current = component[cursor];
                    attached |= anchors[current];
                    volume += fields.LiquidM3[current];
                    for (int entry = plan.Rows[current]; entry < plan.Rows[current + 1]; entry++)
                    {
                        int signed = plan.Incidence[entry], index = Math.Abs(signed) - 1;
                        var face = plan.Faces[index];
                        if (areas[index] <= 1e-10 * face.Area) continue;
                        int next = signed > 0 ? face.High : face.Low;
                        if (next < 0) attached = true;
                        else if (!visited[next])
                        {
                            visited[next] = true;
                            component.Add(next);
                        }
                    }
                }
                if (!attached && volume <= maxVolumeM3) detached.Add(component.ToArray());
            }
            return detached;
        }

        // Return a separate displacement inventory. Gas volume and scalars stay
        // physical, and the caller must account for this volume in projection.
        internal static (CutCellInventories Fields, double[] ParcelDisplacementM3,
            (double RadiusM, double TemperatureK, int Multiplicity, D3 Velocity) Parcel) DebitResolvedLiquid(
            CutCellInventories fields, int[] component, D3 componentMomentum)
        {
            var liquid = (double[])fields.LiquidM3.Clone();
            var enthalpy = (double[])fields.LiquidEnergyJ.Clone();
            var displacement = new double[liquid.Length];
            double volume = 0, energy = 0;
            foreach (int group in component)
            {
                if (!(liquid[group] > 0)) throw new ArgumentException("Conversion requires unique wet groups.");
                volume += liquid[group];
                energy += enthalpy[group];
                displacement[group] = liquid[group];
                liquid[group] = 0;
                enthalpy[group] = 0;
            }
            return (new CutCellInventories(liquid, (double[])fields.GasM3.Clone(), enthalpy,
                (double[])fields.GasEnergyJ.Clone(), (double[])fields.VapourKg.Clone()), displacement,
                FromResolvedLiquid(volume, energy, componentMomentum, 1));
        }

        internal static (double RadiusM, double TemperatureK, int Multiplicity, D3 Velocity) FromResolvedLiquid(
            double liquidM3, double enthalpyJ, D3 momentum, int multiplicity)
        {
            if (!(liquidM3 > 0) || !SiUnits.IsFinite(liquidM3) || multiplicity <= 0)
                throw new ArgumentException("Conversion requires positive liquid volume and multiplicity.");
            double mass = liquidM3 * DropletGasCellExchange.LiquidDensity;
            double temperature = DropletGasCellExchange.ReferenceK + enthalpyJ / (mass * DropletGasCellExchange.LiquidCp);
            if (!(temperature >= DropletGasCellExchange.ReferenceK && temperature <= 320))
                throw new ArgumentException("Resolved liquid enthalpy is outside the parcel model range.");
            return (Math.Pow(liquidM3 / multiplicity * (3 / (4 * Math.PI)), 1.0 / 3),
                temperature, multiplicity, momentum / mass);
        }

        internal static (double LiquidM3, double EnthalpyJ, D3 Momentum) ToResolvedLiquid(
            double radiusM, double temperatureK, int multiplicity, D3 velocity)
        {
            if (!(radiusM > 0) || !SiUnits.IsFinite(radiusM) || multiplicity <= 0 ||
                !(temperatureK >= DropletGasCellExchange.ReferenceK && temperatureK <= 320))
                throw new ArgumentException("Conversion requires a valid liquid parcel.");
            double mass = DropletGasCellExchange.Mass(radiusM) * multiplicity;
            return (mass / DropletGasCellExchange.LiquidDensity, mass * DropletGasCellExchange.LiquidCp *
                (temperatureK - DropletGasCellExchange.ReferenceK), velocity * mass);
        }

        internal static ((D3 Position, double RadiusM, double TemperatureK, int Multiplicity)[] Parcels,
            (double DryAirKg, double VapourKg, double EnthalpyJ)[] Gas, D3[] ParcelVelocity, D3[] GasMomentum,
            (long Droplets, double LiquidKg, double EnthalpyJ, D3 Momentum) Outlet,
            (long Droplets, double LiquidKg, double EnthalpyJ, D3 Momentum) Wall)
            AdvanceMovingGrid(VariableCoefficientProjection plan,
                (D3 Position, double RadiusM, double TemperatureK, int Multiplicity)[] parcels,
                (double DryAirKg, double VapourKg, double EnthalpyJ)[] gas, D3[] parcelVelocity, D3[] gasMomentum,
                double pressurePa, double conductivity, double diffusivity, double viscosity, double dt,
                (int Axis, bool Positive)? stickingWall = null)
        {
            if (parcelVelocity.Length != parcels.Length || gasMomentum.Length != gas.Length)
                throw new ArgumentException("Fog velocity and gas momentum must match their inventories.");
            var exchange = AdvanceGrid(plan, parcels, gas, default, pressurePa, conductivity, diffusivity, dt);
            var retained = new List<(D3 Position, double RadiusM, double TemperatureK, int Multiplicity)>(parcels.Length);
            var velocity = new List<D3>(parcels.Length);
            var momentum = (D3[])gasMomentum.Clone();
            (long Droplets, double LiquidKg, double EnthalpyJ, D3 Momentum) outlet = default;
            (long Droplets, double LiquidKg, double EnthalpyJ, D3 Momentum) wall = default;
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
                momentum[group] += motion.GasMomentumGain * parcel.Multiplicity;
                var updated = exchange.Parcels[index];
                bool sticks = stickingWall.HasValue && DropletGasCellExchange.ContactsBoxWall(plan,
                    stickingWall.Value.Axis, stickingWall.Value.Positive, parcel.Position, motion.Position, updated.RadiusM);
                bool exits = DropletGasCellExchange.ExitsOpenBoundary(plan, parcel.Position, motion.Position);
                if (sticks && exits)
                    throw new InvalidOperationException("A parcel reached a sticking wall and an outlet in the same step. Reduce the timestep.");
                if (exits || sticks)
                {
                    ref var ledger = ref (sticks ? ref wall : ref outlet);
                    double liquid = DropletGasCellExchange.Mass(updated.RadiusM) * updated.Multiplicity;
                    ledger.Droplets += updated.Multiplicity;
                    ledger.LiquidKg += liquid;
                    ledger.EnthalpyJ += liquid * DropletGasCellExchange.LiquidCp *
                        (updated.TemperatureK - DropletGasCellExchange.ReferenceK);
                    ledger.Momentum += motion.Velocity * liquid;
                }
                else
                {
                    DropletGasCellExchange.PressureGroupAt(plan, motion.Position);
                    updated.Position = motion.Position;
                    retained.Add(updated);
                    velocity.Add(motion.Velocity);
                }
            }
            return (retained.ToArray(), exchange.Gas, velocity.ToArray(), momentum, outlet, wall);
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
