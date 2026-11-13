using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Materials;

namespace PhysicsSim.ThreePhase3D.Diagnostics
{
    // Reduce disjoint inventories using the same material references as the boundary audit.
    internal struct EntropyInventory
    {
        private StableReduction.Accumulator entropy;

        internal double TotalJPerK => entropy.Value;

        internal void AddMaterial(double massKg, ThermodynamicState state)
        {
            SiUnits.RequireNonnegative(massKg, nameof(massKg));
            if (massKg == 0) return;
            if (state == null) throw new ArgumentNullException(nameof(state));
            entropy.Add(massKg * state.EntropyJkgK);
        }

        internal void AddSensibleStorage(double capacityJPerK, double kelvin, double referenceKelvin)
        {
            SiUnits.RequireNonnegative(capacityJPerK, nameof(capacityJPerK));
            if (!(kelvin > 0) || !(referenceKelvin > 0) ||
                !SiUnits.IsFinite(kelvin) || !SiUnits.IsFinite(referenceKelvin))
                throw new ArgumentOutOfRangeException(nameof(kelvin), "Entropy storage requires positive absolute temperatures.");
            entropy.Add(capacityJPerK * (Math.Log(kelvin) - Math.Log(referenceKelvin)));
        }
    }
}
