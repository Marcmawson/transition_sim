using System;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // Constant-radius sphere in locally uniform, continuum gas at small slip Reynolds number.
    internal static class StokesDroplet
    {
        internal static (D3 Position, D3 Velocity, D3 GasMomentumGain) Advance(
            D3 position, D3 velocity, D3 gasVelocity, D3 gravity, double radius,
            double liquidDensity, double gasDensity, double gasViscosity, double dt, double vapourMassGainKg = 0)
        {
            if (!Positive(radius) || !Positive(liquidDensity) || !Positive(gasDensity) ||
                !Positive(gasViscosity) || !Positive(dt))
                throw new ArgumentOutOfRangeException(nameof(radius), "Radius, densities, viscosity and timestep must be positive and finite.");

            double mass = liquidDensity * (4 * Math.PI / 3) * radius * radius * radius;
            if (!SiUnits.IsFinite(vapourMassGainKg) || vapourMassGainKg > mass)
                throw new ArgumentOutOfRangeException(nameof(vapourMassGainKg));
            double relaxation = mass / (6 * Math.PI * gasViscosity * radius);
            D3 acceleration = gravity * (1 - gasDensity / liquidDensity);
            D3 initialSlip = velocity - gasVelocity;
            D3 terminalSlip = acceleration * relaxation;
            double ratio = dt / relaxation;
            // Preserve the increment when exp(-ratio) is close to one.
            double response = ratio < 1e-5 ? ratio * (1 - ratio * (0.5 - ratio / 6)) : 1 - Math.Exp(-ratio);
            D3 change = (terminalSlip - initialSlip) * response;
            D3 updated = velocity + change;
            double reynolds = 2 * radius * gasDensity / gasViscosity *
                Math.Max(initialSlip.Length, (updated - gasVelocity).Length);
            if (!(reynolds <= 0.1))
                throw new InvalidOperationException("Stokes droplet drag requires a slip Reynolds number at or below 0.1.");

            // Apply drag first. Phase transfer then carries the donor phase velocity.
            D3 nextPosition = position + (velocity + updated) * (0.5 * dt);
            D3 phaseMomentum = (vapourMassGainKg < 0 ? gasVelocity : updated) * vapourMassGainKg;
            if (vapourMassGainKg < 0)
                updated += (gasVelocity - updated) * (-vapourMassGainKg / (mass - vapourMassGainKg));
            return (nextPosition, updated, (acceleration * dt - change) * mass + phaseMomentum);
        }

        private static bool Positive(double value) => SiUnits.IsFinite(value) && value > 0;
    }
}
