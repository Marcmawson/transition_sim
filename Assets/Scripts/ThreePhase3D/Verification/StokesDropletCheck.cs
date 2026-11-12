using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;

namespace PhysicsSim.ThreePhase3D.Verification
{
    internal static class StokesDropletCheck
    {
        internal static void Run(Action<string> log)
        {
            const double liquidDensity = 998, gasDensity = 1.2, viscosity = 1.81e-5;
            const double gravity = 9.81, duration = 0.2;
            var acceleration = new D3(0, -gravity, 0);
            foreach (double radius in new[] { 5e-6, 10e-6, 15e-6 })
            {
                double terminal = 2 * (liquidDensity - gasDensity) * gravity * radius * radius / (9 * viscosity);
                double relaxation = 2 * liquidDensity * radius * radius / (9 * viscosity);
                double decay = Math.Exp(-duration / relaxation);
                double expectedDistance = terminal * (duration - relaxation * (1 - decay));
                double previousError = double.PositiveInfinity;
                foreach (int steps in new[] { 100, 200, 400 })
                {
                    D3 position = default, velocity = default, gasImpulse = default;
                    for (int step = 0; step < steps; step++)
                    {
                        var result = StokesDroplet.Advance(position, velocity, default, acceleration,
                            radius, liquidDensity, gasDensity, viscosity, duration / steps);
                        position = result.Position;
                        velocity = result.Velocity;
                        gasImpulse += result.GasDragImpulse;
                    }
                    double volume = (4 * Math.PI / 3) * Math.Pow(radius, 3);
                    double momentumResidual = velocity.Y * liquidDensity * volume + gasImpulse.Y +
                        gravity * (liquidDensity - gasDensity) * volume * duration;
                    double error = Math.Abs(-position.Y - expectedDistance);
                    if (Math.Abs(-velocity.Y - terminal * (1 - decay)) > 1e-12 * terminal ||
                        error >= previousError || Math.Abs(momentumResidual) > 1e-24)
                        throw new InvalidOperationException("S14 droplet settling failed velocity, timestep convergence or drag impulse closure.");
                    previousError = error;
                }
                if (previousError > 0.001 * expectedDistance)
                    throw new InvalidOperationException("S14 droplet settling exceeded its final trajectory tolerance.");
                log(FormattableString.Invariant(
                    $"S14 V37 Stokes settling passed: radius {radius * 1e6:G3} um, terminal speed {terminal:G8} m/s, fine trajectory error {previousError:G5} m."));
            }
        }
    }
}
