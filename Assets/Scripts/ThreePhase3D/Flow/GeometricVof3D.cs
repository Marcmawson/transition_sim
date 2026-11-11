using System;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // Physics-ID: FLOW-PLIC-TRANSPORT-3D
    // Implements: R018
    // Model: Specification.md sections 4 and 6; piecewise-linear interface reconstruction.
    // Sources: project specification; M04 and M13 motivate geometric and compatible transport.
    // Source-Locators: S05-Geometric-Transport.md derivation; Validation.md V09 V10.
    // Inputs-SI: phase fraction [1], face velocity [m/s], density [kg/m3], dt [s].
    // Outputs-SI: phase transfers [m3], staggered momentum [kg m/s], energy [J], species [kg].
    // State-Ownership: an advance reads the accepted fields and returns new fields.
    // Conservation: shared face transfers with equal and opposite cell contributions.
    // Discretisation: PLIC slab integration, donor capacity limiter, first-order dual momentum.
    // Validity: uniform Cartesian spacings, periodic domain, solenoidal face field, fixed phase densities.
    // Validation: V09 V10
    // Evidence: PENDING
    // Status: IMPLEMENTED_UNVERIFIED
    // Limitations: regular periodic verification path; solid cut cells and projection coupling remain open.
    public sealed class GeometricVof3D
    {
        private const double Roundoff = 128 * 2.2204460492503131e-16;
        private readonly int[][] next = new int[3][], previous = new int[3][];
        private readonly double liquidDensity, gasDensity;
        public Grid3D Grid { get; }
        public int CellCount { get; }

        public GeometricVof3D(Grid3D grid, double liquidDensity, double gasDensity)
        {
            Grid = grid ?? throw new ArgumentNullException(nameof(grid));
            if (!SiUnits.IsFinite(liquidDensity) || !SiUnits.IsFinite(gasDensity) || liquidDensity <= 0 || gasDensity <= 0 ||
                !SiUnits.IsFinite(grid.CellVolume * Math.Max(liquidDensity, gasDensity)) ||
                grid.CellVolume * Math.Min(liquidDensity, gasDensity) == 0)
                throw new ArgumentException("Both phase densities and full-cell masses must be finite and positive.");
            this.liquidDensity = liquidDensity;
            this.gasDensity = gasDensity;
            var c = grid.Cells;
            CellCount = checked(c.X * c.Y * c.Z);
            for (int axis = 0; axis < 3; axis++)
            {
                next[axis] = new int[CellCount];
                previous[axis] = new int[CellCount];
                for (int k = 0; k < c.Z; k++)
                for (int j = 0; j < c.Y; j++)
                for (int i = 0; i < c.X; i++)
                {
                    int cell = CellIndex(i, j, k);
                    int n = CellIndex(axis == 0 ? (i + 1) % c.X : i,
                        axis == 1 ? (j + 1) % c.Y : j, axis == 2 ? (k + 1) % c.Z : k);
                    next[axis][cell] = n;
                    previous[axis][n] = cell;
                }
            }
        }

        // Compact periodic MAC fields: component a at index c is the positive
        // a-face of c. The opposite face aliases the preceding cell's entry.
        public int CellIndex(int i, int j, int k)
        {
            if (!Grid.Cells.Interior(i, j, k)) throw new ArgumentOutOfRangeException("Cell is outside the periodic domain.");
            return (k * Grid.Cells.Y + j) * Grid.Cells.X + i;
        }

        public VofAdvance Advance(VofFields state, double[][] transportingVelocity, double dt) =>
            Advance(state, transportingVelocity, dt, null);

        internal VofAdvance Advance(VofFields state, double[][] transportingVelocity, double dt,
            Func<double[], double[][], double[][]> geometricCandidates)
        {
            if (state == null || !SiUnits.IsFinite(dt) || dt <= 0) throw new ArgumentException("A state and positive finite timestep are required.");
            Validate(state, transportingVelocity);
            var total = Axes();
            var liquid = Axes();
            var gas = Axes();
            var mass = Axes();
            var plane = new VofPlane[CellCount];
            var outgoing = new double[CellCount];
            var positive = new double[CellCount];
            var negative = new double[CellCount];
            double volume = Grid.CellVolume;

            for (int cell = 0; cell < CellCount; cell++)
            {
                double alpha = state.LiquidFraction[cell];
                if (geometricCandidates == null && alpha > 0 && alpha < 1)
                    plane[cell] = VofPlane.Reconstruct(new D3(
                        state.LiquidFraction[previous[0][cell]] - state.LiquidFraction[next[0][cell]],
                        state.LiquidFraction[previous[1][cell]] - state.LiquidFraction[next[1][cell]],
                        state.LiquidFraction[previous[2][cell]] - state.LiquidFraction[next[2][cell]]), alpha);
                for (int a = 0; a < 3; a++)
                {
                    double signed = dt * transportingVelocity[a][cell] / Grid.Spacing[a];
                    if (!SiUnits.IsFinite(signed) || Math.Abs(signed) > 1)
                        throw new ArgumentException("A face traverses more than one cell in a step.");
                    total[a][cell] = signed;
                    outgoing[signed < 0 ? next[a][cell] : cell] += Math.Abs(signed);
                }
            }

            for (int cell = 0; cell < CellCount; cell++)
            {
                double divergence = 0;
                for (int a = 0; a < 3; a++) divergence += total[a][cell] - total[a][previous[a][cell]];
                if (Math.Abs(divergence) > Roundoff || outgoing[cell] > .5)
                    throw new ArgumentException("PLIC requires a solenoidal transport field and a combined outgoing Courant number at most 0.5.");
            }

            double[][] candidates = geometricCandidates?.Invoke(state.LiquidFraction, total);
            // The donor-cell baseline is feasible. Limit each geometric
            // increment against the donor's remaining inventory of both phases.
            for (int a = 0; a < 3; a++)
            for (int cell = 0; cell < CellCount; cell++)
            {
                double signed = total[a][cell], swept = Math.Abs(signed);
                int donor = signed < 0 ? next[a][cell] : cell;
                double alpha = state.LiquidFraction[donor];
                double geometric = alpha == 0 || alpha == 1 || swept == 0
                    ? alpha * swept : candidates == null ? plane[donor].Swept(a, signed > 0, swept) : candidates[a][cell];
                if (!SiUnits.IsFinite(geometric) || geometric < -Roundoff || geometric > swept + Roundoff)
                    throw new InvalidOperationException("PLIC candidate is outside the swept slab.");
                double correction = geometric - alpha * swept;
                liquid[a][cell] = correction;
                positive[donor] += Math.Max(0, correction);
                negative[donor] += Math.Max(0, -correction);
            }
            double limited = 0, roundoffVolume = 0;
            for (int a = 0; a < 3; a++)
            for (int cell = 0; cell < CellCount; cell++)
            {
                double signed = total[a][cell], swept = Math.Abs(signed);
                int donor = signed < 0 ? next[a][cell] : cell;
                double alpha = state.LiquidFraction[donor], change = liquid[a][cell];
                double capacity = (1 - outgoing[donor]) * (change >= 0 ? alpha : 1 - alpha);
                double demand = change >= 0 ? positive[donor] : negative[donor];
                double accepted = demand > capacity ? change * (capacity / demand) : change;
                limited += Math.Abs(change - accepted) * volume;
                double l = alpha * swept + accepted;
                if (l < -Roundoff || l > swept + Roundoff)
                    throw new InvalidOperationException("Geometric face volume outside the swept slab.");
                double bounded = Math.Min(swept, Math.Max(0, l));
                roundoffVolume += 2 * Math.Abs(l - bounded) * volume;
                double sign = signed < 0 ? -1 : 1;
                liquid[a][cell] = sign * bounded * volume;
                gas[a][cell] = sign * (swept - bounded) * volume;
                mass[a][cell] = liquidDensity * liquid[a][cell] + gasDensity * gas[a][cell];
            }

            var alphaNext = new double[CellCount];
            var liquidEnergy = (double[])state.LiquidEnergyJ.Clone();
            var gasEnergy = (double[])state.GasEnergyJ.Clone();
            var vapour = (double[])state.VapourKg.Clone();
            var cellMassBefore = new double[CellCount];
            var cellMassAfter = new double[CellCount];
            for (int cell = 0; cell < CellCount; cell++)
            {
                double alpha = state.LiquidFraction[cell], l = alpha * volume, g = (1 - alpha) * volume;
                cellMassBefore[cell] = liquidDensity * l + gasDensity * g;
                for (int a = 0; a < 3; a++)
                {
                    int minus = previous[a][cell], plus = next[a][cell];
                    double lOut = liquid[a][cell], lIn = liquid[a][minus];
                    double gOut = gas[a][cell], gIn = gas[a][minus];
                    l += lIn - lOut;
                    g += gIn - gOut;
                    liquidEnergy[cell] += Transfer(lIn, minus, cell, state.LiquidEnergyJ, state.LiquidFraction, true)
                        - Transfer(lOut, cell, plus, state.LiquidEnergyJ, state.LiquidFraction, true);
                    gasEnergy[cell] += Transfer(gIn, minus, cell, state.GasEnergyJ, state.LiquidFraction, false)
                        - Transfer(gOut, cell, plus, state.GasEnergyJ, state.LiquidFraction, false);
                    vapour[cell] += Transfer(gIn, minus, cell, state.VapourKg, state.LiquidFraction, false)
                        - Transfer(gOut, cell, plus, state.VapourKg, state.LiquidFraction, false);
                }
                if (l < -Roundoff * volume || g < -Roundoff * volume || Math.Abs(l + g - volume) > 4 * Roundoff * volume)
                    throw new InvalidOperationException("Phase volume closure failed.");
                double bounded = Math.Min(volume, Math.Max(0, l));
                // Preserve an empty phase exactly when its transported ledger says it is exhausted.
                if (g <= 0) bounded = volume;
                if (l <= 0) bounded = 0;
                alphaNext[cell] = bounded / volume;
                double storedLiquid = alphaNext[cell] * volume;
                double storedGas = (1 - alphaNext[cell]) * volume;
                roundoffVolume += Math.Abs(storedLiquid - l) + Math.Abs(storedGas - g);
                cellMassAfter[cell] = liquidDensity * storedLiquid + gasDensity * storedGas;
            }

            double roundoffVapour = 0, roundoffEnergy = 0;
            for (int cell = 0; cell < CellCount; cell++)
            {
                double maxVapour = gasDensity * (1 - alphaNext[cell]) * volume;
                if (vapour[cell] < -Roundoff * gasDensity * volume || vapour[cell] > maxVapour + 4 * Roundoff * gasDensity * volume)
                    throw new InvalidOperationException("Vapour exceeds the transported gas inventory.");
                double bounded = Math.Min(maxVapour, Math.Max(0, vapour[cell]));
                roundoffVapour += Math.Abs(bounded - vapour[cell]);
                vapour[cell] = bounded;
                if (alphaNext[cell] == 0) { roundoffEnergy += Math.Abs(liquidEnergy[cell]); liquidEnergy[cell] = 0; }
                if (alphaNext[cell] == 1) { roundoffEnergy += Math.Abs(gasEnergy[cell]); gasEnergy[cell] = 0; }
            }

            var velocity = Axes();
            var momentum = Axes();
            double dualMassError = 0;
            for (int a = 0; a < 3; a++)
            for (int cell = 0; cell < CellCount; cell++)
            {
                int neighbour = next[a][cell];
                double initialMass = .5 * (cellMassBefore[cell] + cellMassBefore[neighbour]);
                double finalMass = .5 * (cellMassAfter[cell] + cellMassAfter[neighbour]);
                double transportedMass = initialMass;
                double p = initialMass * state.FaceVelocity[a][cell];
                for (int direction = 0; direction < 3; direction++)
                {
                    int minus = previous[direction][cell], plus = next[direction][cell];
                    double entering = .5 * (mass[direction][minus] + mass[direction][next[a][minus]]);
                    double leaving = .5 * (mass[direction][cell] + mass[direction][neighbour]);
                    transportedMass += entering - leaving;
                    p += entering * state.FaceVelocity[a][entering >= 0 ? minus : cell]
                        - leaving * state.FaceVelocity[a][leaving >= 0 ? cell : plus];
                }
                dualMassError = Math.Max(dualMassError, Math.Abs(transportedMass - finalMass));
                momentum[a][cell] = p;
                velocity[a][cell] = p / finalMass;
                if (!SiUnits.IsFinite(p) || !SiUnits.IsFinite(velocity[a][cell]))
                    throw new InvalidOperationException("Nonfinite staggered momentum.");
            }
            var fields = new VofFields(alphaNext, velocity, liquidEnergy, gasEnergy, vapour);
            Validate(fields, velocity);
            return new VofAdvance(fields, liquid, gas, mass, momentum, limited, roundoffVolume,
                roundoffVapour, roundoffEnergy, dualMassError);
        }

        private double Transfer(double signedVolume, int low, int high, double[] inventory, double[] alpha, bool isLiquid)
        {
            if (signedVolume == 0) return 0;
            int donor = signedVolume >= 0 ? low : high;
            double fraction = isLiquid ? alpha[donor] : 1 - alpha[donor];
            if (fraction == 0) throw new InvalidOperationException("A flux originates in an empty phase.");
            return inventory[donor] * (signedVolume / (Grid.CellVolume * fraction));
        }

        private double[][] Axes() => new[] { new double[CellCount], new double[CellCount], new double[CellCount] };

        private void Validate(VofFields state, double[][] transport)
        {
            CheckAxes(transport);
            CheckAxes(state.FaceVelocity);
            foreach (var field in new[] { state.LiquidFraction, state.LiquidEnergyJ, state.GasEnergyJ, state.VapourKg })
                CheckField(field);
            for (int c = 0; c < CellCount; c++)
            {
                double alpha = state.LiquidFraction[c];
                if (alpha < 0 || alpha > 1 || (alpha == 0 && state.LiquidEnergyJ[c] != 0) ||
                    (alpha == 1 && state.GasEnergyJ[c] != 0) || state.VapourKg[c] < 0 ||
                    state.VapourKg[c] > gasDensity * (1 - alpha) * Grid.CellVolume)
                    throw new ArgumentException("Invalid phase fraction or phase inventory.");
            }
        }

        private void CheckAxes(double[][] axes)
        {
            if (axes == null || axes.Length != 3) throw new ArgumentException("Three face fields are required.");
            foreach (var field in axes) CheckField(field);
        }

        private void CheckField(double[] values)
        {
            if (values == null || values.Length != CellCount || !Array.TrueForAll(values, SiUnits.IsFinite))
                throw new ArgumentException("A finite field of the compact periodic grid size is required.");
        }
    }

    public sealed class VofFields
    {
        public readonly double[] LiquidFraction, LiquidEnergyJ, GasEnergyJ, VapourKg;
        public readonly double[][] FaceVelocity;
        public VofFields(double[] alpha, double[][] velocity, double[] liquidEnergy, double[] gasEnergy, double[] vapour)
        {
            LiquidFraction = alpha;
            FaceVelocity = velocity;
            LiquidEnergyJ = liquidEnergy;
            GasEnergyJ = gasEnergy;
            VapourKg = vapour;
        }
    }

    public sealed class VofAdvance
    {
        public readonly VofFields Fields;
        public readonly double[][] LiquidTransferM3, GasTransferM3, MassTransferKg, MomentumKgMPerS;
        public readonly double LimitedVolumeM3, RoundoffVolumeM3, RoundoffVapourKg, RoundoffEnergyJ, DualMassErrorKg;
        internal VofAdvance(VofFields fields, double[][] liquid, double[][] gas, double[][] mass, double[][] momentum,
            double limited, double roundoffVolume, double roundoffVapour, double roundoffEnergy, double dualMassError)
        {
            Fields = fields;
            LiquidTransferM3 = liquid;
            GasTransferM3 = gas;
            MassTransferKg = mass;
            MomentumKgMPerS = momentum;
            LimitedVolumeM3 = limited;
            RoundoffVolumeM3 = roundoffVolume;
            RoundoffVapourKg = roundoffVapour;
            RoundoffEnergyJ = roundoffEnergy;
            DualMassErrorKg = dualMassError;
        }
    }
}
