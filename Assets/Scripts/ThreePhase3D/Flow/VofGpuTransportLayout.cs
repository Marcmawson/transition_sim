using System;
using System.Runtime.InteropServices;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Flow
{
    [StructLayout(LayoutKind.Sequential)]
    internal readonly struct VofFloatPair
    {
        internal readonly float High, Low;
        internal VofFloatPair(double value)
        {
            High = (float)value;
            Low = (float)(value - High);
            if (!SiUnits.IsFinite(High) || !SiUnits.IsFinite(Low))
                throw new ArgumentException("Value exceeds the GPU VOF float-pair range.");
        }
        internal double Value => (double)High + Low;
    }

    internal sealed class VofGpuTransportLayout
    {
        internal const int InputPlanes = 11, FacePlanes = 21, CellPlanes = 10, MomentumPlanes = 9;
        internal const double Guard = -123456.75;
        private readonly GeometricVof3D vof;
        private readonly int count;

        internal VofGpuTransportLayout(GeometricVof3D vof)
        {
            this.vof = vof;
            count = vof.CellCount;
        }

        internal void Pack(VofFields state, double[][] total, double[] outgoing, VofFloatPair[] destination)
        {
            Write(state.LiquidFraction, 0);
            Write(state.LiquidEnergyJ, 1);
            Write(state.GasEnergyJ, 2);
            Write(state.VapourKg, 3);
            for (int axis = 0; axis < 3; axis++)
            {
                Write(state.FaceVelocity[axis], 4 + axis);
                Write(total[axis], 7 + axis);
            }
            Write(outgoing, 10);

            void Write(double[] values, int plane)
            {
                for (int cell = 0; cell < count; cell++)
                    destination[plane * count + cell] = new VofFloatPair(values[cell]);
            }
        }

        internal VofAdvance Read(VofFloatPair[] facePairs, VofFloatPair[] cellPairs, VofFloatPair[] momentumPairs, out double candidateRoundoff)
        {
            var faces = Decode(facePairs, FacePlanes);
            var cells = Decode(cellPairs, CellPlanes);
            var momentum = Decode(momentumPairs, MomentumPlanes);
            double limited = 0, volumeRoundoff = 0, vapourRoundoff = 0, energyRoundoff = 0, dual = 0;
            candidateRoundoff = 0;
            for (int c = 0; c < count; c++)
            {
                if (cells[9 * count + c] != 0)
                    throw new InvalidOperationException($"GPU VOF phase volume or vapour closure failed in cell {c}.");
                volumeRoundoff += cells[6 * count + c];
                vapourRoundoff += cells[7 * count + c];
                energyRoundoff += cells[8 * count + c];
                for (int axis = 0; axis < 3; axis++)
                {
                    if (faces[(15 + axis) * count + c] != 0)
                        throw new InvalidOperationException($"GPU VOF geometric transfer failed at cell {c}, axis {axis}.");
                    limited += faces[(9 + axis) * count + c];
                    volumeRoundoff += faces[(12 + axis) * count + c];
                    candidateRoundoff += faces[(18 + axis) * count + c];
                    dual = Math.Max(dual, momentum[(6 + axis) * count + c]);
                }
            }
            var alpha = Plane(cells, 0);
            var liquidEnergy = Plane(cells, 1);
            var gasEnergy = Plane(cells, 2);
            var vapour = Plane(cells, 3);
            for (int c = 0; c < count; c++)
            {
                // Decoding the paired representation to one host double can
                // round a near-pure cell to an exact phase endpoint.
                double maxVapour = vof.GasDensity * (1 - alpha[c]) * vof.Grid.CellVolume;
                double excess = vapour[c] - maxVapour;
                if (excess > 2.5e-13 * vof.GasDensity * vof.Grid.CellVolume)
                    throw new InvalidOperationException($"Decoded GPU vapour exceeds the gas inventory in cell {c}.");
                if (excess > 0) { vapourRoundoff += excess; vapour[c] = maxVapour; }
                if (alpha[c] == 0) { energyRoundoff += Math.Abs(liquidEnergy[c]); liquidEnergy[c] = 0; }
                if (alpha[c] == 1) { energyRoundoff += Math.Abs(gasEnergy[c]); gasEnergy[c] = 0; }
            }
            var fields = new VofFields(alpha, Axes(momentum, 0), liquidEnergy, gasEnergy, vapour);
            vof.Validate(fields, fields.FaceVelocity);
            return new VofAdvance(fields, Axes(faces, 0), Axes(faces, 3), Axes(faces, 6), Axes(momentum, 3),
                limited, volumeRoundoff, vapourRoundoff, energyRoundoff, dual);
        }

        private double[] Decode(VofFloatPair[] pairs, int planes)
        {
            if (pairs.Length != planes * count + 1 || pairs[pairs.Length - 1].Value != Guard)
                throw new InvalidOperationException("GPU VOF output buffer guard was overwritten.");
            var values = new double[pairs.Length - 1];
            for (int i = 0; i < values.Length; i++)
            {
                values[i] = pairs[i].Value;
                if (!SiUnits.IsFinite(values[i]))
                    throw new InvalidOperationException("GPU VOF output contains a nonfinite value.");
            }
            return values;
        }

        private double[] Plane(double[] values, int plane)
        {
            var result = new double[count];
            Array.Copy(values, plane * count, result, 0, count);
            return result;
        }

        private double[][] Axes(double[] values, int plane) =>
            new[] { Plane(values, plane), Plane(values, plane + 1), Plane(values, plane + 2) };
    }
}
