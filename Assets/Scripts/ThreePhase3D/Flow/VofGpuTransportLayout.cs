using System;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Flow
{
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

        internal void Pack(VofFields state, double[][] total, double[] outgoing, double[] destination)
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

            void Write(double[] values, int plane) => Array.Copy(values, 0, destination, plane * count, count);
        }

        internal VofAdvance Read(double[] faces, double[] cells, double[] momentum, out double candidateRoundoff)
        {
            Check(faces, FacePlanes);
            Check(cells, CellPlanes);
            Check(momentum, MomentumPlanes);
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
            var fields = new VofFields(Plane(cells, 0), Axes(momentum, 0),
                Plane(cells, 1), Plane(cells, 2), Plane(cells, 3));
            vof.Validate(fields, fields.FaceVelocity);
            return new VofAdvance(fields, Axes(faces, 0), Axes(faces, 3), Axes(faces, 6), Axes(momentum, 3),
                limited, volumeRoundoff, vapourRoundoff, energyRoundoff, dual);
        }

        private void Check(double[] values, int planes)
        {
            if (values.Length != planes * count + 1 || values[values.Length - 1] != Guard)
                throw new InvalidOperationException("GPU VOF output buffer guard was overwritten.");
            if (!Array.TrueForAll(values, SiUnits.IsFinite))
                throw new InvalidOperationException("GPU VOF output contains a nonfinite value.");
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
