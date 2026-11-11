using System;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Flow
{
    internal static class CutCellScalarGpuLayout
    {
        internal const int GroupPlanes = 9, CandidatePlanes = 6, TransferPlanes = 8, StatePlanes = 5;

        internal static void Pack(CutCellScalarWork work, VofFloatPair[] groups, VofFloatPair[] candidates)
        {
            int ng = work.Plan.GroupCount, nf = work.Plan.FaceCount;
            for (int c = 0; c < ng; c++)
            {
                for (int p = 0; p < StatePlanes; p++) groups[p * ng + c] = new VofFloatPair(work.Fields[p][c]);
                groups[5 * ng + c] = new VofFloatPair(work.Plan.Volumes[c]);
                groups[6 * ng + c] = new VofFloatPair(work.Outgoing[c]);
                groups[7 * ng + c] = new VofFloatPair(work.Increase[c]);
                groups[8 * ng + c] = new VofFloatPair(work.Decrease[c]);
            }
            for (int f = 0; f < nf; f++)
            {
                candidates[f] = new VofFloatPair(work.Swept[f]);
                candidates[nf + f] = new VofFloatPair(work.Correction[f]);
                var source = work.Reservoir[f];
                candidates[2 * nf + f] = new VofFloatPair(source.LiquidFraction);
                candidates[3 * nf + f] = new VofFloatPair(source.LiquidEnergyDensity);
                candidates[4 * nf + f] = new VofFloatPair(source.GasEnergyDensity);
                candidates[5 * nf + f] = new VofFloatPair(source.VapourDensity);
            }
        }

        internal static CutCellScalarOutput Decode(CutCellScalarWork work, VofFloatPair[] transfers, VofFloatPair[] state)
        {
            if (transfers[transfers.Length - 1].Value != VofGpuTransportLayout.Guard ||
                state[state.Length - 1].Value != VofGpuTransportLayout.Guard)
                throw new InvalidOperationException("GPU cut-cell transport overwrote a buffer guard.");
            int ng = work.Plan.GroupCount, nf = work.Plan.FaceCount;
            var fields = new double[StatePlanes][];
            var boundary = new double[StatePlanes];
            var liquid = new double[nf];
            var gas = new double[nf];
            double roundoff = 0, limited = 0;
            for (int p = 0; p < StatePlanes; p++)
            {
                fields[p] = new double[ng];
                for (int c = 0; c < ng; c++) fields[p][c] = Read(state[p * ng + c]);
            }
            for (int f = 0; f < nf; f++)
            {
                if (Read(transfers[7 * nf + f]) != 0)
                    throw new InvalidOperationException($"GPU cut-cell limiter rejected face {f}.");
                liquid[f] = Read(transfers[f]);
                gas[f] = Read(transfers[nf + f]);
                roundoff += Read(transfers[5 * nf + f]);
                limited += Read(transfers[6 * nf + f]);
                var face = work.Plan.Faces[f];
                for (int p = 0; p < StatePlanes; p++)
                {
                    double transfer = Read(transfers[p * nf + f]);
                    if (face.Active == 0 && transfer != 0)
                        throw new InvalidOperationException("GPU transported inventory through an inactive cut face.");
                    if (face.Active == 0) continue;
                    if (face.Low < 0) boundary[p] += transfer;
                    if (face.High < 0) boundary[p] -= transfer;
                }
            }
            return new CutCellScalarOutput(fields, liquid, gas, boundary, roundoff, limited);
        }

        private static double Read(VofFloatPair pair)
        {
            double value = pair.Value;
            if (!SiUnits.IsFinite(value)) throw new InvalidOperationException("Nonfinite GPU cut-cell transport result.");
            return value;
        }
    }
}
