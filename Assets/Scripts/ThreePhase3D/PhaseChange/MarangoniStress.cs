using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;

namespace PhysicsSim.ThreePhase3D.PhaseChange
{
    internal static class MarangoniStress
    {
        // Cell-centred continuum force density [N/m³]. The VOF gradient
        // supplies both the surface delta approximation and interface normal.
        internal static D3[] ForceDensity(GeometricVof3D transport, double[] fraction, double[] sigma)
        {
            var grid = transport.Grid;
            var shape = grid.Cells;
            var result = new D3[transport.CellCount];
            for (int k = 0; k < shape.Z; k++)
            for (int j = 0; j < shape.Y; j++)
            for (int i = 0; i < shape.X; i++)
            {
                int west = transport.CellIndex((i + shape.X - 1) % shape.X, j, k);
                int east = transport.CellIndex((i + 1) % shape.X, j, k);
                int south = transport.CellIndex(i, (j + shape.Y - 1) % shape.Y, k);
                int north = transport.CellIndex(i, (j + 1) % shape.Y, k);
                int back = transport.CellIndex(i, j, (k + shape.Z - 1) % shape.Z);
                int front = transport.CellIndex(i, j, (k + 1) % shape.Z);

                D3 Gradient(double[] values) => new D3(
                    (values[east] - values[west]) / (2 * grid.Spacing.X),
                    (values[north] - values[south]) / (2 * grid.Spacing.Y),
                    (values[front] - values[back]) / (2 * grid.Spacing.Z));

                var fractionGradient = Gradient(fraction);
                double magnitude = fractionGradient.Length;
                if (magnitude == 0) continue;
                var normal = fractionGradient / magnitude;
                var sigmaGradient = Gradient(sigma);
                var tangential = sigmaGradient - normal * D3.Dot(normal, sigmaGradient);
                result[transport.CellIndex(i, j, k)] = tangential * magnitude;
            }
            return result;
        }
    }
}
