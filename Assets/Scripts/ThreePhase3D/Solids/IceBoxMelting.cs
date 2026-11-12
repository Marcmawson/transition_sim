using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.PhaseChange;

namespace PhysicsSim.ThreePhase3D.Solids
{
    // Recede one face of a homogeneous box already at its melting temperature.
    // Meltwater initially inherits the rigid velocity of the removed material.
    public static class IceBoxMelting
    {
        public static (IceBody Remaining, D3 DimensionsM, IceBody Released, double LiquidVolumeM3,
            double LiquidEnthalpyJ, double UnusedHeatJ) Apply(
            IceBody body, D3 dimensionsM, int axis, int side, double heatJ, SolidLiquidInterface equilibrium)
        {
            if (body == null) throw new ArgumentNullException(nameof(body));
            if (equilibrium == null) throw new ArgumentNullException(nameof(equilibrium));
            if (axis < 0 || axis > 2) throw new ArgumentOutOfRangeException(nameof(axis));
            if (side != -1 && side != 1) throw new ArgumentOutOfRangeException(nameof(side));
            if (!SiUnits.IsFinite(heatJ) || heatJ < 0) throw new ArgumentOutOfRangeException(nameof(heatJ));

            double density = body.MassKg / (dimensionsM.X * dimensionsM.Y * dimensionsM.Z);
            var box = IceBody.Box(density, dimensionsM, body.Frame);
            if ((box.PrincipalInertiaKgM2 - body.PrincipalInertiaKgM2).Length > 1e-12 * box.PrincipalInertiaKgM2.Length)
                throw new ArgumentException("Ice inertia must match the homogeneous box.", nameof(body));

            double totalLatentHeat = body.MassKg * equilibrium.LatentHeatJkg;
            if (heatJ >= totalLatentHeat)
                return (null, default, body, body.MassKg / equilibrium.LiquidMeltingDensityKgM3,
                    body.MassKg * equilibrium.LiquidMeltingEnthalpyJkg, heatJ - totalLatentHeat);

            double length = dimensionsM[axis];
            double recession = length * (heatJ / totalLatentHeat);
            double remainingLength = length - recession;
            if (remainingLength == length) return (body, dimensionsM, null, 0, 0, heatJ);

            var localAxis = axis == 0 ? new D3(1, 0, 0) : axis == 1 ? new D3(0, 1, 0) : new D3(0, 0, 1);
            D3 remainingSize = Resize(remainingLength);
            IceBody remaining = Piece(remainingSize, -side * recession / 2);
            IceBody released = Piece(Resize(recession), side * remainingLength / 2);
            return (remaining, remainingSize, released, released.MassKg / equilibrium.LiquidMeltingDensityKgM3,
                released.MassKg * equilibrium.LiquidMeltingEnthalpyJkg, 0);

            D3 Resize(double extent) => axis == 0 ? new D3(extent, dimensionsM.Y, dimensionsM.Z) :
                axis == 1 ? new D3(dimensionsM.X, extent, dimensionsM.Z) :
                new D3(dimensionsM.X, dimensionsM.Y, extent);

            IceBody Piece(D3 size, double offset)
            {
                var frame = body.Frame;
                var pieceFrame = new RigidFrame(frame.ToWorld(localAxis * offset), frame.U, frame.V, frame.W);
                var shape = IceBody.Box(density, size, pieceFrame);
                D3 omega = body.AngularVelocityRadPerS;
                D3 spin = frame.U * (D3.Dot(omega, frame.U) * shape.PrincipalInertiaKgM2.X) +
                    frame.V * (D3.Dot(omega, frame.V) * shape.PrincipalInertiaKgM2.Y) +
                    frame.W * (D3.Dot(omega, frame.W) * shape.PrincipalInertiaKgM2.Z);
                return new IceBody(shape.MassKg, shape.PrincipalInertiaKgM2, pieceFrame,
                    body.VelocityAt(pieceFrame.Origin) * shape.MassKg, spin);
            }
        }
    }
}
