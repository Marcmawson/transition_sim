using System;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Flow
{
    public enum FlowBoundaryKind { NoSlip, FreeSlip, OpenPressure }

    public readonly struct FlowBoundary
    {
        public readonly FlowBoundaryKind Kind;
        public readonly double GaugePressurePa;

        public FlowBoundary(FlowBoundaryKind kind, double gaugePressurePa = 0)
        {
            if (!Enum.IsDefined(typeof(FlowBoundaryKind), kind) || !SiUnits.IsFinite(gaugePressurePa) ||
                (kind != FlowBoundaryKind.OpenPressure && gaugePressurePa != 0))
                throw new ArgumentException("Invalid flow boundary.");
            Kind = kind;
            GaugePressurePa = gaugePressurePa;
        }
    }

    public sealed class BoundaryConditions3D
    {
        private readonly FlowBoundary[] sides;

        // Order is negative and positive for each of x, y and z.
        public BoundaryConditions3D(params FlowBoundary[] sides)
        {
            if (sides == null || sides.Length != 6) throw new ArgumentException("Six domain sides are required.");
            this.sides = (FlowBoundary[])sides.Clone();
        }

        public static BoundaryConditions3D Closed => new BoundaryConditions3D(new FlowBoundary[6]);
        internal FlowBoundary Side(int axis, bool positive) => sides[2 * axis + (positive ? 1 : 0)];
    }
}
