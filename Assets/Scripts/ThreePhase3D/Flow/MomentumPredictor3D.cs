using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.PhaseChange;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // Physics-ID: FLOW-MAC-MOMENTUM-PREDICTOR
    // Implements: R019
    // Model: Specification.md section 6.2 in the uniform-viscosity, constant-density limit.
    // Sources: project Specification.md and Validation.md.
    // Source-Locators: section 6.2 momentum balance and V08 analytical channel cases.
    // Inputs-SI: staggered velocity [m/s], kinematic viscosity [m2/s], acceleration [m/s2].
    // Outputs-SI: tentative staggered velocity [m/s].
    // State-Ownership: immutable stencils; a trial reads the accepted velocity field.
    // Conservation: blocked faces carry zero volume flux; advective momentum form is used.
    // Discretisation: forward Euler, first-order upwind advection, centred diffusion.
    // Validity: uniform-property flow with stationary walls.
    // Validation: V07 V08 V35
    // Evidence: ValidationRuns/s04-flow-20261111/cpu-results.txt
    // Status: IMPLEMENTED_UNVERIFIED
    // Limitations: variable-viscosity stress and conservative phase transport are open.
    public sealed class MomentumPredictor3D
    {
        internal const int Stride = 18;
        internal readonly int[] Stencil;
        internal readonly VariableCoefficientProjection Plan;
        internal readonly EmbeddedViscousStencil Diffusion;

        public MomentumPredictor3D(VariableCoefficientProjection plan) : this(plan, null) { }

        // Select slip patches by their geometry in the staggered dual grid.
        // Rebuild the predictor when the selected contact-line region changes.
        public MomentumPredictor3D(VariableCoefficientProjection plan, Func<InterfacePatch, WettingProfile> wallWetting)
        {
            Plan = plan ?? throw new ArgumentNullException(nameof(plan));
            double maximumViscosity = 0;
            Stencil = new int[checked(plan.FaceCount * Stride)];
            for (int axis = 0; axis < 3; axis++)
            {
                var shape = plan.Grid.Shape((GridLocation)(axis + 1));
                for (int index = 0; index < shape.Count; index++)
                {
                    int face = plan.FaceOffsets[axis] + index;
                    if (plan.Faces[face].Active == 0) continue;
                    maximumViscosity = Math.Max(maximumViscosity, plan.Faces[face].Viscosity);
                    shape.Coordinates(index, out int i, out int j, out int k);
                    for (int direction = 0; direction < 3; direction++)
                    {
                        int offset = face * Stride + 2 * direction;
                        Stencil[offset] = Neighbour(axis, direction, i, j, k, -1);
                        Stencil[offset + 1] = Neighbour(axis, direction, i, j, k, 1);
                        for (int sample = 0; sample < 4; sample++)
                        {
                            int normal = -(sample & 1), transverse = sample >> 1;
                            Stencil[face * Stride + 6 + 4 * direction + sample] = axis == direction
                                ? face
                                : AdvectingFace(direction,
                                    i + (axis == 0 ? normal : 0) + (direction == 0 ? transverse : 0),
                                    j + (axis == 1 ? normal : 0) + (direction == 1 ? transverse : 0),
                                    k + (axis == 2 ? normal : 0) + (direction == 2 ? transverse : 0));
                        }
                    }
                }
            }
            if (plan.HasVariableProperties && maximumViscosity > 0)
                throw new NotSupportedException("The viscous predictor requires uniform density and dynamic viscosity.");
            Diffusion = new EmbeddedViscousStencil(plan, Stencil, plan.HasEmbeddedSolid && maximumViscosity > 0, wallWetting);
        }

        private int Neighbour(int axis, int direction, int i, int j, int k, int sign)
        {
            var shape = Plan.Grid.Shape((GridLocation)(axis + 1));
            int ni = i + (direction == 0 ? sign : 0);
            int nj = j + (direction == 1 ? sign : 0);
            int nk = k + (direction == 2 ? sign : 0);
            if (shape.Interior(ni, nj, nk))
            {
                int face = Plan.FaceIndex(axis, ni, nj, nk);
                return Plan.Faces[face].Active == 0 ? -1 : face;
            }

            var boundary = Plan.Boundaries.Side(direction, sign > 0);
            return direction != axis && boundary.Kind == FlowBoundaryKind.NoSlip ? -2 : -3;
        }

        private int AdvectingFace(int axis, int i, int j, int k)
        {
            var shape = Plan.Grid.Shape((GridLocation)(axis + 1));
            i = Math.Max(0, Math.Min(shape.X - 1, i));
            j = Math.Max(0, Math.Min(shape.Y - 1, j));
            k = Math.Max(0, Math.Min(shape.Z - 1, k));
            int face = Plan.FaceIndex(axis, i, j, k);
            return Plan.Faces[face].Active == 0 ? -1 : face;
        }

        internal D3 LocalAcceleration(D3 world) =>
            new D3(D3.Dot(world, Plan.Grid.Frame.U), D3.Dot(world, Plan.Grid.Frame.V), D3.Dot(world, Plan.Grid.Frame.W));

        public double SelectTimestep(float[] velocity, D3 worldAcceleration, double requestedSeconds)
        {
            if (velocity == null || velocity.Length != Plan.FaceCount ||
                !SiUnits.IsFinite(requestedSeconds) || requestedSeconds <= 0)
                throw new ArgumentException("A full finite velocity field and positive requested timestep are required.");
            var speed = new double[3];
            for (int face = 0; face < velocity.Length; face++)
            {
                if (!SiUnits.IsFinite(velocity[face]) || (Plan.Faces[face].Active == 0 && velocity[face] != 0))
                    throw new ArgumentException("Nonfinite velocity or nonzero blocked/ghost face.");
                int axis = Plan.Faces[face].Axis;
                speed[axis] = Math.Max(speed[axis], Math.Abs(velocity[face]));
            }
            D3 h = Plan.Grid.Spacing, a = LocalAcceleration(worldAcceleration);
            double advective = 0, acceleration = 0;
            for (int axis = 0; axis < 3; axis++)
            {
                advective += speed[axis] / h[axis];
                acceleration += Math.Abs(a[axis]) / h[axis];
            }
            // Bound the sum of the explicit upwind and diffusion weights.
            double rate = advective + Diffusion.MaximumDiagonal;
            double stable = 0.9 / (rate + Math.Sqrt(rate * rate + 1.8 * acceleration));
            double dt = Math.Min(requestedSeconds, stable);
            float gpuDt = (float)dt;
            if (!(gpuDt > 0) || float.IsInfinity(gpuDt)) throw new ArgumentException("Timestep is outside positive FP32 range.");
            // Round down if the FP32 conversion rounded above the requested bound.
            if (gpuDt > dt) gpuDt = (float)(dt * (1 - 1e-7));
            if (!(gpuDt > 0) || gpuDt > dt) throw new ArgumentException("No positive FP32 timestep fits the bound.");
            return gpuDt;
        }

        internal float[] Predict(float[] velocity, D3 worldAcceleration, double dt)
        {
            var output = new float[velocity.Length];
            D3 a = LocalAcceleration(worldAcceleration);
            D3 h = Plan.Grid.Spacing;
            for (int face = 0; face < velocity.Length; face++)
            {
                var data = Plan.Faces[face];
                if (data.Active == 0) continue;
                double centre = velocity[face], change = a[data.Axis];
                int start = face * Stride, row = face * EmbeddedViscousStencil.Stride;
                change -= Diffusion.Rates[row + 6] * centre;
                for (int axis = 0; axis < 3; axis++)
                {
                    double lower = Sample(velocity, Stencil[start + 2 * axis], centre);
                    double upper = Sample(velocity, Stencil[start + 2 * axis + 1], centre);
                    double transport = 0;
                    for (int sample = 0; sample < 4; sample++)
                        transport += Sample(velocity, Stencil[start + 6 + 4 * axis + sample], centre) * .25;
                    if (axis != data.Axis)
                        change -= Diffusion.Rates[row + 7 + (axis + 3 - data.Axis) % 3 - 1] * transport;
                    double derivative = transport >= 0 ? centre - lower : upper - centre;
                    change += Diffusion.Rates[row + 2 * axis] * (lower - centre)
                        + Diffusion.Rates[row + 2 * axis + 1] * (upper - centre)
                        - transport * derivative / h[axis];
                }
                output[face] = (float)(centre + dt * change);
            }
            return output;
        }

        private static double Sample(float[] velocity, int index, double centre) =>
            index >= 0 ? velocity[index] : index == -2 ? -centre : index == -3 ? centre : 0;
    }
}
