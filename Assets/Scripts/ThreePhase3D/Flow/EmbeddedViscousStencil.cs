using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.PhaseChange;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // Physics-ID: FLOW-EMBEDDED-NO-SLIP
    // Implements: R019
    // Model: Specification.md sections 6.2 and 8; stationary Dirichlet wall velocity.
    // Sources: project finite-volume momentum balance.
    // Source-Locators: S04-Embedded-Wall-Viscosity.md, discrete wall flux.
    // Inputs-SI: dual fluid volumes [m3], apertures and wall areas [m2], centroids [m], nu [m2/s].
    // Outputs-SI: diffusion row rates [1/s].
    // State-Ownership: immutable rates built from the same solid geometry as pressure.
    // Conservation: an active shared aperture carries paired, opposite viscous fluxes.
    // Discretisation: two-point diffusion between fluid centroids and zero wall velocity.
    // Validity: stationary resolved solid geometry, constant material properties.
    // Validation: V08 V35
    // Evidence: PENDING
    // Status: IMPLEMENTED_UNVERIFIED
    // Limitations: two-point centroid approximation on nonorthogonal cut stencils.
    internal sealed class EmbeddedViscousStencil
    {
        internal const int Stride = 7;
        private const double GeometricTolerance = 1e-10;
        private readonly VariableCoefficientProjection plan;
        private readonly int[] neighbours;
        internal readonly float[] Rates;
        internal double MaximumDiagonal { get; private set; }

        internal EmbeddedViscousStencil(VariableCoefficientProjection plan, int[] neighbours, bool embedded,
            Func<InterfacePatch, WettingProfile> wallWetting)
        {
            this.plan = plan;
            this.neighbours = neighbours;
            Rates = new float[checked(plan.FaceCount * Stride)];
            for (int axis = 0; axis < 3; axis++)
            {
                var shape = plan.Grid.Shape((GridLocation)(axis + 1));
                var dual = embedded ? plan.Geometry.BuildVelocityDual(axis) : null;
                var centroid = new D3[shape.Count];
                var wallConductance = new double[shape.Count];
                if (dual != null)
                {
                    for (int local = 0; local < shape.Count; local++)
                        if (plan.Faces[plan.FaceOffsets[axis] + local].Active != 0 && dual.FluidVolumeM3[local] > 0)
                            centroid[local] = dual.FluidCentroidOffset(local);
                    foreach (var patch in dual.Patches)
                    {
                        if (plan.Faces[plan.FaceOffsets[axis] + patch.Cell].Active == 0 || patch.FluidExposedAreaM2 == 0)
                            continue;
                        shape.Coordinates(patch.Cell, out int i, out int j, out int k);
                        D3 centre = dual.Grid.Lower(i, j, k) + plan.Grid.Spacing * .5 + centroid[patch.Cell];
                        double distance = D3.Dot(centre - patch.CentroidLocalM, patch.NormalLocal);
                        if (distance <= GeometricTolerance * MinimumSpacing())
                            throw new NotSupportedException("The dual fluid centroid does not resolve an embedded wall patch. Refine or reposition the grid.");
                        var wetting = patch.Role == SolidRole.Fixed ? wallWetting?.Invoke(patch) : null;
                        if (wetting == null)
                        {
                            wallConductance[patch.Cell] += patch.FluidExposedAreaM2 / distance;
                            continue;
                        }
                        // This scalar MAC row has no cross-component wall terms.
                        // An oblique wall needs those terms for a tangential traction.
                        var normal = patch.NormalLocal;
                        double alignment = Math.Max(Math.Abs(normal.X), Math.Max(Math.Abs(normal.Y), Math.Abs(normal.Z)));
                        if (alignment < 1 - GeometricTolerance ||
                            Math.Abs(normal.X) + Math.Abs(normal.Y) + Math.Abs(normal.Z) - alignment > GeometricTolerance)
                            throw new NotSupportedException("Navier slip requires a grid-aligned embedded wall in the scalar viscous stencil.");
                        double inverseDistance = Math.Abs(normal[axis]) <= GeometricTolerance
                            ? wetting.WallGradientPerVelocity(distance) : 1 / distance;
                        wallConductance[patch.Cell] += patch.FluidExposedAreaM2 * inverseDistance;
                    }
                }

                for (int local = 0; local < shape.Count; local++)
                {
                    int face = plan.FaceOffsets[axis] + local;
                    var metric = plan.Faces[face];
                    if (metric.Active == 0 || metric.Viscosity == 0) continue;
                    double volume = dual == null ? plan.Grid.CellVolume : dual.FluidVolumeM3[local];
                    if (volume <= GeometricTolerance * plan.Grid.CellVolume)
                        throw new NotSupportedException("Active momentum face has an unresolved dual fluid volume.");
                    shape.Coordinates(local, out int i, out int j, out int k);
                    int row = face * Stride;
                    Rates[row + 6] = Rate(metric.Viscosity * wallConductance[local] / volume);
                    double diagonal = Rates[row + 6];
                    for (int direction = 0; direction < 3; direction++)
                    for (int side = 0; side < 2; side++)
                    {
                        int slot = 2 * direction + side;
                        int neighbour = neighbours[face * MomentumPredictor3D.Stride + slot];
                        double h = plan.Grid.Spacing[direction];
                        double rate = metric.Viscosity / (h * h);
                        if (dual != null)
                        {
                            int fi = i + (direction == 0 ? side : 0);
                            int fj = j + (direction == 1 ? side : 0);
                            int fk = k + (direction == 2 ? side : 0);
                            var faceShape = dual.Grid.Shape((GridLocation)(direction + 1));
                            double area = dual.Areas(direction)[faceShape.Index(fi, fj, fk)];
                            rate = 0;
                            if (area <= GeometricTolerance * plan.Grid.FaceArea(direction))
                            {
                                // A blocked dual boundary also cannot import an
                                // upwind sample from across the embedded wall.
                                neighbours[face * MomentumPredictor3D.Stride + slot] = -3;
                            }
                            else
                            {
                                double distance = h;
                                if (neighbour >= 0)
                                {
                                    D3 offset = centroid[neighbour - plan.FaceOffsets[axis]] - centroid[local];
                                    distance += (side == 0 ? -1 : 1) * offset[direction];
                                }
                                else if (neighbour == -2)
                                {
                                    distance = h - (side == 0 ? -2 : 2) * centroid[local][direction];
                                }
                                else if (neighbour == -1 && !IsDomainNormalWall(axis, direction, i, j, k, side))
                                {
                                    // There is no active velocity degree of freedom on
                                    // the other side. The embedded patch supplies drag.
                                    area = 0;
                                }
                                if (distance <= GeometricTolerance * h)
                                    throw new NotSupportedException("The dual-centroid spacing does not resolve viscous flux.");
                                rate = metric.Viscosity * area / (volume * distance);
                            }
                        }
                        Rates[row + slot] = Rate(rate);
                        diagonal += Rates[row + slot] * (neighbour == -2 ? 2 : neighbour == -3 ? 0 : 1);
                    }
                    MaximumDiagonal = Math.Max(MaximumDiagonal, diagonal);
                }
            }
        }

        private bool IsDomainNormalWall(int axis, int direction, int i, int j, int k, int side)
        {
            if (axis != direction) return false;
            int coordinate = axis == 0 ? i : axis == 1 ? j : k;
            var shape = plan.Grid.Shape((GridLocation)(axis + 1));
            int extent = axis == 0 ? shape.X : axis == 1 ? shape.Y : shape.Z;
            return side == 0 ? coordinate == 1 : coordinate == extent - 2;
        }

        private double MinimumSpacing() => Math.Min(plan.Grid.Spacing.X, Math.Min(plan.Grid.Spacing.Y, plan.Grid.Spacing.Z));

        private static float Rate(double value)
        {
            float rate = (float)value;
            if (!SiUnits.IsFinite(rate) || rate < 0)
                throw new InvalidOperationException("Invalid viscous stencil rate.");
            return rate;
        }
    }
}
