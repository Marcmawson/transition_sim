using System;
using System.Collections.Generic;
using System.Runtime.Serialization;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Geometry;

namespace PhysicsSim.ThreePhase3D.Verification
{
    [DataContract]
    public sealed class FlowCheckRow
    {
        [DataMember] public string Case, Quantity;
        [DataMember] public double Actual, Expected, Tolerance;
        [DataMember] public bool Passed;
    }

    public sealed class FlowChecks
    {
        public delegate FlowStep Advance(MomentumPredictor3D predictor, float[] velocity,
            D3 acceleration, double dt, int maxIterations);

        private readonly Advance advance;
        public readonly List<FlowCheckRow> Rows = new List<FlowCheckRow>();
        public bool Passed => Rows.Count > 0 && Rows.TrueForAll(row => row.Passed);

        public FlowChecks(Advance advance) { this.advance = advance; }

        public void Run()
        {
            var orientation = RigidFrame.Euler(new D3(0.21, -0.12, 0.34), 0.16, 0.11, -0.09);
            Hydrostatic("hydrostatic", new Grid3D(4, 5, 3, new D3(.1, .08, .12)), null, false);
            Hydrostatic("hydrostatic-translated-rotated", new Grid3D(4, 5, 3, new D3(.1, .08, .12), orientation), null, false);
            var cutGrid = new Grid3D(5, 4, 3, new D3(.1, .1, .1));
            var slab = SurfacePrimitives.Box(new D3(.2, -.05, -.05), new D3(.297, .45, .35));
            Hydrostatic("hydrostatic-disconnected-cut", cutGrid, slab, true);
            Hydrostatic("hydrostatic-variable-density", cutGrid, null, false, j => j < 2 ? 700 : 1300);
            Channel();
            EmbeddedShear();
            ViscousBeaker();
            CurvedWallChannel();
            Decay();
            Rollback();
        }

        private void Hydrostatic(string id, Grid3D grid, TriangleSurface solid, bool expectMerge, Func<int, double> density = null)
        {
            var geometry = CutCellGeometry.Build(grid, solid);
            var merge = new TinyCellMergePlan(geometry);
            Check(id, "tiny cells merged", merge.MergedCellCount > 0 ? 1 : 0, expectMerge ? 1 : 0, 0);
            density = density ?? (_ => 997);
            var projection = new VariableCoefficientProjection(geometry, BoundaryConditions3D.Closed,
                cell => { grid.Cells.Coordinates(cell, out _, out int j, out _); return density(j); }, _ => 0);
            var predictor = new MomentumPredictor3D(projection);
            var gravity = new D3(0, -9.80665, 0);
            var step = RunStep(id, predictor, new float[projection.FaceCount], gravity, .005);
            double maxSpeed = 0, maxHeadError = 0, largestClosedFaceSpeed = 0;
            foreach (float speed in step.VelocityMPerS) maxSpeed = Math.Max(maxSpeed, Math.Abs(speed));
            for (int axis = 0; axis < 3; axis++)
            {
                var shape = grid.Shape((GridLocation)(axis + 1));
                for (int index = 0; index < shape.Count; index++)
                {
                    int face = projection.FaceOffsets[axis] + index;
                    var metric = projection.Faces[face];
                    if (metric.Active == 0)
                    {
                        largestClosedFaceSpeed = Math.Max(largestClosedFaceSpeed, Math.Abs(step.VelocityMPerS[face]));
                        continue;
                    }
                    shape.Coordinates(index, out _, out int j, out _);
                    double faceDensity = axis == 1 ? .5 * (density(j - 1) + density(j)) : density(j);
                    double expected = faceDensity * predictor.LocalAcceleration(gravity)[axis] * grid.Spacing[axis];
                    double actual = step.GroupPressurePa[metric.High] - step.GroupPressurePa[metric.Low];
                    maxHeadError = Math.Max(maxHeadError, Math.Abs(actual - expected));
                }
            }
            Check(id, "max spurious velocity [m/s]", maxSpeed, 0, 2e-6);
            Check(id, "max closed face velocity [m/s]", largestClosedFaceSpeed, 0, 0);
            Check(id, "max hydrostatic neighbour head error [Pa]", maxHeadError, 0, .04);
            if (expectMerge)
            {
                Check(id, "total aperture closed within geometry roundoff [m2]", projection.RoundoffClosedAreaM2, 0, 1e-15);
                double[] moments = new double[2];
                double[] volume = new double[2];
                for (int k = 0; k < grid.Cells.Z; k++)
                for (int j = 0; j < grid.Cells.Y; j++)
                for (int i = 0; i < grid.Cells.X; i++)
                {
                    int group = projection.PressureGroup(i, j, k);
                    if (group < 0) continue;
                    int chamber = i < 2 ? 0 : 1;
                    double v = geometry.FluidVolumeM3[grid.Cells.Index(i, j, k)];
                    moments[chamber] += v * step.GroupPressurePa[group];
                    volume[chamber] += v;
                }
                for (int chamber = 0; chamber < 2; chamber++)
                    Check(id, "closed chamber mean gauge [Pa] " + chamber, moments[chamber] / volume[chamber], 0, 1e-6);
            }
        }

        private void EmbeddedShear()
        {
            const string id = "embedded-no-slip";
            var open = new FlowBoundary(FlowBoundaryKind.OpenPressure);
            var slip = new FlowBoundary(FlowBoundaryKind.FreeSlip);
            var boundary = new BoundaryConditions3D(open, open, slip, slip, slip, slip);
            var grid = new Grid3D(4, 8, 3, new D3(.005, .005, .005));
            const double wallY = .0073, shear = 3;
            var slab = SurfacePrimitives.Box(new D3(-.01, -.01, -.01), new D3(.03, wallY, .025));
            var geometry = CutCellGeometry.Build(grid, slab);
            var plan = new VariableCoefficientProjection(geometry, boundary, _ => 1000, _ => .001);
            var predictor = new MomentumPredictor3D(plan);
            var u = XProfile(plan, j =>
            {
                double low = Math.Max(wallY, j * grid.Spacing.Y), high = (j + 1) * grid.Spacing.Y;
                return low < high ? shear * (.5 * (low + high) - wallY) : 0;
            });
            int probe = plan.FaceIndex(0, 2, 1, 1);
            var step = RunStep(id, predictor, u, new D3(), .01);
            Check(id, "linear shear at an off-grid wall [m/s]", step.VelocityMPerS[probe], u[probe], 2e-8);
            double cutHeight = 2 * grid.Spacing.Y - wallY;
            double expectedRate = 1e-6 / (cutHeight * .5 * cutHeight);
            Check(id, "wall rate from the analytical dual volume and centroid [1/s]",
                predictor.Diffusion.Rates[probe * EmbeddedViscousStencil.Stride + 6], expectedRate, 2e-6);
            var plug = XProfile(plan, j => j == 0 ? 0 : .05);
            var damped = RunStep(id + "-decay", predictor, plug, new D3(), .01);
            Check(id, "wall damps uniform tangential speed by the expected amount [m/s]",
                damped.VelocityMPerS[probe], plug[probe] * (1 - damped.Diagnostics.Dt * expectedRate), 2e-8);
            Check(id, "wall damping reduces the velocity norm", Energy(damped.VelocityMPerS) < Energy(plug) ? 1 : 0, 1, 0);
        }

        private void ViscousBeaker()
        {
            const string id = "viscous-beaker-rest";
            var frame = new RigidFrame(new D3(-.08,-.025,-.08), new D3(1,0,0), new D3(0,1,0), new D3(0,0,1));
            var grid = new Grid3D(16, 13, 16, new D3(.01,.01,.01), frame);
            var surface = ApparatusSurface.Beaker(new GeometryInputs
            {
                innerRadiusM = .033, wallThicknessM = .031, baseThicknessM = .031, cavityHeightM = .06
            }, 24);
            var geometry = CutCellGeometry.Build(grid, surface);
            var plan = new VariableCoefficientProjection(geometry, BoundaryConditions3D.Closed, _ => 997, _ => .001);
            var predictor = new MomentumPredictor3D(plan);
            Check(id, "embedded wall rates are present", predictor.Diffusion.MaximumDiagonal > 0 ? 1 : 0, 1, 0);
            var rest = RunStep(id, predictor, new float[plan.FaceCount], new D3(), .005);
            Check(id, "stationary unforced viscous beaker stays at rest [m/s]", Energy(rest.VelocityMPerS), 0, 0);
        }

        private void CurvedWallChannel()
        {
            const string id = "embedded-curved-channel";
            var open = new FlowBoundary(FlowBoundaryKind.OpenPressure);
            var wall = new FlowBoundary(FlowBoundaryKind.NoSlip);
            var boundaries = new BoundaryConditions3D(wall, wall, open, open, wall, wall);
            var frame = new RigidFrame(new D3(-.08, 0, -.08), new D3(1,0,0), new D3(0,1,0), new D3(0,0,1));
            var grid = new Grid3D(16, 3, 16, new D3(.01,.01,.01), frame);
            // The cylindrical wall spans the test domain; its base and rim lie
            // beyond the axial ends. This isolates curved-wall diffusion.
            var solid = ApparatusSurface.Beaker(new GeometryInputs
            {
                innerRadiusM = .034, wallThicknessM = .085, baseThicknessM = .031,
                cavityHeightM = .15, externalBaseHeightM = -.1
            }, 48);
            var geometry = CutCellGeometry.Build(grid, solid);
            var plan = new VariableCoefficientProjection(geometry, boundaries, _ => 1000, _ => .001);
            var predictor = new MomentumPredictor3D(plan);
            var initial = new float[plan.FaceCount];
            for (int k = 0; k < grid.Cells.Z; k++)
            for (int j = 0; j <= grid.Cells.Y; j++)
            for (int i = 0; i < grid.Cells.X; i++)
            {
                int face = plan.FaceIndex(1, i, j, k);
                if (plan.Faces[face].Active != 0) initial[face] = .04f;
            }
            var step = RunStep(id, predictor, initial, new D3(), .01);
            int core = plan.FaceIndex(1, 7, 1, 7);
            Check(id, "core plug speed before wall diffusion reaches it [m/s]", step.VelocityMPerS[core], initial[core], 2e-8);
            Check(id, "curved wall does not reverse the plug flow", Array.Exists(step.VelocityMPerS, speed => speed < -1e-9f) ? 1 : 0, 0, 0);
            Check(id, "curved wall reduces axial plug velocity norm", Energy(step.VelocityMPerS) < Energy(initial) ? 1 : 0, 1, 0);
        }

        private void Channel()
        {
            const string id = "plane-poiseuille";
            var inlet = new FlowBoundary(FlowBoundaryKind.OpenPressure, 0.06);
            var outlet = new FlowBoundary(FlowBoundaryKind.OpenPressure, 0);
            var wall = new FlowBoundary(FlowBoundaryKind.NoSlip);
            var slip = new FlowBoundary(FlowBoundaryKind.FreeSlip);
            const double rho = 1000, mu = .001, h = .005;
            double[] errors = new double[3];
            for (int level = 0; level < errors.Length; level++)
            {
                int n = 4 << level;
                var grid = new Grid3D(4, n, 3, new D3(h, 4 * h / n, h));
                var boundary = new BoundaryConditions3D(inlet, outlet, wall, wall, slip, slip);
                var projection = new VariableCoefficientProjection(CutCellGeometry.Build(grid), boundary, _ => rho, _ => mu);
                double height = n * grid.Spacing.Y;
                double factor = inlet.GaugePressurePa / (grid.Cells.X * h * 2 * mu);
                Func<int, double> analytic = j =>
                {
                    double y = (j + .5) * grid.Spacing.Y;
                    return factor * y * (height - y);
                };
                double correction = factor * grid.Spacing.Y * grid.Spacing.Y / 4;
                var input = XProfile(projection, j => analytic(j) + correction);
                var predictor = new MomentumPredictor3D(projection);
                if (n == 4)
                {
                    var started = RunStep(id + "-from-rest", predictor, new float[projection.FaceCount], new D3(), .05);
                    double expectedSpeed = inlet.GaugePressurePa * started.Diagnostics.Dt / (rho * grid.Cells.X * h);
                    Check(id, "pressure accelerates fluid from rest [m/s]",
                        started.VelocityMPerS[projection.FaceIndex(0, 2, 2, 1)], expectedSpeed, 1e-8);
                }
                var step = RunStep(id + "-" + n, predictor, input, new D3(), .05);
                double analyticError = 0, analyticNorm = 0, steadyError = 0, pressureError = 0;
                for (int k = 0; k < grid.Cells.Z; k++)
                for (int j = 0; j < n; j++)
                for (int i = 0; i <= grid.Cells.X; i++)
                {
                    int face = projection.FaceIndex(0, i, j, k);
                    double difference = step.VelocityMPerS[face] - analytic(j);
                    analyticError += difference * difference;
                    analyticNorm += analytic(j) * analytic(j);
                    steadyError = Math.Max(steadyError, Math.Abs(step.VelocityMPerS[face] - input[face]));
                    if (i == grid.Cells.X) continue;
                    double expected = inlet.GaugePressurePa * (1 - (i + .5) / grid.Cells.X);
                    pressureError = Math.Max(pressureError, Math.Abs(step.GroupPressurePa[projection.PressureGroup(i, j, k)] - expected));
                }
                errors[level] = Math.Sqrt(analyticError / analyticNorm);
                Check(id + "-" + n, "max change in discrete steady profile [m/s]", steadyError, 0, 2e-7);
                Check(id + "-" + n, "max pressure error [Pa]", pressureError, 0, 2e-6);
                if (n == 16) Check(id + "-" + n, "accepted-grid relative L2 profile error", errors[level], 0, .01);
            }
            for (int level = 1; level < errors.Length; level++)
                Check(id, "profile error ratio under spatial refinement " + level, errors[level] / errors[level - 1], .25, .02);
        }

        private void Decay()
        {
            const string id = "viscous-decay";
            var wall = new FlowBoundary(FlowBoundaryKind.NoSlip);
            var slip = new FlowBoundary(FlowBoundaryKind.FreeSlip);
            var open = new FlowBoundary(FlowBoundaryKind.OpenPressure);
            const int ny = 8;
            const double dy = .005, rho = 1000, mu = .001;
            var grid = new Grid3D(3, ny, 3, new D3(.02, dy, .02));
            var boundary = new BoundaryConditions3D(open, open, wall, wall, slip, slip);
            var projection = new VariableCoefficientProjection(CutCellGeometry.Build(grid), boundary, _ => rho, _ => mu);
            var predictor = new MomentumPredictor3D(projection);
            var initial = XProfile(projection, j => .001 * Math.Sin(Math.PI * (j + .5) / ny));
            int probe = projection.FaceIndex(0, 1, ny / 2, 1);
            double lambda = 4 * (mu / rho) * Math.Pow(Math.Sin(Math.PI / (2 * ny)), 2) / (dy * dy);
            double[] errors = new double[2];
            for (int level = 0; level < errors.Length; level++)
            {
                int count = 4 << level;
                double elapsed = 0, previousEnergy = Energy(initial);
                float[] current = initial;
                for (int i = 0; i < count; i++)
                {
                    var step = RunStep(id + "-" + count + "-" + i, predictor, current, new D3(), 4.0 / count);
                    current = step.VelocityMPerS;
                    elapsed += step.Diagnostics.Dt;
                    double energy = Energy(current);
                    Check(id + "-" + count + "-" + i, "kinetic energy decreases", energy < previousEnergy ? 1 : 0, 1, 0);
                    previousEnergy = energy;
                }
                double amplitude = current[probe] / initial[probe];
                double expected = Math.Exp(-lambda * elapsed);
                errors[level] = Math.Abs(amplitude - expected);
                Check(id + "-" + count, "decay amplitude relative to initial", amplitude, expected, .001);
            }
            Check(id, "time refinement error ratio", errors[1] / errors[0], .5, .04);
        }

        private void Rollback()
        {
            const string id = "rollback";
            var grid = new Grid3D(4, 4, 3, new D3(.1, .1, .1));
            var projection = new VariableCoefficientProjection(CutCellGeometry.Build(grid), BoundaryConditions3D.Closed, _ => 997, _ => 0);
            var predictor = new MomentumPredictor3D(projection);
            var velocity = new float[projection.FaceCount];
            var gravity = new D3(0, -9.80665, 0);
            bool failed = false;
            try { advance(predictor, velocity, gravity, .01, 1); }
            catch (FlowConvergenceException failure)
            {
                failed = failure.Diagnostics.Iterations == 1 && !failure.Diagnostics.Converged;
            }
            Check(id, "iteration cap rejected", failed ? 1 : 0, 1, 0);
            Check(id, "input unchanged after failed trial", Array.TrueForAll(velocity, u => u == 0) ? 1 : 0, 1, 0);
            RunStep(id + "-retry", predictor, velocity, gravity, .005);
            Check(id, "acceleration bounds oversized timestep", predictor.SelectTimestep(velocity, gravity, 1) < 1 ? 1 : 0, 1, 0);
        }

        private FlowStep RunStep(string id, MomentumPredictor3D predictor, float[] velocity, D3 acceleration, double dt)
        {
            var step = advance(predictor, velocity, acceleration, dt, 4096);
            var d = step.Diagnostics;
            Check(id, "pressure algebraic residual L2 [m3/s2]", d.ResidualL2, 0, d.RhsL2 == 0 ? 1e-12 : 1e-6 * d.RhsL2);
            Check(id, "post-projection dt times max absolute divergence", d.Dt * d.MaxDivergencePerS, 0, 1e-5);
            return step;
        }

        private static float[] XProfile(VariableCoefficientProjection projection, Func<int, double> profile)
        {
            var result = new float[projection.FaceCount];
            var shape = projection.Grid.XFaces;
            for (int k = 0; k < shape.Z; k++)
            for (int j = 0; j < shape.Y; j++)
            for (int i = 0; i < shape.X; i++)
                result[projection.FaceIndex(0, i, j, k)] = (float)profile(j);
            return result;
        }

        private static double Energy(float[] velocity)
        {
            double total = 0;
            foreach (float u in velocity) total += (double)u * u;
            return total;
        }

        private void Check(string id, string quantity, double actual, double expected, double tolerance)
        {
            Rows.Add(new FlowCheckRow
            {
                Case = id, Quantity = quantity, Actual = actual, Expected = expected, Tolerance = tolerance,
                Passed = SiUnits.IsFinite(actual) && Math.Abs(actual - expected) <= tolerance
            });
        }
    }
}
