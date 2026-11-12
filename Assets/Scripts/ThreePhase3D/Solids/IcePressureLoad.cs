using System;
using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Geometry;

namespace PhysicsSim.ThreePhase3D.Solids
{
    public readonly struct IcePressureLoad
    {
        public readonly D3 ForceN, TorqueNm;

        private IcePressureLoad(D3 forceN, D3 torqueNm)
        {
            ForceN = forceN;
            TorqueNm = torqueNm;
        }

        // Pressure in Pa on the fluid side of the ice. Torque is about the
        // supplied body's centre of mass. Geometry must describe this pose.
        public static IcePressureLoad Integrate(IceBody body, CutCellGeometry geometry,
            Func<InterfacePatch, D3, double> pressurePa)
        {
            if (body == null) throw new ArgumentNullException(nameof(body));
            if (geometry == null) throw new ArgumentNullException(nameof(geometry));
            if (pressurePa == null) throw new ArgumentNullException(nameof(pressurePa));
            var frame = geometry.Grid.Frame;
            D3 force = default, torque = default;
            foreach (var patch in geometry.Patches)
            {
                if (patch.Role != SolidRole.Ice || patch.FluidExposedAreaM2 <= 0) continue;
                if (patch.ContactAreaM2 > 0)
                    throw new NotSupportedException("Pressure torque on a partially blocked patch requires its exposed polygon.");
                var n = patch.NormalLocal;
                D3 normal = frame.U * n.X + frame.V * n.Y + frame.W * n.Z;
                D3 a = patch.Polygon[0];
                for (int i = 1; i + 1 < patch.Polygon.Count; i++)
                {
                    D3 b = patch.Polygon[i], c = patch.Polygon[i + 1];
                    double weight = D3.Cross(b - a, c - a).Length / 6;
                    if (weight == 0) continue;
                    // Degree-two triangle quadrature integrates the moment
                    // of an affine pressure exactly on a planar polygon.
                    Accumulate((a * 4 + b + c) / 6);
                    Accumulate((a + b * 4 + c) / 6);
                    Accumulate((a + b + c * 4) / 6);

                    void Accumulate(D3 localPoint)
                    {
                        D3 point = frame.ToWorld(localPoint);
                        double pressure = pressurePa(patch, point);
                        if (!SiUnits.IsFinite(pressure))
                            throw new ArgumentException("Ice surface pressure must be finite.", nameof(pressurePa));
                        D3 traction = normal * (-pressure * weight);
                        force += traction;
                        torque += D3.Cross(point - body.Frame.Origin, traction);
                    }
                }
            }
            return new IcePressureLoad(force, torque);
        }

        // Use the accepted group pressure as a piecewise-constant sample on
        // each patch. An optional background restores a hydrostatic split.
        public static IcePressureLoad FromProjection(IceBody body, VariableCoefficientProjection plan,
            IReadOnlyList<double> groupPressurePa, Func<D3, double> backgroundPressurePa = null)
        {
            if (plan == null) throw new ArgumentNullException(nameof(plan));
            if (groupPressurePa == null || groupPressurePa.Count != plan.GroupCount)
                throw new ArgumentException("One pressure is required per projection group.", nameof(groupPressurePa));
            return Integrate(body, plan.Geometry, (patch, point) =>
            {
                int group = plan.GroupByCell[patch.Cell];
                if (group < 0) throw new InvalidOperationException("An exposed ice patch has no fluid pressure group.");
                return groupPressurePa[group] + (backgroundPressurePa?.Invoke(point) ?? 0);
            });
        }

        // The outward fluid normal at the ice boundary points into the ice.
        // Rigid velocity is affine, so its integral uses the patch centroid.
        public static double[] OutwardFluidFlux(IceBody body, VariableCoefficientProjection plan)
        {
            if (body == null) throw new ArgumentNullException(nameof(body));
            if (plan == null) throw new ArgumentNullException(nameof(plan));
            var flux = new double[plan.GroupCount];
            var frame = plan.Grid.Frame;
            foreach (var patch in plan.Geometry.Patches)
            {
                if (patch.Role != SolidRole.Ice || patch.FluidExposedAreaM2 <= 0) continue;
                if (patch.ContactAreaM2 > 0)
                    throw new NotSupportedException("Moving ice requires an exposed polygon at partial contact.");
                int group = plan.GroupByCell[patch.Cell];
                if (group < 0) throw new InvalidOperationException("An exposed ice patch has no fluid pressure group.");
                var n = patch.NormalLocal;
                D3 normal = frame.U * n.X + frame.V * n.Y + frame.W * n.Z;
                D3 point = frame.ToWorld(patch.CentroidLocalM);
                flux[group] -= D3.Dot(normal, body.VelocityAt(point)) * patch.AreaM2;
            }
            return flux;
        }

        public IceBody Advance(IceBody body, double seconds, D3 gravityMPerS2)
        {
            if (body == null) throw new ArgumentNullException(nameof(body));
            return body.Advance(seconds, ForceN + gravityMPerS2 * body.MassKg, TorqueNm);
        }
    }
}
