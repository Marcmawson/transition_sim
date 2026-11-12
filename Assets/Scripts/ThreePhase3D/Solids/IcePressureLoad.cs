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
            Func<InterfacePatch, D3, double> pressurePa) =>
            Integrate(body, geometry, pressurePa, null);

        public static IcePressureLoad Hydrostatic(IceBody body, CutCellGeometry geometry,
            D3 waterlinePointM, D3 gravityMPerS2, double liquidDensityKgM3)
        {
            if (geometry == null) throw new ArgumentNullException(nameof(geometry));
            if (!SiUnits.IsFinite(liquidDensityKgM3) || liquidDensityKgM3 <= 0)
                throw new ArgumentOutOfRangeException(nameof(liquidDensityKgM3));
            D3 up = (-gravityMPerS2).Unit();
            var frame = geometry.Grid.Frame;
            var plane = new ClipPlane(new D3(D3.Dot(up, frame.U), D3.Dot(up, frame.V), D3.Dot(up, frame.W)),
                frame.ToLocal(waterlinePointM));
            return Integrate(body, geometry,
                (_, point) => liquidDensityKgM3 * D3.Dot(gravityMPerS2, point - waterlinePointM), _ => plane);
        }

        private static IcePressureLoad Integrate(IceBody body, CutCellGeometry geometry,
            Func<InterfacePatch, D3, double> pressurePa, Func<InterfacePatch, ClipPlane> clip)
        {
            if (body == null) throw new ArgumentNullException(nameof(body));
            if (geometry == null) throw new ArgumentNullException(nameof(geometry));
            if (pressurePa == null) throw new ArgumentNullException(nameof(pressurePa));
            var frame = geometry.Grid.Frame;
            D3 force = default, torque = default;
            foreach (var patch in geometry.Patches)
            {
                if (patch.Role != SolidRole.Ice || patch.FluidExposedAreaM2 <= 0) continue;
                var n = patch.NormalLocal;
                D3 normal = frame.U * n.X + frame.V * n.Y + frame.W * n.Z;
                ClipPlane waterline = clip != null ? clip(patch) : default;
                IntegratePolygon(patch.Polygon, 1);
                if (patch.CoveredPieces != null)
                    foreach (var piece in patch.CoveredPieces)
                        IntegratePolygon(piece.Polygon, -piece.Sign);

                void IntegratePolygon(IReadOnlyList<D3> polygon, double sign)
                {
                    if (clip != null)
                        polygon = PolyhedronClip.Polygon(new List<D3>(polygon), waterline);
                    if (polygon.Count < 3) return;
                    D3 a = polygon[0];
                    for (int i = 1; i + 1 < polygon.Count; i++)
                    {
                        D3 b = polygon[i], c = polygon[i + 1];
                        double weight = sign * D3.Cross(b - a, c - a).Length / 6;
                        if (weight == 0) continue;
                        // Degree-two quadrature also integrates the moment of affine pressure.
                        Accumulate((a * 4 + b + c) / 6, weight);
                        Accumulate((a + b * 4 + c) / 6, weight);
                        Accumulate((a + b + c * 4) / 6, weight);
                    }

                    void Accumulate(D3 localPoint, double weight)
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
            IReadOnlyList<double> groupPressurePa, Func<D3, double> backgroundPressurePa = null, ColumnHydrostaticPressure columnPressure = null)
        {
            if (plan == null) throw new ArgumentNullException(nameof(plan));
            if (groupPressurePa == null || groupPressurePa.Count != plan.GroupCount)
                throw new ArgumentException("One pressure is required per projection group.", nameof(groupPressurePa));
            if (columnPressure != null && backgroundPressurePa != null)
                throw new ArgumentException("Supply one hydrostatic background.");
            var baseLoad = Integrate(body, plan.Geometry, (patch, point) =>
            {
                int group = plan.GroupByCell[patch.Cell];
                if (group < 0) throw new InvalidOperationException("An exposed ice patch has no fluid pressure group.");
                return groupPressurePa[group] + (backgroundPressurePa?.Invoke(point) ?? 0) + (columnPressure?.AirAt(point) ?? 0);
            });
            if (columnPressure == null) return baseLoad;
            var wet = Integrate(body, plan.Geometry, (patch, point) => columnPressure.WaterExcessAt(patch.Cell, point),
                patch => columnPressure.WaterlinePlane(patch.Cell));
            return new IcePressureLoad(baseLoad.ForceN + wet.ForceN, baseLoad.TorqueNm + wet.TorqueNm);
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
                int group = plan.GroupByCell[patch.Cell];
                if (group < 0) throw new InvalidOperationException("An exposed ice patch has no fluid pressure group.");
                var n = patch.NormalLocal;
                D3 normal = frame.U * n.X + frame.V * n.Y + frame.W * n.Z;
                flux[group] -= NormalVelocity(patch.CentroidLocalM) * patch.AreaM2;
                if (patch.CoveredPieces != null)
                    foreach (var piece in patch.CoveredPieces)
                        flux[group] += piece.Sign * PolyhedronClip.Area(piece.Polygon) *
                            NormalVelocity(PolyhedronClip.Centroid(piece.Polygon));

                double NormalVelocity(D3 point) => D3.Dot(normal, body.VelocityAt(frame.ToWorld(point)));
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
