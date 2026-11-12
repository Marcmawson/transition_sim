using System;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Geometry;
using PhysicsSim.ThreePhase3D.PhaseChange;
using PhysicsSim.ThreePhase3D.Solids;

namespace PhysicsSim.ThreePhase3D.Flow
{
    // Apply the thermal and geometry updates at the pose used by the pressure projection.
    public static class IceFaceMeltingStep
    {
        public static (IceBody Body, D3 DimensionsM, VariableCoefficientProjection Topology,
            CutCellInventories Fields, CutCellMomentumResult Momentum, double GasDensityKgM3, double HeatJ)
            Advance(VariableCoefficientProjection plan, CutCellInventories fields, FlowStep flow,
                IceBody body, D3 dimensionsM, int axis, int side, SolidLiquidInterface equilibrium,
                double gasDensityKgM3, double conductivityWmK, double[] liquidTemperatureK,
                Func<InterfacePatch, double> wettedAreaM2,
                Func<CutCellGeometry, VariableCoefficientProjection> buildTopology, double maxCorrectionM3,
                IceFaceConduction.TransientLayer? transientLayer = null)
        {
            if (plan == null || body == null || flow == null || equilibrium == null || buildTopology == null)
                throw new ArgumentNullException();
            if (axis < 0 || axis > 2) throw new ArgumentOutOfRangeException(nameof(axis));
            if (side != -1 && side != 1) throw new ArgumentOutOfRangeException(nameof(side));
            if (!flow.Diagnostics.Converged || flow.VelocityMPerS.Length != plan.FaceCount)
                throw new ArgumentException("Melting requires a converged projection on the ice pose.", nameof(flow));

            double rhoLiquid = equilibrium.LiquidMeltingDensityKgM3;
            D3 normal = (axis == 0 ? body.Frame.U : axis == 1 ? body.Frame.V : body.Frame.W) * side;
            var conducted = IceFaceConduction.Transfer(plan, fields, body, equilibrium, normal,
                flow.Diagnostics.Dt, rhoLiquid, conductivityWmK, liquidTemperatureK, wettedAreaM2, transientLayer);
            var melt = IceBoxMelting.Apply(body, dimensionsM, axis, side, conducted.HeatJ, equilibrium);
            if (melt.Released == null)
            {
                var swept = MovingCutCellRemap.Reconcile(plan, plan, flow, maxCorrectionM3);
                var unchanged = MovingCutCellRemap.ApplyWithMomentum(plan, plan, fields, swept,
                    flow.VelocityMPerS, rhoLiquid, gasDensityKgM3);
                return (body, dimensionsM, plan, unchanged.Fields, unchanged.Momentum, gasDensityKgM3, 0);
            }
            if (melt.UnusedHeatJ > 1e-12)
                throw new InvalidOperationException("The conductive heat exceeds the latent capacity.");

            TriangleSurface surface = melt.Remaining == null ? null : SurfacePrimitives.Transform(
                SurfacePrimitives.Box(-melt.DimensionsM * 0.5, melt.DimensionsM * 0.5), melt.Remaining.Frame);
            var geometry = plan.Geometry.WithIceSurface(surface);
            var nextPlan = buildTopology(geometry);
            if (nextPlan == null || nextPlan.Geometry != geometry)
                throw new InvalidOperationException("The next projection plan must use the receded geometry.");
            double iceDensity = body.MassKg / (dimensionsM.X * dimensionsM.Y * dimensionsM.Z);
            var sources = MeltwaterCutCellSource.Build(plan.Geometry, geometry, melt.Released,
                melt.Released.MassKg / iceDensity, melt.LiquidVolumeM3, melt.LiquidEnthalpyJ);
            var remap = MovingCutCellRemap.ApplyMeltingWithMomentum(plan, nextPlan, conducted.Fields, flow,
                sources, maxCorrectionM3, rhoLiquid, gasDensityKgM3);
            return (melt.Remaining, melt.DimensionsM, nextPlan, remap.Fields, remap.Momentum,
                remap.GasDensityKgM3, conducted.HeatJ);
        }
    }
}
