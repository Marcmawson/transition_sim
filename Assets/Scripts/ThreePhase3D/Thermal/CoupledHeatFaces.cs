using System;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Thermal
{
    // S08/V17: one signed interface rate to debit from A and credit to B.
    public static class CoupledHeatFaces
    {
        public static double HeatRateW(double areaM2, double temperatureAK, double temperatureBK,
            double distanceAM, double conductivityAWmK, double contactResistanceM2KW,
            double distanceBM, double conductivityBWmK)
        {
            if (!SiUnits.IsFinite(areaM2) || areaM2 < 0)
                throw new ArgumentOutOfRangeException(nameof(areaM2));
            if (!SiUnits.IsFinite(temperatureAK) || temperatureAK <= 0)
                throw new ArgumentOutOfRangeException(nameof(temperatureAK));
            if (!SiUnits.IsFinite(temperatureBK) || temperatureBK <= 0)
                throw new ArgumentOutOfRangeException(nameof(temperatureBK));
            if (!SiUnits.IsFinite(contactResistanceM2KW) || contactResistanceM2KW < 0)
                throw new ArgumentOutOfRangeException(nameof(contactResistanceM2KW));

            double resistance = LayerResistance(distanceAM, conductivityAWmK) + contactResistanceM2KW +
                LayerResistance(distanceBM, conductivityBWmK);
            if (!SiUnits.IsFinite(resistance) || resistance <= 0)
                throw new ArgumentOutOfRangeException(nameof(contactResistanceM2KW), "Total resistance must be finite and positive.");
            return areaM2 * ((temperatureAK - temperatureBK) / resistance);
        }

        private static double LayerResistance(double distanceM, double conductivityWmK)
        {
            if (!SiUnits.IsFinite(distanceM) || distanceM < 0)
                throw new ArgumentOutOfRangeException(nameof(distanceM));
            if (!SiUnits.IsFinite(conductivityWmK) || conductivityWmK <= 0)
                throw new ArgumentOutOfRangeException(nameof(conductivityWmK));
            return distanceM / conductivityWmK;
        }
    }
}
