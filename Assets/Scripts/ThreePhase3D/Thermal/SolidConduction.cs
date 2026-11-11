using System;
using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Thermal
{
    // Constant-property finite-volume reference for S08. Each cell has
    // a heat capacity rho * cp * volume in J/K and a cell-centred temperature.
    public sealed class SolidConduction
    {
        public readonly struct Face
        {
            public readonly int A;
            public readonly int B;
            public readonly double ConductanceWPerK;

            public Face(int a, int b, double conductanceWPerK)
            {
                A = a;
                B = b;
                ConductanceWPerK = conductanceWPerK;
            }
        }

        private readonly double[] capacity;
        private readonly double[] temperature;
        private readonly Face[] faces;
        private readonly double[] power;
        private readonly double[] nextTemperature;

        public double MaximumStepSeconds { get; }
        public IReadOnlyList<double> TemperatureK { get; }

        public SolidConduction(double[] capacityJPerK, double[] initialTemperatureK, Face[] internalFaces)
        {
            if (capacityJPerK == null || initialTemperatureK == null || internalFaces == null ||
                capacityJPerK.Length == 0 || capacityJPerK.Length != initialTemperatureK.Length)
                throw new ArgumentException("A nonempty set of cells with matching temperatures and a face array are required.");

            capacity = (double[])capacityJPerK.Clone();
            temperature = (double[])initialTemperatureK.Clone();
            faces = (Face[])internalFaces.Clone();
            power = new double[capacity.Length];
            nextTemperature = new double[capacity.Length];
            TemperatureK = Array.AsReadOnly(temperature);

            var incidentConductance = new double[capacity.Length];
            foreach (Face face in faces)
            {
                if (face.A < 0 || face.A >= capacity.Length || face.B < 0 ||
                    face.B >= capacity.Length || face.A == face.B ||
                    !SiUnits.IsFinite(face.ConductanceWPerK) || face.ConductanceWPerK <= 0)
                    throw new ArgumentException("Each face needs two distinct cells and a positive finite conductance.", nameof(internalFaces));

                incidentConductance[face.A] += face.ConductanceWPerK;
                incidentConductance[face.B] += face.ConductanceWPerK;
            }

            double limit = double.PositiveInfinity;
            for (int i = 0; i < capacity.Length; i++)
            {
                if (!SiUnits.IsFinite(capacity[i]) || capacity[i] <= 0 ||
                    !SiUnits.IsFinite(temperature[i]) || temperature[i] <= 0 ||
                    !SiUnits.IsFinite(incidentConductance[i]))
                    throw new ArgumentException("Cell heat capacities and temperatures must be positive and finite, with finite total conductance.");

                if (incidentConductance[i] > 0)
                    limit = Math.Min(limit, capacity[i] / incidentConductance[i]);
            }
            MaximumStepSeconds = limit;
        }

        // External powers are signed, in W, one per cell. Evaluate every
        // internal face from the old state and apply equal and opposite rates.
        public void Step(double seconds, double[] externalPowerW)
        {
            if (!SiUnits.IsFinite(seconds) || seconds <= 0 || seconds > MaximumStepSeconds)
                throw new ArgumentOutOfRangeException(nameof(seconds), "Step exceeds the explicit conduction limit or is invalid.");
            if (externalPowerW == null || externalPowerW.Length != power.Length)
                throw new ArgumentException("A boundary power is required for every cell.", nameof(externalPowerW));

            Array.Copy(externalPowerW, power, power.Length);
            foreach (Face face in faces)
            {
                double rate = face.ConductanceWPerK * (temperature[face.A] - temperature[face.B]);
                power[face.A] -= rate;
                power[face.B] += rate;
            }

            for (int i = 0; i < temperature.Length; i++)
            {
                double updated = temperature[i] + seconds * power[i] / capacity[i];
                if (!SiUnits.IsFinite(updated) || updated <= 0)
                    throw new InvalidOperationException("The supplied powers produced an invalid temperature.");

                nextTemperature[i] = updated;
            }
            Array.Copy(nextTemperature, temperature, temperature.Length);
        }
    }
}
