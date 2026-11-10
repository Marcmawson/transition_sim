using UnityEngine;

namespace PhaseTransition.Physics
{
    /// <summary>
    /// Minimal SI energy ledger for a fixed mass of water/ice in the beaker experiment.
    /// Heater power is an experiment input in watts; all integrated energy is joules.
    /// This does not yet model spatial heat flow, fluid motion, evaporation into an open
    /// atmosphere, or heat loss to the beaker and surroundings.
    /// </summary>
    public sealed class BeakerEnergyLedger : MonoBehaviour
    {
        [Header("Initial sample")]
        [Min(0.001f)] public float sampleMassKg = 0.25f;
        [Range(-100f, 0f)] public float initialIceTemperatureC = -50f;

        [Header("Experimental input")]
        [Min(0f)] public float heaterPowerW = 500f;
        public bool heaterEnabled = true;
        [Min(0f)] public float altitudeM = 0f;

        public double SpecificEnthalpyJPerKg { get; private set; }
        public WaterThermodynamicsSI.EquilibriumState CurrentState { get; private set; }
        public double AmbientPressurePa { get; private set; }
        private bool initialized;

        private void Start() => ResetSample();

        private void Update()
        {
            if (!initialized) ResetSample();
            AmbientPressurePa = WaterThermodynamicsSI.AmbientPressureAtAltitudePa(altitudeM);
            if (heaterEnabled && sampleMassKg > 0f)
                SpecificEnthalpyJPerKg += heaterPowerW * Time.deltaTime / sampleMassKg;
            CurrentState = WaterThermodynamicsSI.StateFromSpecificEnthalpy(
                SpecificEnthalpyJPerKg, AmbientPressurePa);
        }

        [ContextMenu("Reset sample to initial ice temperature")]
        public void ResetSample()
        {
            float mass = Mathf.Max(0.001f, sampleMassKg);
            sampleMassKg = mass;
            SpecificEnthalpyJPerKg = WaterThermodynamicsSI.IceSpecificEnthalpyJPerKg(
                initialIceTemperatureC + 273.15);
            AmbientPressurePa = WaterThermodynamicsSI.AmbientPressureAtAltitudePa(altitudeM);
            CurrentState = WaterThermodynamicsSI.StateFromSpecificEnthalpy(
                SpecificEnthalpyJPerKg, AmbientPressurePa);
            initialized = true;
        }

        /// <summary>Apply or remove energy transferred to the sample, positive into it.</summary>
        public void AddEnergyJ(double energyJ)
        {
            if (sampleMassKg <= 0f) return;
            SpecificEnthalpyJPerKg += energyJ / sampleMassKg;
            CurrentState = WaterThermodynamicsSI.StateFromSpecificEnthalpy(
                SpecificEnthalpyJPerKg, AmbientPressurePa);
        }
    }
}
