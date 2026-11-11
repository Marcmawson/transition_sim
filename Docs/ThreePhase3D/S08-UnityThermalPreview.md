# S08 GPU thermal preview

Open `Assets/Scenes/ThreePhase3D/BeakerThermalS08.unity` and enter Play Mode. The preview checks 20 GPU conduction steps against the CPU reference on startup. When the check passes, click **Burner on**. The cross-section runs from centre to rim horizontally and from underside to upper face vertically. The panel shows centre and outer glass temperatures and the support temperature. Click **Burner off** to watch heat redistribute or **Reset** to restore 20 °C.

In Edit Mode, select **Three Phase 3D Apparatus → Solid Beaker Thermal Preview** and enable **Larger Beaker** to switch from the 66 mm internal diameter geometry to the 120 mm geometry associated with the planned 250 mL test. The **Absorbed Burner Power W** field sets the starting power.

The GPU advances the 16 × 6 axisymmetric glass-base field and one synthetic support heat capacity. The prescribed 15 mm Gaussian deposits absorbed power on the underside only. Each pair of neighbouring cells uses the same face conductance. The CPU builds the fixture, runs a short reference check and reads back a diagnostic snapshot four times per simulated second.

This preview covers the dry glass base and a single support node using synthetic constant properties. The full S08 gate still requires the resolved glass walls and tripod, material-range checks, ambient and radiation boundaries, and spatial refinement. The 250 mL geometry is available here for the dry test. Glass-to-water exchange and fluid enthalpy transport remain pending.
