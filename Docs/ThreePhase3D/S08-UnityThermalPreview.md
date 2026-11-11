# S08 GPU thermal preview

Open `Assets/Scenes/ThreePhase3D/BeakerThermalS08.unity`, enter Play Mode and click **Burner on**. The preview first checks 20 GPU steps against the CPU reference. The panel shows the glass base cross-section, wall and tripod temperatures, absorbed and stored energy, and the water volume, depth, mean temperature and stored energy.

Select **Three Phase 3D Apparatus → Solid Beaker Thermal Preview** in Edit Mode. The 66 mm internal-diameter beaker starts with **9 mL**. Enable **Larger Beaker** for the 120 mm internal-diameter beaker with **250 mL**. Disable **Include Water** for a dry run. Both the visible stationary fill and the thermal cells use the depth obtained by dividing the selected volume by the beaker's internal cross-sectional area.

The GPU advances the axisymmetric glass, support ring, three segmented tripod legs and stationary water. The burner adds heat only at the outer glass base. The water exchanges heat with the base and submerged wall via common interface conductances. Water density, heat capacity and conductivity are evaluated from the repository's Water95 provider at 20 °C and 101325 Pa and held constant during this reference test. The ideal wetted glass-water interface uses zero contact resistance.

## CPU reference checks

Run `dotnet run -c Release --project Tools/ThreePhase3D/SolidConductionChecks.csproj`.

| Water volume | Fill depth | Energy stored in water after 30 s at 10 W | Mean rise |
| ---: | ---: | ---: | ---: |
| 9 mL | 2.631 mm | 125.953 J | 3.3508 K |
| 250 mL | 22.105 mm | 120.572 J | 0.1155 K |

Both runs conserve the full 300 J across the solids and water. The tests check the represented volume, water thermal capacity and zero direct burner power in the water cells.

This is a stationary, insulated conduction case with synthetic glass and support properties. It does not establish heating rates for flowing water or a real apparatus. S09 fluid enthalpy advection and natural convection, ambient heat loss and phase change remain pending.
