# S08 GPU thermal preview

Open `Assets/Scenes/ThreePhase3D/BeakerThermalS08.unity`, enter Play Mode and click **Burner on**. The preview first checks 20 GPU steps against the CPU reference. The panel shows the glass base cross-section, wall and tripod temperatures, absorbed energy, stored energy, heat transferred to the room, and the water volume, depth, mean temperature and stored energy.

Select **Three Phase 3D Apparatus → Solid Beaker Thermal Preview** in Edit Mode. The 66 mm internal-diameter beaker starts with **9 mL**. Enable **Larger Beaker** for the 120 mm internal-diameter beaker with **250 mL**. Disable **Include Water** for a dry run. A continuous, closed water surface occupies the calculated fill depth. The top, bottom and side use interpolated vertex colours from their local radial and vertical water cells. Both presets default to blue at 20 °C and red at 30 °C, with linear interpolation and clamping. **Hot Water Colour C** sets the red endpoint in the Inspector. The display refreshes from the existing GPU diagnostic snapshot.

The GPU advances the axisymmetric glass, support ring, three segmented tripod legs and stationary water. The burner adds heat only at the outer glass base. The water exchanges heat with the base and submerged wall via common interface conductances. Water density, heat capacity and conductivity are evaluated from the repository's Water95 provider at 20 °C and 101325 Pa and held constant during this reference test. The ideal wetted glass-water interface uses zero contact resistance.

## CPU reference checks

Run `dotnet run -c Release --project Tools/ThreePhase3D/SolidConductionChecks.csproj`.

| Water volume | Fill depth | Energy stored in water after 30 s at 10 W | Mean rise |
| ---: | ---: | ---: | ---: |
| 9 mL | 2.631 mm | 125.953 J | 3.3508 K |
| 250 mL | 22.105 mm | 120.572 J | 0.1155 K |

Both runs conserve the full 300 J across the solids and water. The tests check the represented volume, water thermal capacity and zero direct burner power in the water cells.

The Play Mode preview applies a prescribed synthetic film coefficient of 8 W/(m² K) to exposed surfaces facing a 20 °C room. Set **Room Film Wm2K** to zero in the Inspector for the insulated reference. The wetted glass-water interface retains its internal heat exchange. The panel records room exchange separately from stored energy. It does not establish heating rates for flowing water or a real apparatus. S09 fluid enthalpy advection and natural convection, radiation, conductive heat loss to the bench and phase change remain pending.

The cooldown checks apply 10 W for 30 s, then switch the burner off for 30 s. Stored energy fell from 293.215 to 284.026 J in the 9 mL case and from 293.567 to 288.179 J in the 250 mL case. In both cases, stored energy plus cumulative room heat balanced the 300 J of burner input. This is a synthetic V40 precursor with no measured cooldown curve.
