# S08 thermal boundary references

These CPU reference functions implement the prescribed Gaussian burner boundary and signed interface heat rate from Specification sections 7.1 and 7.2. The Gaussian is integrated over physical annular areas on the external base and normalised to absorbed burner watts. The interface helper returns one signed heat rate to debit from one inventory and credit to the other. Its conductivity and contact resistance come from the caller.

| Synthetic case | Internal diameter | Cavity height | Water volume for subsequent coupled tests |
| --- | ---: | ---: | ---: |
| Small drop | 66 mm | 60 mm | 9 mL |
| Larger beaker | 120 mm | 160 mm | 250 mL |

Both reference cases use a 3 mm wall, a 3 mm base, 100 W absorbed power and a 15 mm Gaussian width. These are synthetic test inputs. ThermalBoundaryMain checks the fit against analytic cavity volume, integrates the burner power using 1, 4, 16 and 64 annuli, checks radial distribution and compares the paired heat rate against an analytic composite resistance. The water volumes are fixture inputs for subsequent coupled runs. No water temperature is solved by these checks.

The annular boundary is a reference for validating the GPU face mapping. The tabulated profile, actual cut-face integration, solid enthalpy evolution, glass and tripod coupling, ambient losses, and V16–V18/V40 acceptance remain open. The ParticleBeakerDrop scene has no thermal coupling in this increment.
