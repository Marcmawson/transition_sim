# S10 V21 seeded planar melting

Run `dotnet run --project Tools/ThreePhase3D/PlanarStefanChecks.csproj` from the repository root.

This one-phase Stefan fixture holds stationary ice at the melting point and conducts heat through the liquid from a wall held 10 K warmer. Both phases have the same idealized density of 1000 kg/m3. Liquid heat capacity is 4200 J/(kg K), conductivity is 0.6 W/(m K), and latent heat is 333427 J/kg. The numerical calculation starts from the analytic front and liquid profile at 10 s and advances to 100 s. The reference is the similarity solution specified in [V21](Validation.md).

The finite-volume solver uses a moving mesh on the liquid interval. The interface is held at the melting point, and the sensible heat flux reaching it supplies exactly the latent heat of the advancing front. The energy ledger compares wall input with the increase in sensible plus latent enthalpy per unit wall area.

| Liquid cells | Front error (m) | Maximum temperature error (K) | Energy residual (J/m2) |
| ---: | ---: | ---: | ---: |
| 16 | 1.522e-6 | 0.0078161 | -3.6089e-9 |
| 32 | 7.8006e-7 | 0.0040196 | 1.6298e-9 |
| 64 | 3.9498e-7 | 0.002038 | 3.085e-9 |

The reference front at 100 s is 0.00185908479 m. The fine-grid front error is 0.0213%, below the 2% V21 limit. Both front and temperature errors decrease with refinement. These results are from the CPU constant-property benchmark. GPU integration, freezing with heat conducted through ice, and unequal-density volume coupling remain open.
