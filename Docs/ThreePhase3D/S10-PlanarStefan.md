# S10 V21 seeded planar melting

Run `dotnet run --project Tools/ThreePhase3D/PlanarStefanChecks.csproj` from the repository root.

This one-phase Stefan fixture holds stationary ice at the melting point and conducts heat through the liquid from a wall held 10 K warmer. Both phases have the same idealized density of 1000 kg/m3. Liquid heat capacity is 4200 J/(kg K), conductivity is 0.6 W/(m K), and latent heat is 333427 J/kg. The numerical calculation starts from the analytic front and liquid profile at 10 s and advances to 100 s. The reference is the similarity solution specified in [V21](Validation.md).

The finite-volume solver uses a moving mesh on the liquid interval. The interface is held at the melting point, and the sensible heat flux reaching it supplies exactly the latent heat of the advancing front. The energy ledger compares wall input with the increase in sensible plus latent enthalpy per unit wall area.

| Liquid cells | Front error (m) | Maximum temperature error (K) | Energy residual (J/m2) |
| ---: | ---: | ---: | ---: |
| 16 | 1.522e-6 | 0.0078161 | -3.6089e-9 |
| 32 | 7.8006e-7 | 0.0040196 | 1.6298e-9 |
| 64 | 3.9498e-7 | 0.002038 | 3.085e-9 |

The reference front at 100 s is 0.00185908479 m. The fine-grid front error is 0.0213%, below the 2% V21 limit. Both front and temperature errors decrease with refinement. These results are from the CPU constant-property benchmark. The freezing case follows below.

## V22 seeded planar freezing

The same benchmark now applies a wall 10 K below the melting point and advances a seeded ice front from 10 s to 100 s. The liquid ahead of the front stays at the melting point. The fixture uses constant ice heat capacity of 2100 J/(kg K), ice conductivity of 2.2 W/(m K), and the same idealized equal density and latent heat as V21. The interface remains sharp at every resolution, with zero numerical mushy interval.

| Ice cells | Front error (m) | Maximum temperature error (K) | Energy residual (J/m2) |
| ---: | ---: | ---: | ---: |
| 16 | 1.4938e-6 | 0.0040384 | -1.0827e-8 |
| 32 | 7.653e-7 | 0.002078 | 1.1991e-8 |
| 64 | 3.8744e-7 | 0.0010539 | 1.6298e-8 |

The analytic front at 100 s is 0.00359543106 m. The fine run removed 845527.232 J/m2 through the cold wall. Its front released 819844.767 J/m2 of latent heat, compared with 819715.584 J/m2 in the analytic solution. The remainder is sensible cooling. The V21 melting regression and V22 freezing benchmark both passed on CPU. GPU integration and V23 volume coupling remain open.

## GPU planar solver check

Run **Physics Sim > Three Phase 3D > Run S10 GPU planar Stefan check** in Unity. The compute shader advances a 32-cell moving mesh through 4096 fixed steps for each case, from a 2 mm seed with a linear temperature profile. The GPU holds temperature, front position and wall energy across batches. Each case compares the final front, temperature profile and wall heat against the FP64 CPU solver with the same initial conditions and timestep, and independently closes the energy ledger. The analytic seeded runs above remain the independent discretisation checks.

The Editor code compiled against Unity 2022.3.46f1 assemblies and the kernel compiled for Direct3D Shader Model 5. An FP32 arithmetic replay of the kernel update met the benchmark tolerances for both cases. Unity GPU execution and coupling to the beaker simulation remain open.
