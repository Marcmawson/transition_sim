# Sources method selection and data provenance

Revision 1.0 · Sources checked on 10 November 2026.

This register links proposed components to primary publications, official formulations, manufacturer data and benchmark owners. It distinguishes an existing published method from this project's proposed integration. It does not claim that the coupled simulator already exists or has been validated.

## 1 Selection criteria

The relevant meaning of best is best supported for this application under stated criteria: conservation, high density contrast, heat/mass interface conditions, realistic material properties, verification cases, 3D implementation suitability and an achievable GPU design.

There is no empirically established single best paper across arbitrary hardware, geometry, phase behaviour, resolution and accuracy targets. The selection below is the proposed engineering baseline. If the programmer substitutes a method, provide a decision record and demonstrate equal or better performance on the same physical accuracy gates.

| Subsystem | Selected baseline | Reason and limit |
| --- | --- | --- |
| Liquid/vapour thermodynamics | IAPWS-95 [D01] | Reference formulation with published check values; GPU tables need their own error assessment |
| Ice equilibrium and properties | IAPWS ice Ih plus melt/sublimation release [D02, D03] | Consistent solid/fluid thermodynamic references; separate conductivity is still required |
| Water transport and surface tension | Separate IAPWS releases [D04, D05, D12] | EOS alone does not provide viscosity, conductivity or surface tension |
| Gas thermodynamics | IAPWS humid air/fugacity plus air data [D06, D07, D13] | Tracks non-condensable air rather than treating every gas cell as pure steam |
| Liquid interface and capillarity | Geometric VOF with balanced force and height functions [M01, M04] | Strong mass preservation and established capillary tests; adaptation to cut cells and phase change must be verified |
| Staggered mass and momentum fluxes | Conservative compatible transport using M13 | Addresses high density contrast; a finer transport grid has memory/cost implications |
| Liquid/gas phase change | M03 conservative energy/PLIC baseline; M02 species/interface treatment; M05 jump-condition benchmarks | Supports an auditable interface-resolved design; mixed-gas/variable-property integration requires a derived coupling |
| Ice melting/freezing | Enthalpy/Stefan method [M06] with explicit moving boundary | Established phase-energy balance; porous drag is not the final free-ice mechanics |
| Solid/fluid coupling | Variational pressure coupling [M09], extended with conservative heat and phase exchange | Provides a rigid-body pressure-coupling component; does not supply the complete thermal multiphase model |
| Boiling onset | Measured seeded protocol then Hsu-style cavity activation [M07] | Apparatus surface conditions are necessary inputs; no universal temperature-only spawn rule |
| Dynamic wetting | Measured contact angles and a Cox-type regularised contact-line treatment [M08] | Material and surface preparation matter; validity and matching scales must be recorded |
| Unresolved droplets/fog | Conservative parcels; M11 for applicable isolated-droplet transfer and M12 for statistical microphysics | Requires seed/aerosol inputs and beaker-specific validation; no claim of spontaneous nucleation from those methods alone |
| Glass and tripod conduction | Conservative conjugate finite volume heat balances with product-specific material data [D10, D11] | Physical coupling is explicit; incomplete manufacturer curves must be acquired or measured |
| Optics | Measured water, glass and ice optical data [O01, O02, D10] | Thermodynamics and render lighting remain separate unless radiative heating is explicitly modelled |

SPH, MPM, LBM and phase-field approaches are not ruled out as incapable. They can solve relevant problems with suitable formulations. They are not selected here as one universal drop-in model for all required phenomena. The acceptance criteria would still require the same conservation, phase-change, material and validation evidence.

A relevant alternative is the consistent thermo-gas-liquid-solid phase-field model [M14]. It explicitly addresses melting and conservation, but its stationary-solid drag treatment and lack of a complete boiling/evaporation/fog/combustion apparatus model do not remove this project's remaining coupling work.

## 2 Evidence and access policy

A source being listed does not mean every coefficient or equation has already been transcribed and independently checked. This document selects methods and verified source identities; the programmer must obtain the exact full version used before implementing coefficients or detailed stencils.

Access levels recorded below mean:

- **Official document/page inspected:** source owner and stated formulation/data inspected; full coefficient extraction remains a tracked implementation task unless explicitly stated.
- **Full primary text accessible:** a paper, official PDF or author-hosted primary text was retrieved during research.
- **Publication/abstract inspected:** identity, scope and selection rationale confirmed; full-text equation extraction is pending.
- **Manufacturer data inspected:** particular published values/conditions checked; no wider validity is inferred.

Some publisher and repository downloads were unavailable from the research environment. They are not represented as full-text reviews. Never invent equation numbers or replace unavailable coefficients with guessed fits.

Pin a lawful copy or persistent source identifier, publication version, access date and checksum when extracting data. Do not bundle third-party papers into the repository without permission. Published algorithm descriptions and available reference code have different licences; inspect software licences before copying code.

## 3 Units and material sources

### D00 SI units

BIPM, *The International System of Units*, 9th edition, updated 2026.

[Official SI brochure](https://www.bipm.org/en/publications/si-brochure).

Use for SI definitions and unit presentation. State-point choices and numerical tolerances are not material constants. Official page inspected.

### D01 Liquid and vapour thermodynamics

IAPWS, *R6-95(2018) Revised Release on the IAPWS Formulation 1995 for the Thermodynamic Properties of Ordinary Water Substance for General and Scientific Use*.

[Official release](https://iapws.org/technical-guidance/release/IAPWS-95) · [Official PDF](https://iapws.org/technical-guidance/release/IAPWS-95.download).

The Helmholtz-energy formulation supplies fluid thermodynamic properties and vapour/liquid equilibrium. Use its computer-program verification section, Tables 6–8, before generating GPU tables. The wide formulation range is not the simulation's validated range. Official PDF and verification-section location inspected.

### D02 Ice thermodynamics

IAPWS, *R10-06(2009) Revised Release on the Equation of State 2006 for H2O Ice Ih*.

[Official release](https://iapws.org/technical-guidance/release/Ice-2009).

Use the Gibbs-energy formulation for ordinary hexagonal ice and consistent solid/fluid property differences. Do not infer ice thermal conductivity from this EOS. Official formulation page inspected.

### D03 Melting and sublimation equilibrium

IAPWS, *R14-08(2011) Revised Release on the Pressure along the Melting and Sublimation Curves of Ordinary Water Substance*.

[Official release](https://www.iapws.org/relguide/MeltSub.html).

Use equilibrium pressure/temperature curves, consistent with D01/D02 for ice Ih. High-pressure ice polymorphs are not part of the first beaker scope. Official page inspected.

### D04 Water viscosity

IAPWS, *R12-08 Release on the IAPWS Formulation 2008 for the Viscosity of Ordinary Water Substance*.

[Official release](https://iapws.org/technical-guidance/release/viscosity).

Use for liquid and vapour viscosity within the release's state limits. Distinguish dynamic viscosity from kinematic viscosity. Official page inspected.

### D05 Water surface tension

IAPWS, *R1-76(2014) Revised Release on Surface Tension of Ordinary Water Substance*.

[Official release](https://www.iapws.org/relguide/Surf-H2O.html).

The reference concerns equilibrium pure water/vapour surface tension. Application to a clean water/air interface is a model assumption to check against the experiment; surfactant contamination cannot be inferred from temperature alone. Official page inspected.

### D06 Humid air

IAPWS, *G8-10 Guideline on an Equation of State for Humid Air in Contact with Seawater and Ice, Consistent with the IAPWS Formulation 2008 for the Thermodynamic Properties of Seawater*.

[Official release](https://iapws.org/technical-guidance/release/SeaAir).

Use the relevant humid-air thermodynamics and compatibility conditions, with its full domain/limitations checked during implementation. This is not a reason to introduce salt into the pure-water experiment. Official page inspected.

### D07 Water fugacity in humid air

IAPWS, *G11-15 Guideline on a Virial Equation for the Fugacity of H2O in Humid Air*.

[Official release](https://iapws.org/technical-guidance/release/VirialFugacity).

Full official release inspected. The continuation implements Eqs 2-11 and Tables 1-4 independently of the full Helmholtz mixture evaluator, checks all seven virial coefficients at 200/300/400 K against Table 7 and fugacity/auxiliary values against Table 8, then compares stable G8 states to G11 with an explicit 0.1% project cross-model threshold. This threshold is not the guideline's uncertainty or a lookup tolerance. Table 8 includes intentionally unstable states, supported only by a separately named reference function.

### D08 Water diffusion in air

W. J. Massman (1998), *A review of the molecular diffusivities of H2O, CO2, CH4, CO, O3, SO2, NH3, N2O, NO, and NO2 in air, O2 and N2 near STP*, Atmospheric Environment 32, 1111–1127.

[DOI](https://doi.org/10.1016/S1352-2310(97)00391-9).

Selected starting source for a water-in-air diffusion correlation near its supported regime. Full coefficient and validity extraction is pending; its title's near-STP qualifier must not be ignored at hot plume conditions. Publication identity checked; publisher full text unavailable during research.

### D09 Evaluated gas diffusion

D. R. Burgess Jr. (2024), *Self-Diffusion and Binary-Diffusion Coefficients in Gases*, NIST Technical Note 2279.

[NIST publication](https://www.nist.gov/publications/self-diffusion-and-binary-diffusion-coefficients-gases) · [DOI](https://doi.org/10.6028/NIST.TN.2279).

Official PDF inspected. Its H2O–N2 entry is not automatically a water–air entry. Use the evaluated data and original references to cross-check a justified gas-mixture treatment and SI conversions; some report columns use cm2/s.

### D10 DURAN borosilicate glass

SCHOTT, *DURAN Technical Data*, document PT_TTS_1019 EN, Business Unit Tubing, February 2026.

[Product and downloads](https://www.schott.com/en-gb/products/duran-p1000368).

Manufacturer sheet inspected. Values include density 2.23 g/cm3 at 25 °C, conductivity 1.2 W/(m K) at 90 °C, Young's modulus 63 GPa, Poisson ratio 0.2 and visible refractive index 1.473 at 587.6 nm. They are approximate state-specific data. The sheet does not supply the required full heat-capacity curve.

### D11 BOROFLOAT as a distinct glass dataset

SCHOTT, *Technical Details of BOROFLOAT*.

[Manufacturer data](https://www.schott.com/en-gb/products/borofloat/technical-details).

Manufacturer data inspected: the listed heat capacity is 0.83 kJ/(kg K) over 20–100 °C and conductivity is 1.2 W/(m K) at 90 °C. This is a different named product; use only when explicitly selecting that glass or documenting a proxy and its uncertainty. It does not fill an unmeasured DURAN data gap by identity.

### D12 Water thermal conductivity

IAPWS, *R15-11 Release on the IAPWS Formulation 2011 for the Thermal Conductivity of Ordinary Water Substance*.

[Official release](https://iapws.org/technical-guidance/release/ThCond).

Use the separate fluid transport formulation and official check values. Official page inspected.

### D13 Air transport

E. W. Lemmon and R. T. Jacobsen (2004), *Viscosity and Thermal Conductivity Equations for Nitrogen, Oxygen, Argon, and Air*, International Journal of Thermophysics 25, 21–69.

[NIST publication](https://www.nist.gov/publications/viscosity-and-thermal-conductivity-equations-nitrogen-oxygen-argon-and-air) · [DOI](https://doi.org/10.1023/B:IJOT.0000022327.04529.f3).

Use for dry-air transport reference data. A humid-mixture transport rule still needs to be specified and checked; dry-air values alone do not define steam-rich plume transport. Publication record inspected.

### D14 Ice conductivity

G. A. Slack (1980), *Thermal conductivity of ice*, Physical Review B 22, 3065.

[Primary publication](https://doi.org/10.1103/PhysRevB.22.3065).

Starting reference for low-pressure ice conductivity and its temperature dependence. Full formulation/measurement extraction is pending. Do not paste a conductivity correlation from an unattributed web table. Primary abstract inspected.

### D15 Ice preparation and corrected data

L. G. Bonales and co-authors (2017), *Thermal conductivity of ice prepared under different conditions*.

[Primary publication](https://doi.org/10.1080/10942912.2017.1306551) · [2018 corrigendum](https://doi.org/10.1080/10942912.2018.1482123).

This work and its correction are relevant to preparation-dependent ice conductivity. The corrigendum reports a coefficient interchange; any data extraction must use the corrected version. Publication and correction records inspected; no coefficients are asserted here.

## 4 Numerical method sources

### M01 Balanced capillary VOF

S. Popinet (2009), *An accurate adaptive solver for surface-tension-driven interfacial flows*, Journal of Computational Physics 228, 5838–5866.

[DOI](https://doi.org/10.1016/j.jcp.2009.04.042) · [Primary paper PDF](https://www.ljll.fr/~frey/papers/interfaces-free_surfaces/Popinet%20S.%2C%20An%20accurate%20adaptive%20solver%20for%20surface-tension%20driven%20interfacial%20flows.pdf).

Geometric VOF, balanced forces, height-function curvature and capillary verification. Its equilibrium-drop result is not a guarantee that translating drops or every high-density-ratio flow are free of numerical error. Full primary text accessible.

### M02 Evaporation and vapour species transport

N. Scapin, P. Costa and L. Brandt (2020), *A volume-of-fluid method for interface-resolved simulations of phase-changing two-fluid flows*, Journal of Computational Physics 407, 109251.

[DOI](https://doi.org/10.1016/j.jcp.2020.109251) · [Author preprint](https://arxiv.org/abs/1907.00890).

Selected for phase-interface velocity construction and evaporation in an inert gas. The paper's constant/incompressible phase and ideal-mixture assumptions must not be silently carried into an asserted variable-property real-gas model. Full author preprint accessible.

### M03 Conservative phase-change energy transport

L. C. Malan, A. G. Malan, S. Zaleski and P. G. Rousseau (2021), *A geometric VOF method for interface resolved phase change and conservative thermal energy advection*, Journal of Computational Physics 426, 109920.

[DOI](https://doi.org/10.1016/j.jcp.2020.109920) · [Author deposit](https://hal.science/hal-03654968).

Selected PLIC/sharp-interface and conservative thermal-advection baseline. Publication and abstract checked; full-text retrieval was unavailable in this environment. The programmer must retrieve the pinned paper before transcribing the algorithm.

### M04 Conservative volume transport

G. D. Weymouth and D. K.-P. Yue (2010), *Conservative Volume-of-Fluid method for free-surface simulations on Cartesian-grids*, Journal of Computational Physics 229, 2853–2865.

[DOI](https://doi.org/10.1016/j.jcp.2009.12.018) · [Author institutional record](https://eprints.soton.ac.uk/349787/).

Baseline for conservative geometric transport. It does not independently supply all phase-change, momentum, cut-cell or wetting extensions. Institutional publication record inspected.

### M05 Boiling verification and jumps

S. Tanguy, M. Sagan, B. Lalanne, F. Couderc and C. Colin (2014), *Benchmarks and numerical methods for the simulation of boiling flows*, Journal of Computational Physics 264, 1–22.

[DOI](https://doi.org/10.1016/j.jcp.2014.01.014) · [Author deposit](https://hal.science/hal-00973210).

Selected for analytical phase-change tests and treatment of thermal/velocity jumps. Publication and abstract checked; full-text retrieval was unavailable in this environment. Exact benchmark definitions must be extracted before implementation evidence is accepted.

### M06 Enthalpy and solidification

V. R. Voller and C. Prakash (1987), *A fixed grid numerical modelling methodology for convection-diffusion mushy region phase-change problems*, International Journal of Heat and Mass Transfer 30, 1709–1719.

[DOI](https://doi.org/10.1016/0017-9310(87)90317-6) · [Author institutional record](https://experts.umn.edu/en/publications/a-fixed-grid-numerical-modelling-methodology-for-convection-diffu/).

Use for an enthalpy-based stationary phase-change verification stage. A fixed solid/porosity closure is not the final moving ice model. Publication record inspected.

### M07 Boiling cavity activation

Y. Y. Hsu (1962), *On the Size Range of Active Nucleation Cavities on a Heating Surface*, Journal of Heat Transfer 84, 207–213.

[Primary publication](https://doi.org/10.1115/1.3684339).

Selected starting closure for apparatus-conditioned boiling onset. Requires wall/cavity and thermal-layer data. It does not justify a universal bubble count or a universal onset temperature. Primary abstract and scope inspected.

### M08 Dynamic wetting

R. G. Cox (1986), *The dynamics of the spreading of liquids on a solid surface. Part 1. Viscous flow*, Journal of Fluid Mechanics 168, 169–194.

[Primary publication](https://doi.org/10.1017/S0022112086000332).

A hydrodynamic contact-line model family, requiring defined matching scales and wettability input. Part 2 concerns surfactants and must not be confused with this selected paper. Publication and abstract inspected.

### M09 Pressure and rigid body coupling

C. Batty, F. Bertails and R. Bridson (2007), *A Fast Variational Framework for Accurate Solid-Fluid Coupling*, ACM Transactions on Graphics 26(3), article 100.

[Author paper and implementation page](https://www.cs.ubc.ca/labs/imager/tr/2007/Batty_VariationalFluids/) · [DOI](https://doi.org/10.1145/1276377.1276502).

Selected component reference for pressure-coupled rigid solids on Cartesian grids. It is a graphics/numerical coupling paper, not empirical validation of melting ice. The author's notes also discuss solver structure and improved face-area weights. Author page inspected.

### M10 Low Mach phase-change reference implementation

Basilisk, author-maintained phase-change documentation and low Mach extension.

[Phase-change notes](https://basilisk.fr/sandbox/ecipriano/doc/phasechange) · [Low Mach source](https://basilisk.fr/sandbox/ecipriano/src/navier-stokes/low-mach.h).

Useful primary implementation reference showing interfacial expansion and variable-density source terms. Sandbox code is not an official guarantee of every combined model. Pin a revision, inspect licences and verify independently before reuse. Source/documentation inspected.

### M11 Dispersed droplet heat and mass transfer

B. Abramzon and W. A. Sirignano (1989), *Droplet vaporization model for spray combustion calculations*, International Journal of Heat and Mass Transfer 32, 1605–1618.

[Primary publication](https://doi.org/10.1016/0017-9310(89)90043-4).

Candidate transfer closure for applicable approximately spherical dilute droplets, requiring a water-specific applicability check. It is not a complete condensation nucleation model. Publication identity and abstract-level scope checked.

### M12 Fog parcel population model

S. Shima, K. Kusano, A. Kawano, T. Sugiyama and S. Kawahara (2009), *The super-droplet method for the numerical simulation of clouds and precipitation: a particle-based and probabilistic microphysics model coupled with a non-hydrostatic model*, Quarterly Journal of the Royal Meteorological Society 135, 1307–1320.

[DOI](https://doi.org/10.1002/qj.441) · [Author preprint](https://arxiv.org/abs/physics/0701103).

Selected representation for multiplicity-based condensation/evaporation and statistical microphysics. Transferring a warm-cloud method to a beaker plume requires appropriate timescales, aerosol inputs, heat/mass coupling and validation. Primary abstract inspected.

### M13 Compatible mass and momentum

S. Pal, D. Fuster and S. Zaleski (2021 preprint), *A novel momentum-conserving, mass-momentum consistent method for interfacial flows involving large density contrasts*.

[Author preprint](https://arxiv.org/abs/2101.04142).

Selected reference for conservative staggered 3D mass/momentum transport; it uses a finer mass-advection grid. Its cost must be budgeted, and combination with phase-change sources and embedded solids requires a derivation. Primary preprint abstract inspected.

### M14 Alternative consistent phase field

Z. Huang, G. Lin and A. M. Ardekani, *A consistent and conservative Phase-Field model for thermo-gas-liquid-solid flows including liquid-solid phase change*, Journal of Computational Physics 449, 110795.

[DOI](https://doi.org/10.1016/j.jcp.2021.110795) · [DOE record](https://www.osti.gov/pages/biblio/1829949).

Relevant alternative rather than selected baseline. Conservation and phase-volume consistency are central, but its solid drag formulation is not a complete freely moving rigid ice and liquid/vapour phase-change apparatus. Primary abstract inspected.

## 5 Benchmarks and convergence sources

### B01 Dam-break experiment

L. Lobovský, E. Botia-Vera, F. Castellana, J. Mas-Soler and A. Souto-Iglesias (2014), *Experimental investigation of dynamic pressure loads during dam break*, Journal of Fluids and Structures 48, 407–434.

[DOI](https://doi.org/10.1016/j.jfluidstructs.2014.03.009) · [Author preprint](https://arxiv.org/abs/1308.0115) · [Institutional record](https://oa.upm.es/25973/).

Use the original geometry, sensor definitions and uncertainty. Publication and data availability checked; exact digitisation/extraction is an implementation task.

### B02 Rising bubble benchmark

S. Hysing and co-authors (2009), *Quantitative benchmark computations of two-dimensional bubble dynamics*, International Journal for Numerical Methods in Fluids 60, 1259–1288.

[DOI](https://doi.org/10.1002/fld.1934) · [Benchmark owner](https://wwwold.mathematik.tu-dortmund.de/~featflow/en/benchmarks/cfdbenchmarking/bubble.html).

Supports canonical interface verification. It is explicitly 2D; an extruded 3D implementation can reproduce its symmetry, but cannot establish genuinely 3D bubble accuracy by that test alone. Primary paper/owner records checked.

### B03 Natural convection benchmark

G. de Vahl Davis (1983), *Natural convection of air in a square cavity: A bench mark numerical solution*, International Journal for Numerical Methods in Fluids 3, 249–264.

[Primary publication](https://doi.org/10.1002/fld.1650030305).

Use the matched dimensionality and Boussinesq benchmark conditions. It is not an empirical water beaker dataset or validation of wide-range water density. Primary abstract inspected.

### B04 Grid convergence

NASA Glenn, *Examining Spatial Grid Convergence*, NPARC verification and validation tutorial.

[Official tutorial](https://www.grc.nasa.gov/www/wind/valid/tutorial/spatconv.html).

Use observed convergence and GCI only when the mesh sequence meets the method's assumptions. A non-monotone interface-breakup sequence needs a more appropriate uncertainty treatment. Official tutorial inspected.

## 6 Optical and Unity references

### O01 Water optical constants

G. M. Hale and M. R. Querry (1973), *Optical Constants of Water in the 200-nm to 200-micrometre Wavelength Region*, Applied Optics 12, 555–563.

[Primary publication](https://doi.org/10.1364/AO.12.000555).

Supports water spectral refraction/absorption at the source conditions. Rendering interpretation does not establish thermodynamic phase fidelity. Primary publication record checked.

### O02 Ice optical constants

S. G. Warren and R. E. Brandt (2008), *Optical constants of ice from the ultraviolet to the microwave: A revised compilation*, Journal of Geophysical Research 113, D14220.

[DOI](https://doi.org/10.1029/2007JD009744) · [Author data page](https://www.atmos.washington.edu/ice_optical_constants/).

Use appropriate wavelength and temperature/ice preparation conditions. Optical data do not prescribe bubble/impurity content of every ice sample. Primary paper/data records checked.

### U01 Unity compute shaders

Unity 2022.3 manual, *Compute shaders*.

[Official manual](https://docs.unity3d.com/2022.3/Documentation/Manual/class-ComputeShader.html).

Compute shaders execute GPU programs outside the normal rendering pipeline. Check actual device/API capabilities, resource binding and dispatch limits. Official documentation inspected.

### U02 Unity asynchronous readback

Unity 2022.3 scripting API, *AsyncGPUReadback*.

[Official API](https://docs.unity3d.com/2022.3/Documentation/ScriptReference/Rendering.AsyncGPUReadback.html).

Use to obtain diagnostics without mandatory synchronous full-field reads. Capability and error handling remain required. Official documentation inspected.

### D16 Humid mixture transport and comparison data

A. Melling, S. Noppenberger, M. Still and H. Venzke (1997), *Interpolation Correlations for Fluid Properties of Humid Air in the Temperature Range 100 C to 200 C*, Journal of Physical and Chemical Reference Data 26, 1111.

[Primary paper hosted by NIST](https://srd.nist.gov/jpcrdreprint/1.555998.pdf) · [DOI](https://doi.org/10.1063/1.555998).

Full primary text inspected. Viscosity Eq 13/Table 8 fits the reported Studnikow data in the 100-150 C interval; extension to 200 C is discussed explicitly. Conductivity Eqs 14-19 use a Mason-Saxena form with the paper's selected g=0.80 for polar/nonpolar gas mixtures. The model is not an exact universal reference. Tables 5-6 reproduce other low-temperature viscosity measurements and Table 9 gives conductivity-ratio data used to select the interaction factor.

The S02 continuation retains Eq 13's composition curvature while rebasing the endpoints to modern IAPWS water and Lemmon/Jacobsen air transport. This is a documented project adaptation, including a checked low-temperature extension; it is not represented as the unchanged published correlation. Production transport is restricted to Y_water<=0.10 or pure vapour. Wider reference predictions are retained to expose disagreement, not certify steam-rich transport. No third-party source code or PDF is redistributed.

### D17 Modern moist-air reference comparisons

ASHRAE (2025), *Fundamentals*, Chapter 1, section 11, Table 5.

[Official table](https://handbook.ashrae.org/handbooks/F25/SI/F25_Ch01/F25_Ch01_si.aspx).

The table provides calculated dry/saturated moist-air transport at 101325 Pa, based on LibHuAirProp/RP-1767. It is an independent modern calculated reference, not a new experimental dataset. Comparisons at 10-90 C expose increasing disagreement of the historical transport shape at high humidity. These differences are recorded separately from numerical assertions; no tolerance is increased to turn them into passing numerical checks.

### D18 Broad pure-ice conductivity regression

Y.-C. Yen (1981), *Review of Thermal Properties of Snow, Ice, and Sea Ice*, CRREL Report 81-10, pp 15-16, Eq 33, Figure 14 and Table 3.

[USACE record](https://usace.contentdm.oclc.org/digital/collection/p266001coll1/id/7366) · [Full primary report scan](https://upload.wikimedia.org/wikipedia/commons/8/80/Review_of_thermal_properties_of_snow%2C_ice%2C_and_sea_ice_-_USACE-p266001coll1-7366.pdf).

The report recommends k=9.828 exp(-0.0057 T_K) W/(m K) as a practical broad regression of the collected pure polycrystalline ice data. It explicitly discusses scatter from purity, preparation, heat-loss correction and reproducibility. The regression correlation coefficient 0.9313 is not an uncertainty bound. S02 requires explicit `Yen1981PurePolycrystalline` selection, restricts its use to 200-273.15 K, and does not switch or blend it with the Bonales preparation. This supplies a sourced model through near melting, not independent validation for an arbitrary physical sample. No pressure, bubbles or anisotropy correction is asserted.

### D19 Wider water-air diffusion comparison

T. R. Marrero and E. A. Mason (1972), *Gaseous Diffusion Coefficients*, Journal of Physical and Chemical Reference Data 1, 3.

[Full primary paper](https://srd.nist.gov/jpcrdreprint/1.3253094.pdf) · [DOI](https://doi.org/10.1063/1.3253094).

Eq 4.3-2 and Table 13 list air-H2O pD=A T^s: A=1.87e-6 atm cm2/(s K^s), s=2.072 over 282-450 K; A=2.75e-5, s=1.632 over 450-1070 K. S02 retains the equimolar correlation as a separately named research comparison over 282-473 K. It disagrees with the ASHRAE calculated diffusion table by about 14% at 190 Celsius and is not wired to production. The production extension explicitly selects ASHRAE 2025 Chapter 1 Table 4 (Mason-Monchick calculated values), using log-log interpolation over 203.15-473 K, inverse-pressure scaling over 80-101.325 kPa, and Y<=0.02. The Massman default is retained; there is no automatic switch between correlations. This widens temperature coverage, not composition-resolved steam-rich diffusion. The two source fits are not forced to join by changing coefficients. Table-node checks are numerical regression, not independent physical validation. Differences from the older equimolar fit are recorded as informational observations, not passing assertions. The 296-1640 K exponential correlation in Table 13 belongs to H2O-CO2 and is not used for air.

### D20 Modern moist-air transport model acquisition

S. Herrmann et al. (2021), *Transport properties of real moist air, dry air, steam, and water*, Science and Technology for the Built Environment 27, 393-401, doi:10.1080/23744731.2021.1877519.

[Author's accessible first page](https://thermofluidprop.com/fileadmin/Dateien/Publikationen/Veroeffentlichungen/Herrmann_Paper_STBE_2021_393_published_Page_2.pdf) · [Author's supplementary calculated tables](https://thermofluidprop.com/fileadmin/Dateien/KCE-Dateien/Tables_Download_Paper_Herrmann_Moist_Air_Transport_Properties_Rev.pdf).

The full publisher article returned HTTP 403. A public text rendering of the original paper was subsequently inspected through a mirrored copy (Scribd document 1075965237). A separately named research candidate implements the low-density matrix limit and Eq32 composition-dependent interaction length, using A*=B*=1.1, unit contact factors and no bulk or critical contribution. It is not the complete finite-density RP-1767 formulation and is not wired into production. Its calculated-table comparisons are informational observations. Exact collision-integral choices and the switch-over-density/contact treatment still require acquisition and verification from the full report before replacing the guarded production provider. No third-party code, software or paper is redistributed.

## 7 Work still needed before claiming physical completion

Extract and independently verify the full coefficients and detailed numerical stencils from the selected pinned sources. Acquire missing glass/tripod/contact/wetting/heat-input data. Derive and review the coupled discrete balances rather than assuming independent published components can simply be concatenated.

These are explicit implementation and data-acquisition steps. No missing coefficient, material datum, test run or apparatus measurement has been fabricated in this documentation.
