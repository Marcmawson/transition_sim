# S04 embedded-wall viscosity

S04 imposes zero velocity at stationary embedded solid walls. The momentum predictor uses the same solid surfaces as the S03 pressure grid. For each staggered component it clips a second grid shifted by half a cell along that component. Each resulting velocity control volume supplies its accessible volume, fluid centroid, shared apertures and exposed wall patches.

For uniform kinematic viscosity, the rate through an active shared aperture is `nu × area / (dual fluid volume × projected centroid separation)`. Both rows use the same aperture and centroid separation, giving equal and opposite integrated viscous exchange. The stationary wall supplies a sink with rate `nu × exposed wall area / (dual fluid volume × normal distance)`. Wall patches and masked velocities close the solid. A blocked dual face also excludes an upwind sample across the wall. The maximum assembled diagonal participates in the explicit timestep bound. Unresolved dual geometry raises an error.

The fluid centroids are computed from signed tetrahedral first moments of clipped solid volume relative to each dual cell centre. On curved and oblique cuts the shared-face operator uses a two-point projected distance. The verification scope is constant properties and fixed walls with resolved geometry. The momentum predictor uses the existing advective discretisation on open portions of the staggered grid.

## Evidence

- Off-grid planar wall: analytical linear shear remains unchanged at the wall-adjacent probe. The wall rate is 0.274348408 s⁻¹ against the analytical 0.274348423 s⁻¹. A tangential plug decays by the predicted amount.
- Curved channel: axial plug speed remains unchanged at the core after one step and decreases in norm as the wall damps the outer flow.
- Synthetic thick-walled beaker: dual geometry and viscous stencils assemble and the unforced fluid stays at rest.

The managed reference and standalone software Vulkan paths each pass 106 checks. The curved-channel check tests one diffusion step. The beaker case is a geometry and rest check. [Verification logs](../../ValidationRuns/s04-embedded-viscosity-20261111)

To validate the new shader on the target device, use **Physics Sim > Three Phase 3D > Run S04 GPU flow checks** in Unity with Play mode stopped. The prior Unity run verified the earlier single-phase checkpoint. The embedded-wall checks were added after that run.
