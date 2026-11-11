# S03 grid and embedded geometry checkpoint

11 November 2026. Source baseline: `b6dc56f0bb7c00695195827dbb212c5427bc8258`.

S03 supplies the grid, geometric metrics, conservative tiny-cell grouping and GPU resource/check infrastructure. Its GPU execution gate remains pending. It does not integrate velocity, pressure, heat or phase change; those operators begin in S04 and subsequent stages. The user authorised progression with the previously documented S02 transport limits deferred; this is not a claim that unrestricted S02 transport is complete.

## Implemented contract

| Component | Supplied behaviour |
| --- | --- |
| `Core/Grid3D.cs` | Immutable SI layout, orthonormal rigid frame, x-fastest cell and distinct x/y/z-face storage, explicit ghost widths, checked indexing and integer overflow guards |
| `Geometry/CutCellGeometry.cs` | Separate fixed-solid, ice and accessible-fluid volumes/fractions; one shared aperture per grid face; outward normals, clipped triangle patches, contact areas and geometric interface distances |
| `Geometry/PolyhedronClip.cs` | Oriented closed-mesh signed tetrahedral decomposition, convex volume/face clipping and surface-patch clipping |
| `Geometry/TinyCellMergePlan.cs` | Immutable agglomeration map and aggregate volume/inventory operation; no geometry edits or tiny-volume deletion; isolated small components fail explicitly |
| `Core/GridGpuPlan.cs` | Checked byte budget, count/stride specifications and per-kernel required-buffer registry |
| `Rendering/GridGpuResources.cs` | Scoped GPU allocation, explicit initialization/upload/binding, tail canaries, two-dimensional dispatch tiling and idempotent release, including allocation-failure cleanup |
| `GridGeometryChecks.compute` | Separate bounded clearing and metric-verification kernels; independent thread ownership of each result |
| Editor and preview commands | Reproducible CPU/GPU checks and a generated scene displaying the shared beaker/ice geometry and cut cells |

All geometry snapshots and group maps expose read-only views. Changing a body pose or geometry requires a new snapshot; moving-body evolution is later work. The beaker render mesh and MeshCollider come from `ApparatusSurface.Beaker`, also supplied directly to the cut-cell builder. The ice render mesh, MeshCollider and embedded geometry likewise use the same triangle surface. Tripod rendering remains a display preview; tripod material/contact conduction and a combined support embedding are not claimed here.

## Geometry and accounting

For one common reference point, every oriented surface triangle forms a signed tetrahedron. The fixed or ice volume in a cell is the sum of each clipped tetrahedron volume multiplied by its orientation sign. This handles the cavity of a watertight hollow beaker without sampling only cell centres. Integration is exact for the supplied polygonal surface up to numerical roundoff; polygonal cylinders and spheres still have tessellation error against smooth analytical shapes.

Face areas use signed cross-sections of the same tetrahedra. Both one-sided sections are assembled across fixed and ice geometry before selecting the blocked face area, so aligned solid contact blocks the face once. The displacement used to distinguish boundary ownership is 1e-9 times the relevant cell spacing. Fixed/ice volume overlap is rejected. Solid–solid contact is clipped separately and excluded from the exposed fluid interface area; both sides remain in the patch records for later contact/thermal operators.

Each physical cell satisfies `V_fluid = V_cell - V_fixed - V_ice`. Ghost cells own no physical volume or inventory. There is no density rescaling. Negative/out-of-bound roundoff and uncut-cell endpoint snapping are bounded and recorded as absolute volume/area adjustments. Positive cut-cell volumes are retained, however small. Cells with no reconstructed boundary may be snapped only within 1e-12 of a full-cell volume; this removes numerical residues in mathematically full/empty cells.

Interface patch normals are outward from the named solid. `CellCentreNormalDistanceM` is the absolute normal projection from the cell centre to the clipped patch centroid. This is a declared geometric distance, not an already validated thermal-conductance or phase-centroid stencil. Opposing interfaces remain separate patches even when their area vectors cancel in a cell.

The builder checks finite coordinates, indexable grid-space coordinates, manifold edge multiplicity, consistent orientation and positive total volume. Arbitrary self-intersecting imported meshes are outside its declared input contract; the checks are not a general CAD repair/intersection validator.

## Tiny cells and thin walls

The default tiny-cell grouping threshold is 5% of a full cell. Cells/groups join across their largest accessible shared face. Their original metrics remain unchanged; extensive inventories are summed into the group. A subsequent finite-volume operator must cancel internal group faces and use the aggregate external flux and volume. That operator is S04 work, not implemented flow here.

An isolated single-cell pocket or a connected small component that cannot reach the grouping threshold is rejected, with its geometric volume retained. Ghost inventories are rejected. The tests include both cases and verify aggregate volume and extensive-mass conservation.

Geometry can detect a wall thinner than one cell without establishing that flow or through-thickness conduction is resolved. `RequireFeatureResolution` applies a declared minimum of three cells across the feature, using the largest cell spacing. This is an engineering preflight floor, not proof of thermal accuracy. No homogenised-wall law is substituted. The coarse interactive preview intentionally fails this physical-use floor and therefore has no field evolution. Actual heating still requires the S08 convergence tests and appropriate solid material data.

## CPU results

**120 numerical comparisons pass**, zero failures, over 13 recorded shape/grid cases plus indexing, aperture, contact, thin-wall, grouping, immutability and invalid-input checks. The cases cover three non-cubic grid resolutions, translated/rotated registrations, boxes, cylinders, spheres and hollow beakers. The actual preview's glass–ice contact is checked separately. All **25 foundation groups pass**, and the ThreePhase3D C# sources compile against the pinned **Unity 2022.3.46f1** assemblies. Assembly compilation is not an Editor import or shader compilation.

[CPU results and case records](../../ValidationRuns/s03-grid-geometry-20261111/geometry-results.json), [source fingerprints](../../ValidationRuns/s03-grid-geometry-20261111/source-hashes.txt), [runner identity](../../ValidationRuns/s03-grid-geometry-20261111/runner.txt), and [file seals](../../ValidationRuns/s03-grid-geometry-20261111/files.sha256).

Preregistered numerical targets in the runner are mesh-to-cut volume/area agreement within 1e-8 of the analytical scale, cell partition closure within 1e-8 of cell volume, and oriented face/interface closure within 2e-6 of a face-area scale. The finest cylinder/sphere smooth-geometry volume and sphere area discrepancies must be below 1%, with decreasing volume error under tessellation refinement. The report retains analytical, mesh and integrated values separately.

Grid dimensions are 4/8/16 in x, with y=x+1 and z=x+2. Angular segments are 12/24/48; sphere latitude counts are 6/12/24. Because mesh integration already matches the mesh, the observed smooth-shape convergence is tessellation convergence. It is not a measured convergence order of a future flow solver. Wall thickness and feature-to-cell ratios are retained for the beaker cases.

## GPU resource and execution gate

The resource plan contains 17 buffers. Scalars/uints have stride 4; interface area vectors and results are float4 with stride 16. Cell and face allocations include ghost regions and one extra tail guard each. Planned/owned bytes are `80*C + 8*(Fx+Fy+Fz) + 104`, where the counts include ghosts and exclude the tail guards. This records the owned buffer payload, not total process/driver VRAM usage. The default budget is 512 MiB and is checked before allocation.

The GPU command checks six configurations: box, cylinder, sphere, hollow beaker, separate fixed/ice bodies and aligned solid contact. It poisons and clears all logical cell/face fields, validates physical and ghost indexing, checks partition/aperture/oriented-closure metrics against the CPU data, checks tail guards, injects a nonfinite metric, and checks disposal. It records actual buffer counts/strides/bytes and host elapsed time for each dispatch plus synchronous readback. Those times are not isolated GPU kernel timings or a real-time performance claim; use Unity's GPU Profiler for device-only timing.

The local batch Editor attempt stopped before import: licensing IPC was refused, then timed out after 60 seconds and Unity reported exit code 199. [Exact startup log](../../ValidationRuns/s03-grid-geometry-20261111/editor-startup.log). No S03 GPU execution or shader compilation is claimed from that attempt.

After pulling main, use these **top menu bar** commands in the licensed Unity Editor:

1. **Physics Sim > Three Phase 3D > Run S03 CPU grid and geometry checks**
2. **Physics Sim > Three Phase 3D > Run S03 GPU resource and geometry checks**
3. **Physics Sim > Three Phase 3D > Create S03 grid geometry preview**

The GPU Console line must report `executed=True; passed=True`. Save the entire `ValidationRuns/local/s03-gpu-*` directory. The preview command creates a new scene through Unity's normal save-current-scene prompt; it does not overwrite the existing Beaker3D asset. Select the preview root and enable Scene gizmos: amber identifies glass cut cells and cyan identifies ice cut cells. Rebuild after changing inputs; stale geometry is identified explicitly. GPU checks do not require this preview scene to be open.

Standalone CPU reproduction:

```powershell
powershell -ExecutionPolicy Bypass -File Tools/ThreePhase3D/RunGeometryChecks.ps1
powershell -ExecutionPolicy Bypass -File Tools/ThreePhase3D/RunChecks.ps1 -CompileUnitySources
```

Primary Unity API references: [2022.3 structured-buffer contract](https://docs.unity.cn/2022.3/Documentation/ScriptReference/ComputeBufferType.Structured.html), [thread-group sizes](https://docs.unity3d.com/2022.3/Documentation/ScriptReference/ComputeShader.GetKernelThreadGroupSizes.html) and [dispatch](https://docs.unity3d.com/2022.3/Documentation/ScriptReference/ComputeShader.Dispatch.html). The geometric construction is original project code; no third-party implementation or dependency is added.
