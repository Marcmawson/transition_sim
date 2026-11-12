# S11 rigid ice body

Run `dotnet run --project Tools/ThreePhase3D/IceBodyChecks.csproj` from the repository root.

The constant-mass rigid state stores world linear momentum and angular momentum about the centre of mass. Its frame follows the principal axes. Angular velocity uses the inverse inertia tensor in the current orientation. The body supplies point velocity for a moving fluid boundary and accepts a force and torque or an impulse at a world point. Updates return a new state so a coupled solver can reject a trial before accepting it.

The CPU verification passed for a homogeneous rectangular body. It checks analytic mass and inertia, constant-force translation, principal-axis rotation under constant torque, off-centre impulse, point velocity, rotated inertia and unforced principal-axis motion over 1000 steps. The orientation integrator is explicit and tested here in principal-axis motion.

This supplies the rigid state for V24. The pressure-load verification below supplies the next piece. Coupled flotation remains open.

## Cut-cell pressure load

Run `dotnet run --project Tools/ThreePhase3D/IcePressureChecks.csproj`. The pressure load integrates fluid pressure on the cut ice surface and computes world force and torque about the centre of mass. A projection overload reads group pressures and accepts an optional hydrostatic background. The body update applies gravity once alongside this traction. Three-point triangle quadrature integrates affine pressure and its torque exactly on each planar polygon. Partially blocked solid-contact polygons require exposed geometry before they can be integrated.

The CPU test uses a rotated, fully submerged box on two grid offsets. It recovered 0.23544 N of hydrostatic lift and zero hydrostatic torque within the stated tolerances. A separate prescribed top-face pressure gradient produced -0.000128 N m of torque. The rigid-body step reproduced the analytic initial upward velocity under pressure and gravity. Pressure is prescribed in this fixture. The moving-boundary pressure iteration, fluid reaction, free-surface flotation and GPU execution remain open for V24.

## Moving-wall pressure projection

The pressure projection can now accept one outward boundary volume flux per pressure group. The ice supplies it by integrating rigid-body normal velocity over its cut-cell surface. Both projection backends include that flux in the pressure equation and divergence diagnostics. A closed pressure component rejects an incompatible net imposed flux.

The CPU fixture prescribes combined translation and rotation in the existing geometry and projects fluid initially at rest. At both grid offsets the pressure reaction opposes the prescribed motion. The body-side pressure power matches the sum of pressure times outward boundary flux to within 1e-10 W. The GPU solver C# integration compiled against Unity 2022.3.46f1. The next section verifies the coupled step on CPU. GPU execution and moving geometry remain open.

## Coupled pressure and body timestep

The fixed-geometry step iterates the ice momentum and the fluid projection using the same trial normal wall flux. Each candidate body momentum uses the original momentum plus the current pressure and gravity impulse. Under-relaxation updates the trial. The step accepts when the translational velocity mismatch and the angular mismatch scaled by the inertia length are both within 1e-8 m/s. It returns the accepted body and fluid states together and throws on nonconvergence.

The CPU fixture started a rotated submerged box and fluid at rest. On both grid offsets the solve converged in 25 iterations. The accepted upward speeds were 0.00596879 and 0.00602221 m/s, below the pressure-only rise speed of 0.00887939 m/s. Both momentum updates matched the final pressure and gravity impulses. The pressure projection backend is injected, and this verification used the CPU backend. Moving cut-cell geometry and conservative remapping between poses remain open, as does free-surface flotation.
