# S11 rigid ice body

Run `dotnet run --project Tools/ThreePhase3D/IceBodyChecks.csproj` from the repository root.

The constant-mass rigid state stores world linear momentum and angular momentum about the centre of mass. Its frame follows the principal axes. Angular velocity uses the inverse inertia tensor in the current orientation. The body supplies point velocity for a moving fluid boundary and accepts a force and torque or an impulse at a world point. Updates return a new state so a coupled solver can reject a trial before accepting it.

The CPU verification passed for a homogeneous rectangular body. It checks analytic mass and inertia, constant-force translation, principal-axis rotation under constant torque, off-centre impulse, point velocity, rotated inertia and unforced principal-axis motion over 1000 steps. The orientation integrator is explicit and tested here in principal-axis motion.

This supplies the rigid state for V24. Fluid traction, pressure/body iteration, flotation and grid-offset verification remain open.
