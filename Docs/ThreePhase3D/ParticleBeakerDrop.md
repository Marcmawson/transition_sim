# Interactive GPU particle beaker

Open `Assets/Scenes/ThreePhase3D/ParticleBeakerDrop.unity` and enter Play Mode. Use **Run** for inspection or **Measure drop + slosh** to reset the water and time a 0.4 s simulated release. Copy the `GPU drop:` line from Unity's Console. The test reports wall-clock frame rate and simulation throughput, then requests one asynchronous position snapshot if the device supports it.

The 9 mL initial cube starts from rest 10 mm above the floor before spawn jitter. The existing GPU particle solver advances in 0.5 ms substeps, eight per rendered frame. The particle renderer consumes the solver's position and velocity buffers directly. The standard test performs no per-frame CPU readback. The snapshot reports particle count, nonfinite positions, particles outside the cavity below the rim, lowest vertical position and radial extent. A particle below the rim and outside the cavity may be a real escape over the open rim, so the snapshot count requires visual context.

The interactive acceptance target is at least 30 rendered frames per wall-clock second for the measured drop. This is separate from simulation throughput. Inspect the fall, floor impact and outward spread in the Game view. The snapshot and frame timing do not establish agreement with a physical experiment.

This scene uses a cylindrical inner collision boundary and a separate exterior box. It is an interactive liquid test with provisional SPH parameters and collision damping. It does not yet model glass exterior contact, heat transfer, ice, vapour, or phase change. The earlier cut-cell scenes remain available for their existing tests.
