# S09 GPU liquid buoyancy check

Run **Physics Sim → Three Phase 3D → Run S09 GPU buoyancy check** in Unity. A closed, fully liquid 4 × 4 × 2 cavity exercises the GPU enthalpy recovery, a vertical buoyancy predictor and the existing GPU pressure projection. The Console reports the projected vertical velocity in the test column for uniform temperature, heating from 274 K to 277.125 K, and heating from 277.125 K to 283 K. The check requires zero velocity for the uniform case, downward velocity in the first heated case and upward velocity in the second.

The predictor uses the nonlinear atmospheric liquid-density table. In this small-density-variation benchmark, the projection and mass transport use a fixed density, and the local density difference contributes only to the body force. Each internal vertical face uses the mean of the adjoining liquid densities. This benchmark tests the initial response, not steady cavity circulation, free-surface motion or pressure/volume coupling across the full water range. The current GPU pressure API takes a host array, so the tentative velocities are read back between the two GPU kernels.

The runtime and Editor components were compiled separately against Unity 2022.3.46f1. The new GPU check still needs to run in Unity. The thermal beaker scene remains stationary pending coupling of the flow and thermal fields.

## V20 reference and energy check

The same Unity menu now compares the sampled projected velocity with a CPU projection driven by direct FP64 IAPWS-95 density evaluations at 101325 Pa. The direct evaluation bypasses the GPU table and its interpolation. For each case, it also advects the initialized liquid enthalpy through one accepted GPU projected step in the closed cavity and checks the total inventory. The pass limits are 2e-7 m/s absolute velocity error and 1e-8 J absolute enthalpy drift.

The CPU reference predicts -2.57977117e-5 m/s below the density maximum and +8.289377e-5 m/s above it. The previous GPU velocity logs differ by less than 9e-9 m/s. The updated runtime compiled against Unity 2022.3.46f1 assemblies. Run the menu again to capture the new V20 energy and reference result.
