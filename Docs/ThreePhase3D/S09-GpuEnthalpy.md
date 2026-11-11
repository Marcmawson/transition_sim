# S09 GPU liquid temperature recovery

In Unity select **Physics Sim → Three Phase 3D → Run S09 GPU enthalpy check**. The check transports a warm pulse through the GPU cut-cell scalar path and recovers liquid temperature directly from the resulting GPU volume and enthalpy planes. It compares the recovered temperatures against the CPU isobaric reference and checks the transported energy ledger. The Console reports the downstream temperature, maximum temperature error and ledger residual.

The recovery kernel reads compensated volume and enthalpy pairs, derives specific enthalpy using the fixed transport density, and inverts the same atmospheric liquid table as the CPU. It exposes temperature and specific enthalpy on the GPU with separate status values for liquid, dry and invalid groups. Values beyond the table range are invalid, with a small endpoint tolerance for FP32 arithmetic.

The C# integration compiled against the Unity 2022.3.46f1 assemblies. The verification menu still needs to be run on the GPU. This increment supplies recovery after accepted scalar transport. Temperature-driven buoyancy, pressure/volume coupling, glass-to-moving-water heat exchange and a convecting Unity scene remain open.
