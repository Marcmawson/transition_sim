using Seb.Fluid2D.Rendering;
using Seb.Helpers;
using UnityEngine;
using Unity.Mathematics;
using UnityEngine.Serialization;

namespace Seb.Fluid2D.Simulation
{
	public class FluidSim2D : MonoBehaviour
	{
		public event System.Action SimulationStepCompleted;

		[Header("Simulation Settings")]
		public float timeScale = 1;
		public float maxTimestepFPS = 60; // if time-step dips lower than this fps, simulation will run slower (set to 0 to disable)
		public int iterationsPerFrame;
		public float gravity;
		[Range(0, 1)] public float collisionDamping = 0.95f;
		public float smoothingRadius = 2;
		public float targetDensity;
		public float pressureMultiplier;
		public float nearPressureMultiplier;
		public float viscosityStrength;
		public Vector2 boundsSize;
		public Vector2 boundsCentre;
		public bool obstacleEnabled;
		public Vector2 obstacleSize;
		public Vector2 obstacleCentre;

		[Header("Thermal Loop")]
		public ThermalLoopLayout2D thermalLoop;

		[Header("Tank Rotation")]
		[Range(-180, 180)] public float tankRotationDegrees;
		[Min(0)] public float tankRotationStepDegrees = 5;
		public bool tankRotationKeyboardInput = true;

		[Header("Interaction Settings")]
		public float interactionRadius;

		public float interactionStrength;

		[Header("References")]
		public ComputeShader compute;

		public Spawner2D spawner2D;

		// Buffers
		public ComputeBuffer positionBuffer { get; private set; }
		public ComputeBuffer velocityBuffer { get; private set; }
		public ComputeBuffer densityBuffer { get; private set; }
		public ComputeBuffer thermalEnergyBuffer { get; private set; }
		public ComputeBuffer particlePhaseBuffer { get; private set; }

		ComputeBuffer sortTarget_Position;
		ComputeBuffer sortTarget_PredicitedPosition;
		ComputeBuffer sortTarget_Velocity;

		ComputeBuffer predictedPositionBuffer;
		ComputeBuffer sortTarget_ThermalEnergy;
		ComputeBuffer sortTarget_ParticlePhase;
		ComputeBuffer thermalEnergyNextBuffer;
		ComputeBuffer wallTemperatureBuffer;
		ComputeBuffer wallTemperatureScratchBuffer;
		bool wallSpatialReady;
		SpatialHash spatialHash;

		// Kernel IDs
		const int externalForcesKernel = 0;
		const int spatialHashKernel = 1;
		const int reorderKernel = 2;
		const int copybackKernel = 3;
		const int densityKernel = 4;
		const int pressureKernel = 5;
		const int viscosityKernel = 6;
		const int updatePositionKernel = 7;
		const int phaseChangeKernel = 8;
		const int diffuseThermalEnergyKernel = 9;
		const int applyThermalEnergyKernel = 10;
		const int updateWallTemperatureKernel = 11;
		const int applyWallTemperatureKernel = 12;

		// State
		bool isPaused;
		Spawner2D.ParticleSpawnData spawnData;
		bool pauseNextFrame;

		public int numParticles { get; private set; }


		void Start()
		{
			Debug.Log("Controls: B = burner, [ / ] = adjust air collar, Space = Play/Pause, R = Reset, LMB = Attract, RMB = Repel");
			if (thermalLoop == null) thermalLoop = GetComponent<ThermalLoopLayout2D>();

			Init();
		}

		void Init()
		{
			float deltaTime = 1 / 60f;
			Time.fixedDeltaTime = deltaTime;

			spawnData = spawner2D.GetSpawnData();
			numParticles = spawnData.positions.Length;
			spatialHash = new SpatialHash(numParticles);

			// Create buffers
			positionBuffer = ComputeHelper.CreateStructuredBuffer<float2>(numParticles);
			predictedPositionBuffer = ComputeHelper.CreateStructuredBuffer<float2>(numParticles);
			velocityBuffer = ComputeHelper.CreateStructuredBuffer<float2>(numParticles);
			densityBuffer = ComputeHelper.CreateStructuredBuffer<float2>(numParticles);
			thermalEnergyBuffer = ComputeHelper.CreateStructuredBuffer<float>(numParticles);
			particlePhaseBuffer = ComputeHelper.CreateStructuredBuffer<uint>(numParticles);
			thermalEnergyNextBuffer = ComputeHelper.CreateStructuredBuffer<float>(numParticles);
			int wallSegments = Mathf.Max(2, Mathf.CeilToInt(boundsSize.x / Mathf.Max(smoothingRadius, 0.01f)) + 1);
			wallTemperatureBuffer = ComputeHelper.CreateStructuredBuffer<float>(wallSegments);
			wallTemperatureScratchBuffer = ComputeHelper.CreateStructuredBuffer<float>(wallSegments);
			compute.SetInt("wallSegmentCount", wallSegments);
			compute.SetFloat("wallSegmentWidth", boundsSize.x / (wallSegments - 1));

			sortTarget_Position = ComputeHelper.CreateStructuredBuffer<float2>(numParticles);
			sortTarget_PredicitedPosition = ComputeHelper.CreateStructuredBuffer<float2>(numParticles);
			sortTarget_Velocity = ComputeHelper.CreateStructuredBuffer<float2>(numParticles);
			sortTarget_ThermalEnergy = ComputeHelper.CreateStructuredBuffer<float>(numParticles);
			sortTarget_ParticlePhase = ComputeHelper.CreateStructuredBuffer<uint>(numParticles);

			// Set buffer data
			SetInitialBufferData(spawnData);

			// Init compute
			ComputeHelper.SetBuffer(compute, positionBuffer, "Positions", externalForcesKernel, updatePositionKernel, copybackKernel);
			ComputeHelper.SetBuffer(compute, predictedPositionBuffer, "PredictedPositions", externalForcesKernel, spatialHashKernel, densityKernel, pressureKernel, viscosityKernel, copybackKernel, phaseChangeKernel, diffuseThermalEnergyKernel);
			ComputeHelper.SetBuffer(compute, velocityBuffer, "Velocities", externalForcesKernel, pressureKernel, viscosityKernel, updatePositionKernel, copybackKernel, phaseChangeKernel);
			ComputeHelper.SetBuffer(compute, densityBuffer, "Densities", externalForcesKernel, densityKernel, pressureKernel, viscosityKernel, phaseChangeKernel, updatePositionKernel);
			ComputeHelper.SetBuffer(compute, thermalEnergyBuffer, "ThermalEnergy", externalForcesKernel, copybackKernel, phaseChangeKernel, diffuseThermalEnergyKernel, applyThermalEnergyKernel);
			ComputeHelper.SetBuffer(compute, particlePhaseBuffer, "ParticlePhase", externalForcesKernel, densityKernel, pressureKernel, viscosityKernel, updatePositionKernel, copybackKernel, phaseChangeKernel, diffuseThermalEnergyKernel);
			ComputeHelper.SetBuffer(compute, thermalEnergyNextBuffer, "ThermalEnergyNext", diffuseThermalEnergyKernel, applyThermalEnergyKernel);
			ComputeHelper.SetBuffer(compute, wallTemperatureBuffer, "WallTemperatureRead", updateWallTemperatureKernel);
			ComputeHelper.SetBuffer(compute, wallTemperatureScratchBuffer, "WallTemperatureNext", updateWallTemperatureKernel, applyWallTemperatureKernel);
			ComputeHelper.SetBuffer(compute, wallTemperatureBuffer, "WallTemperatureForFluid", externalForcesKernel);
			ComputeHelper.SetBuffer(compute, wallTemperatureBuffer, "WallTemperatureCurrent", applyWallTemperatureKernel);
			ComputeHelper.SetBuffer(compute, predictedPositionBuffer, "PredictedPositions", updateWallTemperatureKernel);
			ComputeHelper.SetBuffer(compute, positionBuffer, "Positions", updateWallTemperatureKernel);
			ComputeHelper.SetBuffer(compute, thermalEnergyBuffer, "ThermalEnergy", updateWallTemperatureKernel);
			ComputeHelper.SetBuffer(compute, particlePhaseBuffer, "ParticlePhase", updateWallTemperatureKernel);
			ComputeHelper.SetBuffer(compute, spatialHash.SpatialOffsets, "SpatialOffsets", updateWallTemperatureKernel);
			ComputeHelper.SetBuffer(compute, spatialHash.SpatialKeys, "SpatialKeys", updateWallTemperatureKernel);

			// Reorder reads the original arrays as SRVs and writes only to sort-target UAVs.
			ComputeHelper.SetBuffer(compute, positionBuffer, "InputPositions", reorderKernel);
			ComputeHelper.SetBuffer(compute, predictedPositionBuffer, "InputPredictedPositions", reorderKernel);
			ComputeHelper.SetBuffer(compute, velocityBuffer, "InputVelocities", reorderKernel);
			ComputeHelper.SetBuffer(compute, thermalEnergyBuffer, "InputThermalEnergy", reorderKernel);
			ComputeHelper.SetBuffer(compute, particlePhaseBuffer, "InputParticlePhase", reorderKernel);

			// Copyback reads sort-target arrays as SRVs and writes only to simulation UAVs.
			ComputeHelper.SetBuffer(compute, sortTarget_Position, "InputSortedPositions", copybackKernel);
			ComputeHelper.SetBuffer(compute, sortTarget_PredicitedPosition, "InputSortedPredictedPositions", copybackKernel);
			ComputeHelper.SetBuffer(compute, sortTarget_Velocity, "InputSortedVelocities", copybackKernel);
			ComputeHelper.SetBuffer(compute, sortTarget_ThermalEnergy, "InputSortedThermalEnergy", copybackKernel);
			ComputeHelper.SetBuffer(compute, sortTarget_ParticlePhase, "InputSortedParticlePhase", copybackKernel);

			ComputeHelper.SetBuffer(compute, spatialHash.SpatialIndices, "SortedIndices", spatialHashKernel, reorderKernel);
			ComputeHelper.SetBuffer(compute, spatialHash.SpatialOffsets, "SpatialOffsets", spatialHashKernel, densityKernel, pressureKernel, viscosityKernel, phaseChangeKernel, diffuseThermalEnergyKernel);
			ComputeHelper.SetBuffer(compute, spatialHash.SpatialKeys, "SpatialKeys", spatialHashKernel, densityKernel, pressureKernel, viscosityKernel, phaseChangeKernel, diffuseThermalEnergyKernel);

			ComputeHelper.SetBuffer(compute, sortTarget_Position, "SortTarget_Positions", reorderKernel);
			ComputeHelper.SetBuffer(compute, sortTarget_PredicitedPosition, "SortTarget_PredictedPositions", reorderKernel);
			ComputeHelper.SetBuffer(compute, sortTarget_Velocity, "SortTarget_Velocities", reorderKernel);
			ComputeHelper.SetBuffer(compute, sortTarget_ThermalEnergy, "SortTarget_ThermalEnergy", reorderKernel);
			ComputeHelper.SetBuffer(compute, sortTarget_ParticlePhase, "SortTarget_ParticlePhase", reorderKernel);

			compute.SetInt("numParticles", numParticles);
		}


		void Update()
		{
			if (!isPaused)
			{
				float maxDeltaTime = maxTimestepFPS > 0 ? 1 / maxTimestepFPS : float.PositiveInfinity; // If framerate dips too low, run the simulation slower than real-time
				float dt = Mathf.Min(Time.deltaTime * timeScale, maxDeltaTime);
				RunSimulationFrame(dt);
			}

			if (pauseNextFrame)
			{
				isPaused = true;
				pauseNextFrame = false;
			}

			HandleInput();
		}

		void RunSimulationFrame(float frameTime)
		{
			float timeStep = frameTime / iterationsPerFrame;

			UpdateSettings(timeStep);

			for (int i = 0; i < iterationsPerFrame; i++)
			{
				RunSimulationStep();
				SimulationStepCompleted?.Invoke();
			}

		}

		void RunSimulationStep()
		{
			compute.SetInt("wallSpatialReady", wallSpatialReady ? 1 : 0);
			ComputeHelper.Dispatch(compute, wallTemperatureBuffer.count, kernelIndex: updateWallTemperatureKernel);
			ComputeHelper.Dispatch(compute, numParticles, kernelIndex: externalForcesKernel);
			ComputeHelper.Dispatch(compute, wallTemperatureBuffer.count, kernelIndex: applyWallTemperatureKernel);

			RunSpatial();

			ComputeHelper.Dispatch(compute, numParticles, kernelIndex: densityKernel);
			ComputeHelper.Dispatch(compute, numParticles, kernelIndex: phaseChangeKernel);
			ComputeHelper.Dispatch(compute, numParticles, kernelIndex: densityKernel);
			ComputeHelper.Dispatch(compute, numParticles, kernelIndex: pressureKernel);
			ComputeHelper.Dispatch(compute, numParticles, kernelIndex: viscosityKernel);
			ComputeHelper.Dispatch(compute, numParticles, kernelIndex: diffuseThermalEnergyKernel);
			ComputeHelper.Dispatch(compute, numParticles, kernelIndex: applyThermalEnergyKernel);
			ComputeHelper.Dispatch(compute, numParticles, kernelIndex: updatePositionKernel);
		}

		void RunSpatial()
		{
			ComputeHelper.Dispatch(compute, numParticles, kernelIndex: spatialHashKernel);
			spatialHash.Run();

			ComputeHelper.Dispatch(compute, numParticles, kernelIndex: reorderKernel);
			ComputeHelper.Dispatch(compute, numParticles, kernelIndex: copybackKernel);
			wallSpatialReady = true;
		}

		void UpdateSettings(float deltaTime)
		{
			compute.SetFloat("deltaTime", deltaTime);
			compute.SetFloat("gravity", gravity);
			compute.SetFloat("collisionDamping", collisionDamping);
			compute.SetFloat("smoothingRadius", smoothingRadius);
			compute.SetFloat("targetDensity", targetDensity);
			compute.SetFloat("pressureMultiplier", pressureMultiplier);
			compute.SetFloat("nearPressureMultiplier", nearPressureMultiplier);
			compute.SetFloat("viscosityStrength", viscosityStrength);
			compute.SetVector("boundsSize", boundsSize);
			compute.SetVector("boundsCentre", boundsCentre);
			compute.SetInt("obstacleEnabled", obstacleEnabled ? 1 : 0);
			compute.SetVector("obstacleSize", obstacleSize);
			compute.SetVector("obstacleCentre", obstacleCentre);
			compute.SetFloat("boundsRotationRadians", tankRotationDegrees * Mathf.Deg2Rad);
			if (thermalLoop != null)
			{
				compute.SetFloat("burnerPower", thermalLoop.burnerEnabled ? thermalLoop.burnerPower : 0);
				compute.SetFloat("burnerFocus", thermalLoop.airCollarOpening);
				compute.SetFloat("burnerLocalX", thermalLoop.burnerLocalX);
				compute.SetFloat("thermalCooling", thermalLoop.thermalCooling);
				compute.SetFloat("vapourCoolingMultiplier", thermalLoop.vapourCoolingMultiplier);
				compute.SetFloat("ambientTemperatureC", thermalLoop.ambientTemperatureC);
				compute.SetFloat("thermalDiffusionRate", thermalLoop.thermalDiffusionRate);
				compute.SetFloat("metresPerSimulationUnit", thermalLoop.metresPerSimulationUnit);
				compute.SetFloat("liquidThermalDiffusivity", thermalLoop.liquidThermalDiffusivity);
				compute.SetFloat("vapourThermalDiffusivity", thermalLoop.vapourThermalDiffusivity);
				compute.SetFloat("wallThermalDiffusivity", thermalLoop.wallThermalDiffusivity);
				compute.SetFloat("wallHeatCapacityScale", thermalLoop.wallHeatCapacityScale);
				compute.SetFloat("wallWaterTransferRate", thermalLoop.wallWaterTransferRate);
				compute.SetFloat("wallAmbientCoolingRate", thermalLoop.wallAmbientCoolingRate);
				compute.SetFloat("evaporationThreshold", thermalLoop.evaporationThreshold);
				compute.SetFloat("condensationThreshold", thermalLoop.condensationThreshold);
				compute.SetFloat("nucleationSiteSpacing", thermalLoop.nucleationSiteSpacing);
				compute.SetFloat("latentHeat", thermalLoop.latentHeat);
				compute.SetFloat("surfaceDensityFraction", thermalLoop.surfaceDensityFraction);
				compute.SetFloat("nearSurfaceDensityFraction", thermalLoop.nearSurfaceDensityFraction);
				compute.SetFloat("liquidThermalBuoyancy", thermalLoop.liquidThermalBuoyancy);
				compute.SetFloat("vapourBuoyancy", thermalLoop.vapourBuoyancy);
				compute.SetFloat("vapourDensityRatio", thermalLoop.vapourDensityRatio);
				compute.SetFloat("vapourPressureScale", thermalLoop.vapourPressureScale);
				compute.SetFloat("vapourViscosityScale", thermalLoop.vapourViscosityScale);
				compute.SetFloat("interphaseRepulsion", thermalLoop.interphaseRepulsion);
				compute.SetFloat("interphaseDrag", thermalLoop.interphaseDrag);
				compute.SetInt("pressureVentEnabled", thermalLoop.pressureVentEnabled ? 1 : 0);
				compute.SetFloat("pressureVentLocalX", thermalLoop.pressureVentLocalX);
				compute.SetFloat("pressureVentWidth", thermalLoop.pressureVentWidth);
				compute.SetFloat("pressureVentDensityThreshold", thermalLoop.pressureVentDensityThreshold);
				compute.SetFloat("boundsRotationRadians", 0);
			}

			compute.SetFloat("Poly6ScalingFactor", 4 / (Mathf.PI * Mathf.Pow(smoothingRadius, 8)));
			compute.SetFloat("SpikyPow3ScalingFactor", 10 / (Mathf.PI * Mathf.Pow(smoothingRadius, 5)));
			compute.SetFloat("SpikyPow2ScalingFactor", 6 / (Mathf.PI * Mathf.Pow(smoothingRadius, 4)));
			compute.SetFloat("SpikyPow3DerivativeScalingFactor", 30 / (Mathf.Pow(smoothingRadius, 5) * Mathf.PI));
			compute.SetFloat("SpikyPow2DerivativeScalingFactor", 12 / (Mathf.Pow(smoothingRadius, 4) * Mathf.PI));

			// Mouse interaction settings:
			Vector2 mousePos = Camera.main.ScreenToWorldPoint(Input.mousePosition);
			bool isPullInteraction = Input.GetMouseButton(0);
			bool isPushInteraction = Input.GetMouseButton(1);
			float currInteractStrength = 0;
			if (isPushInteraction || isPullInteraction)
			{
				currInteractStrength = isPushInteraction ? -interactionStrength : interactionStrength;
			}

			compute.SetVector("interactionInputPoint", mousePos);
			compute.SetFloat("interactionInputStrength", currInteractStrength);
			compute.SetFloat("interactionInputRadius", interactionRadius);
		}

		void SetInitialBufferData(Spawner2D.ParticleSpawnData spawnData)
		{
			float[] thermal = new float[numParticles];
			float ambientTemperature = thermalLoop != null ? thermalLoop.ambientTemperatureC : 20f;
			for (int i = 0; i < thermal.Length; i++) thermal[i] = ambientTemperature;
			uint[] phases = new uint[numParticles];
			positionBuffer.SetData(spawnData.positions);
			predictedPositionBuffer.SetData(spawnData.positions);
			velocityBuffer.SetData(spawnData.velocities);
			thermalEnergyBuffer.SetData(thermal);
			particlePhaseBuffer.SetData(phases);
			thermalEnergyNextBuffer.SetData(thermal);
			float[] wallTemperature = new float[wallTemperatureBuffer.count];
			for (int i = 0; i < wallTemperature.Length; i++) wallTemperature[i] = ambientTemperature;
			wallTemperatureBuffer.SetData(wallTemperature);
			wallTemperatureScratchBuffer.SetData(wallTemperature);
		}

		void HandleInput()
		{
			if (tankRotationKeyboardInput)
			{
				if (Input.GetKeyDown(KeyCode.A)) tankRotationDegrees -= tankRotationStepDegrees;
				if (Input.GetKeyDown(KeyCode.D)) tankRotationDegrees += tankRotationStepDegrees;
				if (Input.GetKeyDown(KeyCode.Backspace)) tankRotationDegrees = 0;
				tankRotationDegrees = Mathf.Clamp(tankRotationDegrees, -180, 180);
			}
			if (Input.GetKeyDown(KeyCode.O)) obstacleEnabled = !obstacleEnabled;
			if (Input.GetKeyDown(KeyCode.Space))
			{
				isPaused = !isPaused;
			}

			if (Input.GetKeyDown(KeyCode.RightArrow))
			{
				isPaused = false;
				pauseNextFrame = true;
			}

			if (Input.GetKeyDown(KeyCode.R))
			{
				isPaused = true;
				// Reset positions, the run single frame to get density etc (for debug purposes) and then reset positions again
				SetInitialBufferData(spawnData);
				wallSpatialReady = false;
				RunSimulationStep();
				SetInitialBufferData(spawnData);
				wallSpatialReady = false;
			}
		}


		void OnDestroy()
		{
			ComputeHelper.Release(positionBuffer, predictedPositionBuffer, velocityBuffer, densityBuffer, thermalEnergyBuffer, particlePhaseBuffer, thermalEnergyNextBuffer, wallTemperatureBuffer, wallTemperatureScratchBuffer, sortTarget_Position, sortTarget_Velocity, sortTarget_PredicitedPosition, sortTarget_ThermalEnergy, sortTarget_ParticlePhase);
			spatialHash.Release();
		}


		void OnDrawGizmos()
		{
			Matrix4x4 previousMatrix = Gizmos.matrix;
			Gizmos.color = new Color(0, 1, 0, 0.4f);
			Gizmos.matrix = Matrix4x4.TRS(boundsCentre, Quaternion.Euler(0, 0, tankRotationDegrees), Vector3.one);
			Gizmos.DrawWireCube(Vector2.zero, boundsSize);
			if (obstacleEnabled) Gizmos.DrawWireCube(obstacleCentre, obstacleSize);
			Gizmos.matrix = previousMatrix;

			if (Application.isPlaying)
			{
				Vector2 mousePos = Camera.main.ScreenToWorldPoint(Input.mousePosition);
				bool isPullInteraction = Input.GetMouseButton(0);
				bool isPushInteraction = Input.GetMouseButton(1);
				bool isInteracting = isPullInteraction || isPushInteraction;
				if (isInteracting)
				{
					Gizmos.color = isPullInteraction ? Color.green : Color.red;
					Gizmos.DrawWireSphere(mousePos, interactionRadius);
				}
			}
		}

		void OnGUI()
		{
			if (Application.isPlaying)
			{
				string rotationControls = tankRotationKeyboardInput ? "A/D rotate | Backspace reset" : "rotation locked";
				GUI.Label(new Rect(12, 12, 420, 28), "Tank angle: " + tankRotationDegrees.ToString("0.#") + " degrees | " + rotationControls);
				if (thermalLoop != null)
					GUI.Label(new Rect(12, 38, 620, 28), $"Burner {(thermalLoop.burnerEnabled ? "ON" : "OFF")} (B) | air collar [ / ] | power {thermalLoop.burnerPower:0}");
			}
		}
	}
}
