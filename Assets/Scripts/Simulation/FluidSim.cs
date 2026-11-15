using System;
using UnityEngine;
using Seb.GPUSorting;
using Unity.Mathematics;
using UnityEngine.Rendering;
using Stopwatch = System.Diagnostics.Stopwatch;
using System.Collections.Generic;
using Seb.Helpers;
using static Seb.Helpers.ComputeHelper;

namespace Seb.Fluid.Simulation
{
	public class FluidSim : MonoBehaviour
	{
		public event Action<FluidSim> SimulationInitCompleted;

		[Header("Time Step")] public float normalTimeScale = 1;
		public float slowTimeScale = 0.1f;
		public float maxTimestepFPS = 60; // if time-step dips lower than this fps, simulation will run slower (set to 0 to disable)
		public int iterationsPerFrame = 3;

		[Header("Simulation Settings")] public float gravity = -10;
		public float smoothingRadius = 0.2f;
		public float targetDensity = 630;
		public float pressureMultiplier = 288;
		public float nearPressureMultiplier = 2.15f;
		public float viscosityStrength = 0;
		public bool approachOnlyDamping;
		public float particleContactDistance, particleDampingDistance;
		[Range(0, 1)] public float collisionDamping = 0.95f;

		[Header("Foam Settings")] public bool foamActive;
		public bool sprayOnly;
		public int maxFoamParticleCount = 1000;
		public float trappedAirSpawnRate = 70;
		public float spawnRateFadeInTime = 0.5f;
		public float spawnRateFadeStartTime = 0;
		public Vector2 trappedAirVelocityMinMax = new(5, 25);
		public Vector2 foamKineticEnergyMinMax = new(15, 80);
		public float bubbleBuoyancy = 1.5f;
		public int sprayClassifyMaxNeighbours = 5;
		public int bubbleClassifyMinNeighbours = 15;
		public float bubbleScale = 0.5f;
		public float bubbleChangeScaleSpeed = 7;

		[Header("Volumetric Render Settings")] public bool renderToTex3D;
		public int densityTextureRes;
		[HideInInspector] public bool worldSpaceSurface;

		[Header("References")] public ComputeShader compute;
		public Spawner3D spawner;

		[HideInInspector] public RenderTexture DensityMap;
		public Vector3 Scale => transform.localScale;

		// Buffers
		public ComputeBuffer foamBuffer { get; private set; }
		public ComputeBuffer foamSortTargetBuffer { get; private set; }
		public ComputeBuffer foamCountBuffer { get; private set; }
		public ComputeBuffer positionBuffer { get; private set; }
		public ComputeBuffer velocityBuffer { get; private set; }
		public ComputeBuffer densityBuffer { get; private set; }
		public ComputeBuffer predictedPositionsBuffer;
		public ComputeBuffer debugBuffer { get; private set; }

		ComputeBuffer sortTarget_positionBuffer;
		ComputeBuffer sortTarget_velocityBuffer;
		ComputeBuffer sortTarget_predictedPositionsBuffer;

		// Kernel IDs
		int externalForcesKernel;
		int spatialHashKernel;
		int reorderKernel;
		int reorderCopybackKernel;
		int densityKernel;
		int pressureKernel;
		int viscosityKernel, commitDampedVelocitiesKernel;
		int updatePositionsKernel;
		int renderKernel;
		int foamUpdateKernel;
		int foamReorderCopyBackKernel;

		SpatialHash spatialHash;

		// State
		bool isPaused;
		bool pauseNextFrame;
		float smoothRadiusOld;
		public const int MaxIceBodies = 4;
		private const int MaxBoxBoundaries = 12;
		public int IceCount { get; set; }
		readonly Vector4[] iceHalfExtents = new Vector4[MaxBoxBoundaries];
		readonly Matrix4x4[] iceWorldToLocal = new Matrix4x4[MaxBoxBoundaries];
		readonly Matrix4x4[] iceLocalToWorld = new Matrix4x4[MaxBoxBoundaries];
		readonly Vector4[] iceVelocity = new Vector4[MaxBoxBoundaries];
		readonly Vector4[] iceAngularVelocity = new Vector4[MaxBoxBoundaries];
		readonly Vector4[] iceMassCentre = new Vector4[MaxBoxBoundaries];

		public void SetIceBoundary(int index, Rigidbody body, BoxCollider collider)
		{
			SetBoxBoundary(index, body.position + body.rotation * collider.center, body.rotation,
				collider.size, body.velocity, body.angularVelocity, body.worldCenterOfMass);
		}

		public void SetStaticBoxBoundary(int index, Vector3 centre, Vector3 size)
		{
			SetStaticBoxBoundary(index, centre, size, Quaternion.identity);
		}

		public void SetStaticBoxBoundary(int index, Vector3 centre, Vector3 size, Quaternion rotation)
		{
			SetBoxBoundary(index, centre, rotation, size, Vector3.zero, Vector3.zero, centre);
		}

		private void SetBoxBoundary(int index, Vector3 centre, Quaternion rotation, Vector3 size,
			Vector3 velocity, Vector3 angularVelocity, Vector3 massCentre)
		{
			iceHalfExtents[index] = size * .5f;
			iceLocalToWorld[index] = Matrix4x4.TRS(centre, rotation, Vector3.one);
			iceWorldToLocal[index] = iceLocalToWorld[index].inverse;
			iceVelocity[index] = velocity;
			iceAngularVelocity[index] = angularVelocity;
			iceMassCentre[index] = massCentre;
		}
		double simTimer;
		bool inSlowMode;
		Spawner3D.SpawnData spawnData;
		Dictionary<ComputeBuffer, string> bufferNameLookup;

		[HideInInspector] public bool manualControl;
		[HideInInspector] public bool transportTemperature;
		[HideInInspector] public bool thermalBuoyancy;
		[HideInInspector] public bool beakerFloorPressure;
		[HideInInspector] public bool boxWallPressure;
		[HideInInspector] public float beakerGlassFloorY;
		[NonSerialized] public float[] buoyancyDensityByDegree;
		public ComputeBuffer TemperatureRise { get; private set; }
		[HideInInspector] public bool trackLiquidMass;
		public ComputeBuffer LiquidFraction { get; private set; }
		float[] initialLiquidFraction;
		internal int MixedCoarseParticleCount { get; set; }
		internal float MixedCoarseMass { get; set; }
		internal float MixedGridCellScale { get; set; } = 1f;
		private bool MixedResolution => MixedCoarseParticleCount > 0;
		int sortLiquidMassKernel, copyLiquidMassKernel;
		[HideInInspector] public float liquidThermalDiffusivity;
		int sumThermalKernel, diffuseHeatKernel, commitHeatKernel;
		ComputeBuffer sortedTemperature;
		float[] initialTemperature;
		int activeReorderKernel, activeCopyBackKernel;
		int activeExternalForcesKernel;
		int activePressureKernel;
		[HideInInspector] public Vector4 beakerCollision; // Centre X/Z, inner radius, floor height.
		[HideInInspector] public float beakerRimY;
		public float SimulatedSeconds => (float)simTimer;

		public void AdvanceManually(float stepSeconds, int steps, bool updateSurface = true)
		{
			if (!manualControl || positionBuffer == null || stepSeconds <= 0 || steps <= 0)
				throw new InvalidOperationException("A ready manual simulation and positive timestep are required.");
			RunSimulationFrame(stepSeconds * steps, steps, updateSurface);
		}

		public void ResetParticles()
		{
			if (positionBuffer == null) return;
			if (positionBuffer.count != spawnData.points.Length)
			{
				ReleaseParticleBuffers();
				Initialize(spawnData);
				if (worldSpaceSurface && renderToTex3D) RunSimulationFrame(0, 1);
				return;
			}
			SetInitialBufferData(spawnData);
			if (worldSpaceSurface && renderToTex3D) RunSimulationFrame(0, 1);
		}

		public void AddWater(Vector3 centre, bool preserveExisting, float volumeFraction,
			float puddleRadius = 0, float puddleHeight = 0, bool refreshSurface = true)
		{
			if (trackLiquidMass || transportTemperature)
				throw new InvalidOperationException("Dynamic releases require the beaker mesh solver.");
			var batch = spawnData;
			var region = spawner.spawnRegions[0];
			if (volumeFraction < .999f || puddleRadius > 0)
			{
				var original = spawner.spawnRegions;
				float scale = Mathf.Pow(volumeFraction, 1f / 3f);
				region.cylinderRadius = puddleRadius > 0 ? puddleRadius : region.cylinderRadius * scale;
				region.cylinderHeight = puddleRadius > 0 ? puddleHeight : region.cylinderHeight * scale;
				try
				{
					spawner.spawnRegions = new[] { region };
					batch = spawner.GetSpawnData();
				}
				finally { spawner.spawnRegions = original; }
			}
			int oldCount = preserveExisting ? positionBuffer.count : 0;
			var previousPositions = positionBuffer;
			var previousVelocities = velocityBuffer;
			float3 offset = (float3)(centre - spawner.spawnRegions[0].centre);
			var points = new float3[batch.points.Length];
			for (int i = 0; i < points.Length; i++)
				points[i] = batch.points[i] + offset;

			double time = simTimer;
			ReleaseParticleBuffers(keepParticleState: oldCount > 0);
			Initialize(new Spawner3D.SpawnData { points = points, velocities = batch.velocities }, oldCount);
			simTimer = time;
			if (oldCount > 0)
			{
				int copy = compute.FindKernel("CopyWaterState");
				compute.SetInt("waterCopyCount", oldCount);
				compute.SetBuffer(copy, "WaterCopyPositions", previousPositions);
				compute.SetBuffer(copy, "WaterCopyVelocities", previousVelocities);
				compute.SetBuffer(copy, "Positions", positionBuffer);
				compute.SetBuffer(copy, "Velocities", velocityBuffer);
				Dispatch(compute, oldCount, kernelIndex: copy);
				Release(previousPositions, previousVelocities);
			}
			if (refreshSurface && renderToTex3D) RunSimulationFrame(0, 1);
		}

		void Start()
		{
			Debug.Log("Controls: Space = Play/Pause, Q = SlowMode, R = Reset");
			isPaused = false;

			Initialize();
		}

		void Initialize(Spawner3D.SpawnData? release = null, int existingCount = 0)
		{
			externalForcesKernel = compute.FindKernel("ExternalForces");
			spatialHashKernel = compute.FindKernel(MixedResolution ? "UpdateMixedSpatialHash" : "UpdateSpatialHash");
			reorderKernel = compute.FindKernel("Reorder");
			reorderCopybackKernel = compute.FindKernel("ReorderCopyBack");
			densityKernel = compute.FindKernel(MixedResolution ? "CalculateMixedDensities" : "CalculateDensities");
			pressureKernel = compute.FindKernel(MixedResolution ? "CalculateMixedPressureForce" : "CalculatePressureForce");
			viscosityKernel = compute.FindKernel(MixedResolution ? "CalculateMixedViscosity" : "CalculateViscosity");
			commitDampedVelocitiesKernel = compute.FindKernel("CommitDampedVelocities");
			updatePositionsKernel = compute.FindKernel("UpdatePositions");
			renderKernel = compute.FindKernel(MixedResolution ? "UpdateMixedDensityTexture" : "UpdateDensityTexture");
			foamUpdateKernel = compute.FindKernel("UpdateWhiteParticles");
			foamReorderCopyBackKernel = compute.FindKernel("WhiteParticlePrepareNextFrame");
			activeReorderKernel = reorderKernel;
			activeCopyBackKernel = reorderCopybackKernel;
			activeExternalForcesKernel = externalForcesKernel;
			activePressureKernel = pressureKernel;
			if (release == null) spawnData = spawner.GetSpawnData();
			var initial = release ?? spawnData;
			int numParticles = existingCount + initial.points.Length;

			spatialHash = new SpatialHash(numParticles);
			
			// Create buffers
			positionBuffer = CreateStructuredBuffer<float3>(numParticles);
			predictedPositionsBuffer = CreateStructuredBuffer<float3>(numParticles);
			velocityBuffer = CreateStructuredBuffer<float3>(numParticles);
			densityBuffer = CreateStructuredBuffer<float2>(numParticles);
			foamBuffer = CreateStructuredBuffer<FoamParticle>(maxFoamParticleCount);
			foamSortTargetBuffer = CreateStructuredBuffer<FoamParticle>(maxFoamParticleCount);
			foamCountBuffer = CreateStructuredBuffer<uint>(4096);
			debugBuffer = CreateStructuredBuffer<float3>(numParticles);

			sortTarget_positionBuffer = CreateStructuredBuffer<float3>(numParticles);
			sortTarget_predictedPositionsBuffer = CreateStructuredBuffer<float3>(numParticles);
			sortTarget_velocityBuffer = CreateStructuredBuffer<float3>(numParticles);

			bufferNameLookup = new Dictionary<ComputeBuffer, string>
			{
				{ positionBuffer, "Positions" },
				{ predictedPositionsBuffer, "PredictedPositions" },
				{ velocityBuffer, "Velocities" },
				{ densityBuffer, "Densities" },
				{ spatialHash.SpatialKeys, "SpatialKeys" },
				{ spatialHash.SpatialOffsets, "SpatialOffsets" },
				{ spatialHash.SpatialIndices, "SortedIndices" },
				{ sortTarget_positionBuffer, "SortTarget_Positions" },
				{ sortTarget_predictedPositionsBuffer, "SortTarget_PredictedPositions" },
				{ sortTarget_velocityBuffer, "SortTarget_Velocities" },
				{ foamCountBuffer, "WhiteParticleCounters" },
				{ foamBuffer, "WhiteParticles" },
				{ foamSortTargetBuffer, "WhiteParticlesCompacted" },
				{ debugBuffer, "Debug" }
			};

			if (transportTemperature)
			{
				activeReorderKernel = compute.FindKernel("ReorderWithHeat");
				activeCopyBackKernel = compute.FindKernel("ReorderCopyBackWithHeat");
				TemperatureRise = CreateStructuredBuffer<float>(numParticles);
				bufferNameLookup.Add(TemperatureRise, "TemperatureRise");
				sortedTemperature = CreateStructuredBuffer<float>(numParticles);
				bufferNameLookup.Add(sortedTemperature, "SortedTemperatureRise");
				initialTemperature = new float[numParticles];
				compute.SetBuffer(activeReorderKernel, "TemperatureRise", TemperatureRise);
				compute.SetBuffer(activeReorderKernel, "SortedTemperatureRise", sortedTemperature);
				compute.SetBuffer(activeCopyBackKernel, "TemperatureRise", TemperatureRise);
				compute.SetBuffer(activeCopyBackKernel, "SortedTemperatureRise", sortedTemperature);
				if (buoyancyDensityByDegree != null && buoyancyDensityByDegree.Length >= 2)
				{
					activePressureKernel = compute.FindKernel("CalculateThermalPressureForce");
					var densityTable = CreateStructuredBuffer<float>(buoyancyDensityByDegree.Length);
					bufferNameLookup.Add(densityTable, "BuoyancyDensity");
					densityTable.SetData(buoyancyDensityByDegree);
					compute.SetInt("BuoyancyDensityCount", buoyancyDensityByDegree.Length);
					compute.SetBuffer(activePressureKernel, "BuoyancyDensity", densityTable);
					compute.SetBuffer(activePressureKernel, "PressureTemperatureRise", TemperatureRise);
				}
			}

			compute.SetInt("TrackLiquidMass", trackLiquidMass ? 1 : 0);
			bool sortParticleWeights = trackLiquidMass || MixedResolution;
			LiquidFraction = CreateStructuredBuffer<float>(sortParticleWeights ? numParticles : 1);
			var sortedFraction = CreateStructuredBuffer<float>(sortParticleWeights ? numParticles : 1);
			bufferNameLookup.Add(LiquidFraction, "LiquidFraction");
			bufferNameLookup.Add(sortedFraction, "SortedLiquidFraction");
			initialLiquidFraction = new float[LiquidFraction.count];
			for (int i = 0; i < initialLiquidFraction.Length; i++) initialLiquidFraction[i] = i < MixedCoarseParticleCount ? MixedCoarseMass : 1;
			foreach (int kernel in new[] { densityKernel, activePressureKernel, viscosityKernel, renderKernel })
				compute.SetBuffer(kernel, "LiquidFraction", LiquidFraction);
			if (MixedResolution)
			{
				compute.SetFloat("MixedGridCellScale", MixedGridCellScale);
				compute.SetBuffer(spatialHashKernel, "LiquidFraction", LiquidFraction);
			}
			if (sortParticleWeights)
			{
				sortLiquidMassKernel = compute.FindKernel("SortLiquidMass");
				copyLiquidMassKernel = compute.FindKernel("CopyLiquidMass");
				compute.SetBuffer(sortLiquidMassKernel, "SortedIndices", spatialHash.SpatialIndices);
				foreach (int kernel in new[] { sortLiquidMassKernel, copyLiquidMassKernel })
				{
					compute.SetBuffer(kernel, "LiquidFraction", LiquidFraction);
					compute.SetBuffer(kernel, "SortedLiquidFraction", sortedFraction);
				}
			}

			if (transportTemperature && liquidThermalDiffusivity > 0)
			{
				sumThermalKernel = compute.FindKernel("SumThermalNeighbours");
				diffuseHeatKernel = compute.FindKernel("DiffuseLiquidHeat");
				commitHeatKernel = compute.FindKernel("CommitLiquidHeat");
				var weights = CreateStructuredBuffer<float>(numParticles);
				bufferNameLookup.Add(weights, "ThermalNeighbourWeight");
				foreach (int kernel in new[] { sumThermalKernel, diffuseHeatKernel })
					SetBuffers(compute, kernel, bufferNameLookup, new[] { weights, LiquidFraction,
						predictedPositionsBuffer, spatialHash.SpatialKeys, spatialHash.SpatialOffsets });
				compute.SetBuffer(diffuseHeatKernel, "TemperatureRise", TemperatureRise);
				compute.SetBuffer(diffuseHeatKernel, "SortedTemperatureRise", sortedTemperature);
				compute.SetBuffer(commitHeatKernel, "TemperatureRise", TemperatureRise);
				compute.SetBuffer(commitHeatKernel, "SortedTemperatureRise", sortedTemperature);
				compute.SetFloat("LiquidThermalDiffusivity", liquidThermalDiffusivity);
			}

			// Set buffer data
			SetInitialBufferData(initial, existingCount);

			// External forces kernel
			SetBuffers(compute, activeExternalForcesKernel, bufferNameLookup, new ComputeBuffer[]
			{
				positionBuffer,
				predictedPositionsBuffer,
				velocityBuffer
			});

			// Spatial hash kernel
			SetBuffers(compute, spatialHashKernel, bufferNameLookup, new ComputeBuffer[]
			{
				spatialHash.SpatialKeys,
				spatialHash.SpatialOffsets,
				predictedPositionsBuffer,
				spatialHash.SpatialIndices
			});

			// Reorder kernel
			SetBuffers(compute, activeReorderKernel, bufferNameLookup, new ComputeBuffer[]
			{
				positionBuffer,
				sortTarget_positionBuffer,
				predictedPositionsBuffer,
				sortTarget_predictedPositionsBuffer,
				velocityBuffer,
				sortTarget_velocityBuffer,
				spatialHash.SpatialIndices
			});

			// Reorder copyback kernel
			SetBuffers(compute, activeCopyBackKernel, bufferNameLookup, new ComputeBuffer[]
			{
				positionBuffer,
				sortTarget_positionBuffer,
				predictedPositionsBuffer,
				sortTarget_predictedPositionsBuffer,
				velocityBuffer,
				sortTarget_velocityBuffer,
				spatialHash.SpatialIndices
			});

			// Density kernel
			SetBuffers(compute, densityKernel, bufferNameLookup, new ComputeBuffer[]
			{
				predictedPositionsBuffer,
				densityBuffer,
				spatialHash.SpatialKeys,
				spatialHash.SpatialOffsets
			});

			// Pressure kernel
			SetBuffers(compute, activePressureKernel, bufferNameLookup, new ComputeBuffer[]
			{
				predictedPositionsBuffer,
				densityBuffer,
				velocityBuffer,
				spatialHash.SpatialKeys,
				spatialHash.SpatialOffsets,
				foamBuffer,
				foamCountBuffer,
				debugBuffer
			});

			// Viscosity kernel
			SetBuffers(compute, viscosityKernel, bufferNameLookup, new ComputeBuffer[]
			{
				predictedPositionsBuffer,
				densityBuffer,
				velocityBuffer,
				spatialHash.SpatialKeys,
				spatialHash.SpatialOffsets
			});

			compute.SetBuffer(viscosityKernel, "SortTarget_Velocities", sortTarget_velocityBuffer);
			compute.SetBuffer(commitDampedVelocitiesKernel, "SortTarget_Velocities", sortTarget_velocityBuffer);
			compute.SetBuffer(commitDampedVelocitiesKernel, "Velocities", velocityBuffer);

			// Update positions kernel
			SetBuffers(compute, updatePositionsKernel, bufferNameLookup, new ComputeBuffer[]
			{
				positionBuffer,
				velocityBuffer
			});

			// Render to 3d tex kernel
			SetBuffers(compute, renderKernel, bufferNameLookup, new ComputeBuffer[]
			{
				predictedPositionsBuffer,
				densityBuffer,
				spatialHash.SpatialKeys,
				spatialHash.SpatialOffsets,
			});

			// Foam update kernel
			SetBuffers(compute, foamUpdateKernel, bufferNameLookup, new ComputeBuffer[]
			{
				foamBuffer,
				foamCountBuffer,
				predictedPositionsBuffer,
				densityBuffer,
				velocityBuffer,
				spatialHash.SpatialKeys,
				spatialHash.SpatialOffsets,
				foamSortTargetBuffer,
				//debugBuffer
			});


			// Foam reorder copyback kernel
			SetBuffers(compute, foamReorderCopyBackKernel, bufferNameLookup, new ComputeBuffer[]
			{
				foamBuffer,
				foamSortTargetBuffer,
				foamCountBuffer,
			});

			compute.SetInt("numParticles", positionBuffer.count);
			compute.SetInt("MaxWhiteParticleCount", maxFoamParticleCount);

			UpdateSmoothingConstants();

			// Run single frame of sim with deltaTime = 0 to initialize density texture
			// (so that display can work even if paused at start)
			if (renderToTex3D && release == null)
			{
				RunSimulationFrame(0);
			}

			SimulationInitCompleted?.Invoke(this);
		}

		void Update()
		{
			if (manualControl) return;
			// Run simulation
			if (!isPaused)
			{
				float maxDeltaTime = maxTimestepFPS > 0 ? 1 / maxTimestepFPS : float.PositiveInfinity; // If framerate dips too low, run the simulation slower than real-time
				float dt = Mathf.Min(Time.deltaTime * ActiveTimeScale, maxDeltaTime);
				RunSimulationFrame(dt);
			}

			if (pauseNextFrame)
			{
				isPaused = true;
				pauseNextFrame = false;
			}

			HandleInput();
		}

		void RunSimulationFrame(float frameDeltaTime) => RunSimulationFrame(frameDeltaTime, iterationsPerFrame);

		void RunSimulationFrame(float frameDeltaTime, int steps, bool updateSurface = true)
		{
			float subStepDeltaTime = frameDeltaTime / steps;
			UpdateSettings(subStepDeltaTime, frameDeltaTime);

			// Simulation sub-steps
			for (int i = 0; i < steps; i++)
			{
				simTimer += subStepDeltaTime;
				RunSimulationStep();
			}

			// Foam and spray particles
			if (foamActive)
			{
				Dispatch(compute, maxFoamParticleCount, kernelIndex: foamUpdateKernel);
				Dispatch(compute, maxFoamParticleCount, kernelIndex: foamReorderCopyBackKernel);
			}

			// 3D density map
			if (renderToTex3D && updateSurface)
			{
				UpdateDensityMap();
			}
		}

		void UpdateDensityMap()
		{
			float maxAxis = Mathf.Max(transform.localScale.x, transform.localScale.y, transform.localScale.z);
			int w = Mathf.RoundToInt(transform.localScale.x / maxAxis * densityTextureRes);
			int h = Mathf.RoundToInt(transform.localScale.y / maxAxis * densityTextureRes);
			int d = Mathf.RoundToInt(transform.localScale.z / maxAxis * densityTextureRes);
			CreateRenderTexture3D(ref DensityMap, w, h, d, (worldSpaceSurface ? UnityEngine.Experimental.Rendering.GraphicsFormat.R32_SFloat : UnityEngine.Experimental.Rendering.GraphicsFormat.R16_SFloat), TextureWrapMode.Clamp);
			//Debug.Log(w + " " + h + "  " + d);
			compute.SetTexture(renderKernel, "DensityMap", DensityMap);
			compute.SetVector("densityMapCentre", worldSpaceSurface ? transform.position : Vector3.zero);
			compute.SetInts("densityMapSize", DensityMap.width, DensityMap.height, DensityMap.volumeDepth);
			Dispatch(compute, DensityMap.width, DensityMap.height, DensityMap.volumeDepth, renderKernel);
		}

		internal static readonly string[] StageProfileLabels =
		{
			"MARIN.Predict", "MARIN.HashAndSort", "MARIN.Reorder", "MARIN.Density",
			"MARIN.Pressure", "MARIN.Viscosity", "MARIN.Integrate"
		};
		internal long[] StageCompletionTicks { get; set; }

		internal void WaitForSubmittedWork() => WaitForStageBuffer(positionBuffer);

		private static void WaitForStageBuffer(ComputeBuffer buffer)
		{
			var request = AsyncGPUReadback.Request(buffer, buffer.stride, 0);
			request.WaitForCompletion();
			if (request.hasError) throw new InvalidOperationException("MARIN stage-completion readback failed.");
		}

		private long CompleteStage(int stage, ComputeBuffer buffer, long start)
		{
			if (StageCompletionTicks == null) return 0;
			WaitForStageBuffer(buffer);
			long end = Stopwatch.GetTimestamp();
			StageCompletionTicks[stage] += end - start;
			return end;
		}

		void RunSimulationStep()
		{
			long stageStart = StageCompletionTicks != null ? Stopwatch.GetTimestamp() : 0;
			Dispatch(compute, positionBuffer.count, kernelIndex: activeExternalForcesKernel);
			stageStart = CompleteStage(0, predictedPositionsBuffer, stageStart);

			Dispatch(compute, positionBuffer.count, kernelIndex: spatialHashKernel);
			spatialHash.Run();
			stageStart = CompleteStage(1, spatialHash.SpatialIndices, stageStart);
			
			if (trackLiquidMass || MixedResolution)
			{
				Dispatch(compute, positionBuffer.count, kernelIndex: sortLiquidMassKernel);
				Dispatch(compute, positionBuffer.count, kernelIndex: copyLiquidMassKernel);
			}
			Dispatch(compute, positionBuffer.count, kernelIndex: activeReorderKernel);
			Dispatch(compute, positionBuffer.count, kernelIndex: activeCopyBackKernel);
			stageStart = CompleteStage(2, positionBuffer, stageStart);
			if (commitHeatKernel != 0)
			{
				Dispatch(compute, positionBuffer.count, kernelIndex: sumThermalKernel);
				Dispatch(compute, positionBuffer.count, kernelIndex: diffuseHeatKernel);
				Dispatch(compute, positionBuffer.count, kernelIndex: commitHeatKernel);
			}

			Dispatch(compute, positionBuffer.count, kernelIndex: densityKernel);
			stageStart = CompleteStage(3, densityBuffer, stageStart);
			Dispatch(compute, positionBuffer.count, kernelIndex: activePressureKernel);
			stageStart = CompleteStage(4, velocityBuffer, stageStart);
			if (viscosityStrength != 0 || approachOnlyDamping)
			{
				Dispatch(compute, positionBuffer.count, kernelIndex: viscosityKernel);
				if (approachOnlyDamping || MixedResolution)
					Dispatch(compute, positionBuffer.count, kernelIndex: commitDampedVelocitiesKernel);
			}
			stageStart = CompleteStage(5, velocityBuffer, stageStart);
			Dispatch(compute, positionBuffer.count, kernelIndex: updatePositionsKernel);
			CompleteStage(6, positionBuffer, stageStart);
		}

		void UpdateSmoothingConstants()
		{
			float r = smoothingRadius;
			float spikyPow2 = 15 / (2 * Mathf.PI * Mathf.Pow(r, 5));
			float spikyPow3 = 15 / (Mathf.PI * Mathf.Pow(r, 6));
			float spikyPow2Grad = 15 / (Mathf.PI * Mathf.Pow(r, 5));
			float spikyPow3Grad = 45 / (Mathf.PI * Mathf.Pow(r, 6));

			compute.SetFloat("K_SpikyPow2", spikyPow2);
			compute.SetFloat("K_SpikyPow3", spikyPow3);
			compute.SetFloat("K_SpikyPow2Grad", spikyPow2Grad);
			compute.SetFloat("K_SpikyPow3Grad", spikyPow3Grad);
		}

		void UpdateSettings(float stepDeltaTime, float frameDeltaTime)
		{
			if (smoothingRadius != smoothRadiusOld)
			{
				smoothRadiusOld = smoothingRadius;
				UpdateSmoothingConstants();
			}

			Vector3 simBoundsSize = transform.localScale;
			Vector3 simBoundsCentre = transform.position;

			compute.SetFloat("deltaTime", stepDeltaTime);
			compute.SetFloat("predictionTime", manualControl ? stepDeltaTime : 1f / 120f);
			compute.SetVector("beakerCollision", beakerCollision);
			compute.SetInt("iceCount", IceCount);
			if (IceCount > 0)
			{
				compute.SetVectorArray("iceHalfExtents", iceHalfExtents);
				compute.SetMatrixArray("iceWorldToLocal", iceWorldToLocal);
				compute.SetMatrixArray("iceLocalToWorld", iceLocalToWorld);
				compute.SetVectorArray("iceVelocity", iceVelocity);
				compute.SetVectorArray("iceAngularVelocity", iceAngularVelocity);
				compute.SetVectorArray("iceMassCentre", iceMassCentre);
			}
			compute.SetFloat("beakerRimY", beakerRimY);
			compute.SetFloat("whiteParticleDeltaTime", frameDeltaTime);
			compute.SetBool("diffuseActive", foamActive);
			compute.SetBool("sprayOnly", foamActive && sprayOnly);
			compute.SetFloat("simTime", (float)simTimer);
			compute.SetFloat("gravity", gravity);
			compute.SetBool("BoxWallPressure", boxWallPressure);
			compute.SetFloat("BeakerFloorPressure", beakerFloorPressure ? 1f : 0f);
			compute.SetFloat("beakerGlassFloorY", beakerGlassFloorY);
			if (activePressureKernel != pressureKernel)
				compute.SetFloat("ThermalBuoyancy", thermalBuoyancy ? 1f : 0f);
			compute.SetFloat("collisionDamping", collisionDamping);
			compute.SetFloat("smoothingRadius", smoothingRadius);
			compute.SetFloat("targetDensity", targetDensity);
			compute.SetFloat("pressureMultiplier", pressureMultiplier);
			compute.SetFloat("nearPressureMultiplier", nearPressureMultiplier);
			compute.SetFloat("viscosityStrength", viscosityStrength);
			compute.SetBool("approachOnlyDamping", approachOnlyDamping);
			compute.SetFloat("particleContactDistance", particleContactDistance);
			compute.SetFloat("particleDampingDistance", Mathf.Min(particleDampingDistance, smoothingRadius));
			compute.SetVector("boundsSize", simBoundsSize);
			compute.SetVector("centre", simBoundsCentre);

			compute.SetMatrix("localToWorld", transform.localToWorldMatrix);
			compute.SetMatrix("worldToLocal", transform.worldToLocalMatrix);

			// Foam settings
			float fadeInT = (spawnRateFadeInTime <= 0) ? 1 : Mathf.Clamp01((float)((simTimer - spawnRateFadeStartTime) / spawnRateFadeInTime));
			compute.SetVector("trappedAirParams", new Vector3(trappedAirSpawnRate * fadeInT * fadeInT, trappedAirVelocityMinMax.x, trappedAirVelocityMinMax.y));
			compute.SetVector("kineticEnergyParams", foamKineticEnergyMinMax);
			compute.SetFloat("bubbleBuoyancy", bubbleBuoyancy);
			compute.SetInt("sprayClassifyMaxNeighbours", sprayClassifyMaxNeighbours);
			compute.SetInt("bubbleClassifyMinNeighbours", bubbleClassifyMinNeighbours);
			compute.SetFloat("bubbleScaleChangeSpeed", bubbleChangeScaleSpeed);
			compute.SetFloat("bubbleScale", bubbleScale);
		}

		void SetInitialBufferData(Spawner3D.SpawnData spawnData, int offset = 0)
		{
			positionBuffer.SetData(spawnData.points, 0, offset, spawnData.points.Length);
			predictedPositionsBuffer.SetData(spawnData.points, 0, offset, spawnData.points.Length);
			velocityBuffer.SetData(spawnData.velocities, 0, offset, spawnData.velocities.Length);
			if (TemperatureRise != null) TemperatureRise.SetData(initialTemperature);
			LiquidFraction?.SetData(initialLiquidFraction);

			foamBuffer.SetData(new FoamParticle[foamBuffer.count]);

			debugBuffer.SetData(new float3[debugBuffer.count]);
			foamCountBuffer.SetData(new uint[foamCountBuffer.count]);
			simTimer = 0;
		}

		void HandleInput()
		{
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
				pauseNextFrame = true;
				SetInitialBufferData(spawnData);
				// Run single frame of sim with deltaTime = 0 to initialize density texture
				// (so that display can work even if paused at start)
				if (renderToTex3D)
				{
					RunSimulationFrame(0);
				}
			}

			if (Input.GetKeyDown(KeyCode.Q))
			{
				inSlowMode = !inSlowMode;
			}
		}

		private float ActiveTimeScale => inSlowMode ? slowTimeScale : normalTimeScale;

		void ReleaseParticleBuffers(bool keepParticleState = false)
		{
			if (bufferNameLookup != null)
			{
				foreach (var buffer in bufferNameLookup.Keys)
				{
					// SpatialHash owns these buffers and releases them together with its sorter.
					if (spatialHash != null && (buffer == spatialHash.SpatialKeys ||
						buffer == spatialHash.SpatialOffsets || buffer == spatialHash.SpatialIndices))
						continue;
					if (keepParticleState && (buffer == positionBuffer || buffer == velocityBuffer)) continue;
					Release(buffer);
				}
				bufferNameLookup = null;
			}

			spatialHash?.Release();
			spatialHash = null;
		}

		void OnDestroy()
		{
			ReleaseParticleBuffers();
			if (DensityMap != null) { DensityMap.Release(); Destroy(DensityMap); }
		}


		public struct FoamParticle
		{
			public float3 position;
			public float3 velocity;
			public float lifetime;
			public float scale;
		}

		void OnDrawGizmos()
		{
			// Draw Bounds
			var m = Gizmos.matrix;
			Gizmos.matrix = transform.localToWorldMatrix;
			Gizmos.color = new Color(0, 1, 0, 0.5f);
			Gizmos.DrawWireCube(Vector3.zero, Vector3.one);
			Gizmos.matrix = m;
		}
	}
}
