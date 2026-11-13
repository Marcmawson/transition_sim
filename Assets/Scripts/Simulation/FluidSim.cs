using System;
using UnityEngine;
using Seb.GPUSorting;
using Unity.Mathematics;
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
		[Range(0, 1)] public float collisionDamping = 0.95f;

		[Header("Foam Settings")] public bool foamActive;
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
		const int externalForcesKernel = 0;
		const int spatialHashKernel = 1;
		const int reorderKernel = 2;
		const int reorderCopybackKernel = 3;
		const int densityKernel = 4;
		const int pressureKernel = 5;
		const int viscosityKernel = 6;
		const int updatePositionsKernel = 7;
		const int renderKernel = 8;
		const int foamUpdateKernel = 9;
		const int foamReorderCopyBackKernel = 10;

		SpatialHash spatialHash;

		// State
		bool isPaused;
		bool pauseNextFrame;
		float smoothRadiusOld;
		public float iceHalfExtent;
		public Matrix4x4 iceWorldToLocal, iceLocalToWorld;
		public Vector3 iceVelocity, iceAngularVelocity, iceMassCentre;
		double simTimer;
		bool inSlowMode;
		Spawner3D.SpawnData spawnData;
		Dictionary<ComputeBuffer, string> bufferNameLookup;

		[HideInInspector] public bool manualControl;
		[HideInInspector] public bool transportTemperature;
		[HideInInspector] public bool thermalBuoyancy;
		[HideInInspector] public bool beakerFloorPressure;
		[HideInInspector] public float beakerGlassFloorY;
		[NonSerialized] public float[] buoyancyDensityByDegree;
		public ComputeBuffer TemperatureRise { get; private set; }
		[HideInInspector] public bool trackLiquidMass;
		public ComputeBuffer LiquidFraction { get; private set; }
		float[] initialLiquidFraction;
		int sortLiquidMassKernel, copyLiquidMassKernel;
		[HideInInspector] public float liquidThermalDiffusivity;
		int sumThermalKernel, diffuseHeatKernel, commitHeatKernel;
		ComputeBuffer sortedTemperature;
		float[] initialTemperature;
		int activeReorderKernel = reorderKernel, activeCopyBackKernel = reorderCopybackKernel;
		int activeExternalForcesKernel = externalForcesKernel;
		int activePressureKernel = pressureKernel;
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
				return;
			}
			SetInitialBufferData(spawnData);
			if (worldSpaceSurface && renderToTex3D) RunSimulationFrame(0, 1);
		}

		public void AddWater(Vector3 centre, bool preserveExisting, float volumeFraction)
		{
			if (trackLiquidMass || transportTemperature)
				throw new InvalidOperationException("Dynamic releases require the beaker mesh solver.");
			var batch = spawnData;
			var region = spawner.spawnRegions[0];
			if (volumeFraction < .999f)
			{
				var original = spawner.spawnRegions;
				float scale = Mathf.Pow(volumeFraction, 1f / 3f);
				region.cylinderRadius *= scale;
				region.cylinderHeight *= scale;
				try
				{
					spawner.spawnRegions = new[] { region };
					batch = spawner.GetSpawnData();
				}
				finally { spawner.spawnRegions = original; }
			}
			int oldCount = preserveExisting ? positionBuffer.count : 0;
			var points = new float3[oldCount + batch.points.Length];
			var velocities = new float3[points.Length];
			if (oldCount > 0)
			{
				positionBuffer.GetData(points, 0, 0, oldCount);
				velocityBuffer.GetData(velocities, 0, 0, oldCount);
			}
			float3 offset = (float3)(centre - spawner.spawnRegions[0].centre);
			for (int i = 0; i < batch.points.Length; i++)
				points[oldCount + i] = batch.points[i] + offset;
			double time = simTimer;
			ReleaseParticleBuffers();
			Initialize(new Spawner3D.SpawnData { points = points, velocities = velocities });
			simTimer = time;
		}

		void Start()
		{
			Debug.Log("Controls: Space = Play/Pause, Q = SlowMode, R = Reset");
			isPaused = false;

			Initialize();
		}

		void Initialize(Spawner3D.SpawnData? release = null)
		{
			if (release == null) spawnData = spawner.GetSpawnData();
			var initial = release ?? spawnData;
			int numParticles = initial.points.Length;

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
			LiquidFraction = CreateStructuredBuffer<float>(trackLiquidMass ? numParticles : 1);
			var sortedFraction = CreateStructuredBuffer<float>(trackLiquidMass ? numParticles : 1);
			bufferNameLookup.Add(LiquidFraction, "LiquidFraction");
			bufferNameLookup.Add(sortedFraction, "SortedLiquidFraction");
			initialLiquidFraction = new float[LiquidFraction.count];
			for (int i = 0; i < initialLiquidFraction.Length; i++) initialLiquidFraction[i] = 1;
			foreach (int kernel in new[] { densityKernel, activePressureKernel, viscosityKernel, renderKernel })
				compute.SetBuffer(kernel, "LiquidFraction", LiquidFraction);
			if (trackLiquidMass)
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
			SetInitialBufferData(initial);

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
			if (renderToTex3D)
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

		void RunSimulationStep()
		{
			Dispatch(compute, positionBuffer.count, kernelIndex: activeExternalForcesKernel);

			Dispatch(compute, positionBuffer.count, kernelIndex: spatialHashKernel);
			spatialHash.Run();
			
			if (trackLiquidMass)
			{
				Dispatch(compute, positionBuffer.count, kernelIndex: sortLiquidMassKernel);
				Dispatch(compute, positionBuffer.count, kernelIndex: copyLiquidMassKernel);
			}
			Dispatch(compute, positionBuffer.count, kernelIndex: activeReorderKernel);
			Dispatch(compute, positionBuffer.count, kernelIndex: activeCopyBackKernel);
			if (commitHeatKernel != 0)
			{
				Dispatch(compute, positionBuffer.count, kernelIndex: sumThermalKernel);
				Dispatch(compute, positionBuffer.count, kernelIndex: diffuseHeatKernel);
				Dispatch(compute, positionBuffer.count, kernelIndex: commitHeatKernel);
			}

			Dispatch(compute, positionBuffer.count, kernelIndex: densityKernel);
			Dispatch(compute, positionBuffer.count, kernelIndex: activePressureKernel);
			if (viscosityStrength != 0) Dispatch(compute, positionBuffer.count, kernelIndex: viscosityKernel);
			Dispatch(compute, positionBuffer.count, kernelIndex: updatePositionsKernel);
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
			compute.SetFloat("iceHalfExtent", iceHalfExtent);
			compute.SetMatrix("iceWorldToLocal", iceWorldToLocal);
			compute.SetMatrix("iceLocalToWorld", iceLocalToWorld);
			compute.SetVector("iceVelocity", iceVelocity);
			compute.SetVector("iceAngularVelocity", iceAngularVelocity);
			compute.SetVector("iceMassCentre", iceMassCentre);
			compute.SetFloat("beakerRimY", beakerRimY);
			compute.SetFloat("whiteParticleDeltaTime", frameDeltaTime);
			compute.SetFloat("simTime", (float)simTimer);
			compute.SetFloat("gravity", gravity);
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

		void SetInitialBufferData(Spawner3D.SpawnData spawnData)
		{
			positionBuffer.SetData(spawnData.points);
			predictedPositionsBuffer.SetData(spawnData.points);
			velocityBuffer.SetData(spawnData.velocities);
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

		void ReleaseParticleBuffers()
		{
			if (bufferNameLookup != null)
			foreach (var kvp in bufferNameLookup)
			{
				Release(kvp.Key);
			}

			spatialHash?.Release();
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
