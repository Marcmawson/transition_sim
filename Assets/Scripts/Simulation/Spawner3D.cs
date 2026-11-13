using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace Seb.Fluid.Simulation
{

	public class Spawner3D : MonoBehaviour
	{
		public int particleSpawnDensity = 600;
		public float3 initialVel;
		public float jitterStrength;
		public bool showSpawnBounds;
		public SpawnRegion[] spawnRegions;

		[Header("Debug Info")] public int debug_num_particles;
		public float debug_spawn_volume;


		public SpawnData GetSpawnData()
		{
			List<float3> allPoints = new();
			List<float3> allVelocities = new();

			foreach (SpawnRegion region in spawnRegions)
			{
				if (region.cylinderRadius > 0 && region.cylinderHeight > 0)
				{
					SpawnCylinder(region, allPoints, allVelocities);
					continue;
				}

				int particlesPerAxis = region.CalculateParticleCountPerAxis(particleSpawnDensity);
				(float3[] points, float3[] velocities) = SpawnCube(particlesPerAxis, region.centre, Vector3.one * region.size);
				allPoints.AddRange(points);
				allVelocities.AddRange(velocities);
			}

			return new SpawnData() { points = allPoints.ToArray(), velocities = allVelocities.ToArray() };
		}

		void SpawnCylinder(SpawnRegion region, List<float3> points, List<float3> velocities)
		{
			float radius = region.cylinderRadius;
			float height = region.cylinderHeight;
			float spacing = Mathf.Pow(1f / particleSpawnDensity, 1f / 3f);
			int width = Mathf.Max(1, Mathf.RoundToInt(2 * radius / spacing));
			int layers = Mathf.Max(1, Mathf.RoundToInt(height / spacing));
			float radialStep = 2 * radius / width;
			float verticalStep = height / layers;
			float jitter = Mathf.Min(jitterStrength, .25f * Mathf.Min(radialStep, verticalStep));

			for (int x = 0; x < width; x++)
			for (int z = 0; z < width; z++)
			{
				float px = (x + .5f) * radialStep - radius;
				float pz = (z + .5f) * radialStep - radius;
				if (px * px + pz * pz > (radius - jitter) * (radius - jitter)) continue;
				for (int y = 0; y < layers; y++)
				{
					var offset = new Vector3(px, (y + .5f) * verticalStep - height / 2, pz);
					points.Add((float3)(region.centre + offset + UnityEngine.Random.insideUnitSphere * jitter));
					velocities.Add(initialVel);
				}
			}
		}

		(float3[] p, float3[] v) SpawnCube(int numPerAxis, Vector3 centre, Vector3 size)
		{
			int numPoints = numPerAxis * numPerAxis * numPerAxis;
			float3[] points = new float3[numPoints];
			float3[] velocities = new float3[numPoints];

			int i = 0;

			for (int x = 0; x < numPerAxis; x++)
			{
				for (int y = 0; y < numPerAxis; y++)
				{
					for (int z = 0; z < numPerAxis; z++)
					{
						float tx = x / (numPerAxis - 1f);
						float ty = y / (numPerAxis - 1f);
						float tz = z / (numPerAxis - 1f);

						float px = (tx - 0.5f) * size.x + centre.x;
						float py = (ty - 0.5f) * size.y + centre.y;
						float pz = (tz - 0.5f) * size.z + centre.z;
						float3 jitter = UnityEngine.Random.insideUnitSphere * jitterStrength;
						points[i] = new float3(px, py, pz) + jitter;
						velocities[i] = initialVel;
						i++;
					}
				}
			}

			return (points, velocities);
		}



		void OnValidate()
		{
			debug_spawn_volume = 0;
			debug_num_particles = 0;

			if (spawnRegions != null)
			{
				foreach (SpawnRegion region in spawnRegions)
				{
					debug_spawn_volume += region.Volume;
					int numPerAxis = region.CalculateParticleCountPerAxis(particleSpawnDensity);
					debug_num_particles += region.cylinderRadius > 0 && region.cylinderHeight > 0
						? Mathf.RoundToInt(region.Volume * particleSpawnDensity)
						: numPerAxis * numPerAxis * numPerAxis;
				}
			}
		}

		void OnDrawGizmos()
		{
			if (showSpawnBounds && !Application.isPlaying)
			{
				foreach (SpawnRegion region in spawnRegions)
				{
					Gizmos.color = region.debugDisplayCol;
					Gizmos.DrawWireCube(region.centre, region.cylinderRadius > 0 && region.cylinderHeight > 0
						? new Vector3(2 * region.cylinderRadius, region.cylinderHeight, 2 * region.cylinderRadius)
						: Vector3.one * region.size);
				}
			}
		}

		[System.Serializable]
		public struct SpawnRegion
		{
			public Vector3 centre;
			public float size;
			public float cylinderRadius, cylinderHeight;
			public Color debugDisplayCol;

			public float Volume => cylinderRadius > 0 && cylinderHeight > 0
				? Mathf.PI * cylinderRadius * cylinderRadius * cylinderHeight : size * size * size;

			public int CalculateParticleCountPerAxis(int particleDensity)
			{
				int targetParticleCount = (int)(Volume * particleDensity);
				int particlesPerAxis = (int)Math.Cbrt(targetParticleCount);
				return particlesPerAxis;
			}
		}

		public struct SpawnData
		{
			public float3[] points;
			public float3[] velocities;
		}
	}
}