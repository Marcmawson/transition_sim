using Seb.Helpers;
using UnityEngine;
using Seb.Fluid.Simulation;

namespace Seb.Fluid.Rendering
{

	public class ParticleDisplay3D : MonoBehaviour
	{
		public enum DisplayMode
		{
			None,
			Shaded3D,
			Billboard
		}

		[Header("Settings")] public DisplayMode mode;
		public float scale;
		public Gradient colourMap;
		[HideInInspector] public bool showTemperature = true;
		Gradient previousColourMap;
		public int gradientResolution;
		public float velocityDisplayMax;
		[HideInInspector] public float temperatureRiseDisplayMax = 10f;
		public int meshResolution;

		[Header("References")] public FluidSim sim;
		public Shader shaderShaded;
		public Shader shaderBillboard;

		Mesh mesh;
		Material mat;
		ComputeBuffer argsBuffer;
		Texture2D gradientTexture;
		DisplayMode modeOld;
		bool needsUpdate = true;

		void LateUpdate()
		{
			if (sim == null || sim.positionBuffer == null) return;
			UpdateSettings();

			if (mode != DisplayMode.None)
			{
				Bounds bounds = new Bounds(Vector3.zero, Vector3.one * 10000);
				Graphics.DrawMeshInstancedIndirect(mesh, 0, mat, bounds, argsBuffer);
			}
		}

		void UpdateSettings()
		{
			if (modeOld != mode)
			{
				modeOld = mode;
				if (mesh != null) Destroy(mesh);
				if (mat != null) Destroy(mat);
				mesh = null;
				mat = null;
				needsUpdate = true;
				if (mode != DisplayMode.None)
				{
					if (mode == DisplayMode.Billboard) mesh = QuadGenerator.GenerateQuadMesh();
					else mesh = SphereGenerator.GenerateSphereMesh(meshResolution);
					ComputeHelper.CreateArgsBuffer(ref argsBuffer, mesh, sim.positionBuffer.count);

					mat = mode switch
					{
						DisplayMode.Shaded3D => new Material(shaderShaded),
						DisplayMode.Billboard => new Material(shaderBillboard),
						_ => null
					};


					mat.SetBuffer("Positions", sim.positionBuffer);
						if (mode == DisplayMode.Billboard)
						{
							mat.SetInt("TrackLiquidMass", sim.trackLiquidMass ? 1 : 0);
							mat.SetBuffer("LiquidFraction", sim.LiquidFraction);
						}
					mat.SetBuffer("Velocities", sim.velocityBuffer);
					mat.SetBuffer("DebugBuffer", sim.debugBuffer);
					if (mode == DisplayMode.Billboard && sim.TemperatureRise != null)
					{
						mat.EnableKeyword("PARTICLE_TEMPERATURE");
						mat.SetBuffer("TemperatureRise", sim.TemperatureRise);
					}
				}
			}

			if (mat != null)
			{
				if (needsUpdate || previousColourMap != colourMap)
				{
					needsUpdate = false;
					previousColourMap = colourMap;
					TextureFromGradient(ref gradientTexture, gradientResolution, colourMap);
					mat.SetTexture("ColourMap", gradientTexture);
				}

				mat.SetFloat("scale", scale * 0.01f);
				mat.SetFloat("velocityMax", velocityDisplayMax);
				if (mode == DisplayMode.Billboard && sim.TemperatureRise != null)
					{
					mat.SetFloat("temperatureRiseMax", Mathf.Max(.1f, temperatureRiseDisplayMax));
					mat.SetFloat("temperatureView", showTemperature ? 1f : 0f);
				}

				Vector3 s = transform.localScale;
				transform.localScale = Vector3.one;
				var localToWorld = transform.localToWorldMatrix;
				transform.localScale = s;

				mat.SetMatrix("localToWorld", localToWorld);
			}
		}

		public static void TextureFromGradient(ref Texture2D texture, int width, Gradient gradient, FilterMode filterMode = FilterMode.Bilinear)
		{
			if (texture == null)
			{
				texture = new Texture2D(width, 1);
			}
			else if (texture.width != width)
			{
				texture.Reinitialize(width, 1);
			}

			if (gradient == null)
			{
				gradient = new Gradient();
				gradient.SetKeys(
					new GradientColorKey[] { new(Color.black, 0), new(Color.black, 1) },
					new GradientAlphaKey[] { new(1, 0), new(1, 1) }
				);
			}

			texture.wrapMode = TextureWrapMode.Clamp;
			texture.filterMode = filterMode;

			Color[] cols = new Color[width];
			for (int i = 0; i < cols.Length; i++)
			{
				float t = i / (cols.Length - 1f);
				cols[i] = gradient.Evaluate(t);
			}

			texture.SetPixels(cols);
			texture.Apply();
		}

		private void OnValidate()
		{
			needsUpdate = true;
		}

		void OnDestroy()
		{
			ComputeHelper.Release(argsBuffer);
			if (mesh != null) Destroy(mesh);
			if (mat != null) Destroy(mat);
			if (gradientTexture != null) Destroy(gradientTexture);
		}
	}
}