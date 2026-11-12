using System;
using PhysicsSim.ThreePhase3D.Flow;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    public static class FogTransmissionGpuCheck
    {
        public static void Run()
        {
            var shader = Resources.Load<ComputeShader>("ThreePhase3D/FogTransmission");
            if (!SystemInfo.supportsComputeShaders || shader == null)
                throw new NotSupportedException("The S14 fog transmission check requires the compute shader.");

            var parcels = new[] { (8e-6, 290.0, 12), (10e-6, 290.0, 20), (12e-6, 290.0, 8) };
            float extinction = (float)SeededFogCell.VisibleExtinctionPerM(parcels, 1e-6);
            var samples = new[]
            {
                new Vector2(0, 0.01f), new Vector2(extinction, 0),
                new Vector2(extinction, 0.01f), new Vector2(extinction, 0.02f),
                new Vector2(extinction, 1 / extinction), new Vector2(extinction, 10 / extinction)
            };
            var result = new float[samples.Length];
            using var input = new ComputeBuffer(samples.Length, sizeof(float) * 2);
            using var output = new ComputeBuffer(samples.Length, sizeof(float));
            input.SetData(samples);
            Array.Fill(result, float.NaN);
            output.SetData(result);
            int kernel = shader.FindKernel("Transmit");
            shader.SetInt("SampleCount", samples.Length);
            shader.SetBuffer(kernel, "FogSamples", input);
            shader.SetBuffer(kernel, "Transmission", output);
            shader.Dispatch(kernel, (samples.Length + 31) / 32, 1, 1);
            output.GetData(result);

            double maxError = 0;
            for (int index = 0; index < result.Length; index++)
            {
                double expected = Math.Exp(-(double)samples[index].x * samples[index].y);
                double error = Math.Abs(result[index] - expected);
                if (!(result[index] >= 0 && result[index] <= 1 && error <= expected * 2e-6))
                    throw new InvalidOperationException($"S14 fog transmission sample {index} returned {result[index]:G8}, expected {expected:G8}.");
                maxError = Math.Max(maxError, error);
            }
            Debug.Log(FormattableString.Invariant(
                $"S14 GPU fog transmission passed: six samples, 1 cm transmission {result[2]:P8}, max absolute error {maxError:G5}."));
        }
    }
}
