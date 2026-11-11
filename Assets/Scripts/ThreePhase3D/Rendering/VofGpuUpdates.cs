using System;
using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Flow;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    internal sealed class VofGpuUpdates : IDisposable
    {
        private readonly ComputeShader shader;
        private readonly GeometricVof3D vof;
        private readonly VofGpuTransportLayout layout;
        private readonly int groups, limit, flux, updateCells, updateMomentum;
        private readonly VofFloatPair[] input, faces, cells, momentum;
        private readonly ComputeBuffer inputBuffer, budgetBuffer, faceBuffer, cellBuffer, momentumBuffer;
        private readonly List<ComputeBuffer> owned = new List<ComputeBuffer>();

        internal VofGpuUpdates(GeometricVof3D vof, ComputeShader shader)
        {
            this.vof = vof;
            this.shader = shader;
            if (shader == null) throw new InvalidOperationException("VofConservativeUpdates.compute did not import.");
            int count = vof.CellCount;
            groups = (int)((count + 63L) / 64);
            if (8L * count * (VofGpuTransportLayout.InputPlanes + VofGpuTransportLayout.FacePlanes +
                VofGpuTransportLayout.CellPlanes + VofGpuTransportLayout.MomentumPlanes + 2) > 512L * 1024 * 1024)
                throw new NotSupportedException("GPU VOF update buffers exceed 512 MiB.");
            limit = Find("LimitDonors");
            flux = Find("BuildFaceFlux");
            updateCells = Find("UpdateCells");
            updateMomentum = Find("UpdateMomentum");
            layout = new VofGpuTransportLayout(vof);
            input = new VofFloatPair[checked(VofGpuTransportLayout.InputPlanes * count)];
            faces = new VofFloatPair[checked(VofGpuTransportLayout.FacePlanes * count + 1)];
            cells = new VofFloatPair[checked(VofGpuTransportLayout.CellPlanes * count + 1)];
            momentum = new VofFloatPair[checked(VofGpuTransportLayout.MomentumPlanes * count + 1)];
            try
            {
                inputBuffer = Allocate(input.Length, false);
                budgetBuffer = Allocate(2 * count, false);
                faceBuffer = Allocate(faces.Length, true);
                cellBuffer = Allocate(cells.Length, true);
                momentumBuffer = Allocate(momentum.Length, true);
            }
            catch { Dispose(); throw; }
        }

        internal VofAdvance Advance(VofFields state, double[][] total, double[] outgoing, ComputeBuffer candidates,
            out double candidateRoundoff)
        {
            layout.Pack(state, total, outgoing, input);
            inputBuffer.SetData(input);
            shader.SetInt("Nx", vof.Grid.Cells.X);
            shader.SetInt("Ny", vof.Grid.Cells.Y);
            shader.SetInt("Nz", vof.Grid.Cells.Z);
            shader.SetInt("Count", vof.CellCount);
            SetPair("Volume", vof.Grid.CellVolume);
            SetPair("LiquidDensity", vof.LiquidDensity);
            SetPair("GasDensity", vof.GasDensity);

            Bind(limit, "InputFields", inputBuffer);
            Bind(limit, "CandidateFractions", candidates);
            Bind(limit, "Budget", budgetBuffer);
            shader.Dispatch(limit, groups, 1, 1);
            Bind(flux, "InputFields", inputBuffer);
            Bind(flux, "CandidateFractions", candidates);
            Bind(flux, "Budget", budgetBuffer);
            Bind(flux, "FaceData", faceBuffer);
            shader.Dispatch(flux, groups, 1, 1);
            Bind(updateCells, "InputFields", inputBuffer);
            Bind(updateCells, "FaceData", faceBuffer);
            Bind(updateCells, "CellData", cellBuffer);
            shader.Dispatch(updateCells, groups, 1, 1);
            Bind(updateMomentum, "InputFields", inputBuffer);
            Bind(updateMomentum, "FaceData", faceBuffer);
            Bind(updateMomentum, "CellData", cellBuffer);
            Bind(updateMomentum, "MomentumData", momentumBuffer);
            shader.Dispatch(updateMomentum, groups, 1, 1);

            faceBuffer.GetData(faces);
            cellBuffer.GetData(cells);
            momentumBuffer.GetData(momentum);
            return layout.Read(faces, cells, momentum, out candidateRoundoff);
        }

        private int Find(string name)
        {
            int kernel = shader.FindKernel(name);
            if (!shader.IsSupported(kernel))
                throw new NotSupportedException($"{name} is unsupported on {SystemInfo.graphicsDeviceType}.");
            return kernel;
        }

        private void SetPair(string name, double value)
        {
            var pair = new VofFloatPair(value);
            shader.SetInt(name + "High", BitConverter.ToInt32(BitConverter.GetBytes(pair.High), 0));
            shader.SetInt(name + "Low", BitConverter.ToInt32(BitConverter.GetBytes(pair.Low), 0));
        }

        private void Bind(int kernel, string name, ComputeBuffer buffer) => shader.SetBuffer(kernel, name, buffer);

        private ComputeBuffer Allocate(int count, bool guard)
        {
            var buffer = new ComputeBuffer(count, 8, ComputeBufferType.Structured);
            owned.Add(buffer);
            if (guard) buffer.SetData(new[] { new VofFloatPair(VofGpuTransportLayout.Guard) }, 0, count - 1, 1);
            return buffer;
        }

        public void Dispose()
        {
            foreach (var buffer in owned) buffer.Release();
            owned.Clear();
        }
    }
}
