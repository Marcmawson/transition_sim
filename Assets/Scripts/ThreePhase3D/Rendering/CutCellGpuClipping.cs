using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Flow;
using PhysicsSim.ThreePhase3D.Geometry;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    internal sealed class CutCellGpuClipping
    {
        private readonly ComputeShader shader;
        private readonly Grid3D grid;
        private readonly int kernel;
        private readonly Dictionary<int, Vector4[]> cells = new Dictionary<int, Vector4[]>();
        private long cachedBytes;
        internal int Dispatches { get; private set; }

        internal CutCellGpuClipping(Grid3D grid, ComputeShader shader)
        {
            this.grid = grid;
            this.shader = shader;
            if (shader == null) throw new InvalidOperationException("Cut-cell clipping compute shader is unavailable.");
            kernel = shader.FindKernel("ClipCutFaces");
            if (!shader.IsSupported(kernel))
                throw new NotSupportedException("Cut-cell clipping is unsupported on " + SystemInfo.graphicsDeviceType);
        }

        internal double[] Clip(CutCellVofGeometry geometry, IReadOnlyList<CutCellClipJob> requests)
        {
            int count = requests.Count;
            int dispatch = (int)((count + 63L) / 64);
            if (dispatch > 65535) throw new NotSupportedException("Cut-cell clipping dispatch exceeds one-dimensional limits.");
            var tetra = new List<Vector4>();
            var ranges = new Dictionary<int, int>();
            var jobs = new Job[count];
            var normal = new Vector4[count];
            var volume = new Vector2[count];
            double full = grid.CellVolume;
            for (int n = 0; n < count; n++)
            {
                var request = requests[n];
                if (!cells.TryGetValue(request.Cell, out var vertices))
                {
                    var packed = new List<Vector4>();
                    geometry.PackTetrahedra(request.Cell, (a,b,c,d,sign) =>
                    {
                        packed.Add(Vertex(a,(float)sign));
                        packed.Add(Vertex(b,0));
                        packed.Add(Vertex(c,0));
                        packed.Add(Vertex(d,0));
                    });
                    long bytes = packed.Count * 16L;
                    if (cachedBytes + bytes > 256L * 1024 * 1024)
                        throw new NotSupportedException("Cut-cell clipping geometry exceeds the 256 MiB cache budget.");
                    vertices = packed.ToArray();
                    cells.Add(request.Cell, vertices);
                    cachedBytes += bytes;
                }
                if (!ranges.TryGetValue(request.Cell, out int first))
                {
                    first = tetra.Count / 4;
                    tetra.AddRange(vertices);
                    ranges.Add(request.Cell, first);
                }
                jobs[n] = new Job { First = first, Count = vertices.Length/4,
                    Axis = request.Axis, Direction = request.Positive ? -1 : 1 };
                D3 p = request.Normal;
                if (p.Length == 0) p = new D3(1,0,0);
                var scaled = new D3(p.X*grid.Spacing.X, p.Y*grid.Spacing.Y, p.Z*grid.Spacing.Z).Unit();
                double fraction = request.FluidVolume / full;
                normal[n] = Vertex(scaled, (float)(request.Alpha*fraction));
                volume[n] = new Vector2((float)fraction, (float)(request.SweptVolume/full));
            }
            if (tetra.Count*16L + count*56L > 512L*1024*1024)
                throw new NotSupportedException("Cut-cell clipping upload exceeds the 512 MiB payload budget.");

            using (var vertices = Upload(tetra.ToArray(),16))
            using (var descriptors = Upload(jobs,16))
            using (var planes = Upload(normal,16))
            using (var volumes = Upload(volume,8))
            using (var results = new ComputeBuffer(count+1,16,ComputeBufferType.Structured))
            {
                var guard = new Vector4(-1,-1,-1,-1);
                results.SetData(new[] { guard },0,count,1);
                shader.SetInt("JobCount", count);
                shader.SetBuffer(kernel,"TetraVertices",vertices);
                shader.SetBuffer(kernel,"Jobs",descriptors);
                shader.SetBuffer(kernel,"Normals",planes);
                shader.SetBuffer(kernel,"Volumes",volumes);
                shader.SetBuffer(kernel,"Results",results);
                shader.Dispatch(kernel,dispatch,1,1);
                var data = new Vector4[count+1];
                results.GetData(data);
                var tail = data[count];
                if (tail.x != guard.x || tail.y != guard.y || tail.z != guard.z || tail.w != guard.w)
                    throw new InvalidOperationException("Cut-cell clipping overwrote its output guard.");
                var output = new double[count];
                for (int n = 0; n < count; n++)
                {
                    var value = data[n];
                    if (!SiUnits.IsFinite(value.x) || !SiUnits.IsFinite(value.y) ||
                        !SiUnits.IsFinite(value.z) || !SiUnits.IsFinite(value.w) ||
                        Math.Abs(value.y)>2e-6 || Math.Abs(value.w)>2e-6 ||
                        Math.Abs(value.z)>Math.Max(5e-8,2e-4*requests[n].SweptVolume/full))
                        throw new InvalidOperationException($"GPU clipping volume inversion failed for face {requests[n].Face}: " +
                            $"interface={value.y:E3}, slab={value.z:E3}, accessible={value.w:E3}, target slab={volume[n].y:E3}.");
                    output[n] = value.x * full;
                }
                Dispatches++;
                return output;
            }
        }

        private static Vector4 Vertex(D3 p, float sign) => new Vector4((float)p.X,(float)p.Y,(float)p.Z,sign);

        private static ComputeBuffer Upload(Array values,int stride)
        {
            var buffer = new ComputeBuffer(values.Length,stride,ComputeBufferType.Structured);
            try { buffer.SetData(values); return buffer; }
            catch { buffer.Release(); throw; }
        }

        [StructLayout(LayoutKind.Sequential)]
        private struct Job { public int First, Count, Axis, Direction; }
    }
}
