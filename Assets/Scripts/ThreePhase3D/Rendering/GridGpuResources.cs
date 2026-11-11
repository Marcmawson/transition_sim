using System;
using System.Collections.Generic;
using System.Linq;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Geometry;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    public sealed class GridGpuResources : IDisposable
    {
        public readonly GridGpuPlan Plan;
        public readonly Grid3D Grid;
        private readonly Dictionary<string,ComputeBuffer> buffers=new Dictionary<string,ComputeBuffer>();
        public long LiveBytes {get;private set;}
        public bool Disposed {get;private set;}
        public GridGpuResources(CutCellGeometry geometry,long budgetBytes=512L*1024*1024)
        {
            Grid=geometry.Grid;Plan=new GridGpuPlan(Grid,budgetBytes);
            if(!SystemInfo.supportsComputeShaders)throw new NotSupportedException("Compute shaders unavailable on this actual graphics device.");
            try
            {
                foreach(var spec in Plan.Buffers)
                {
                    var buffer=new ComputeBuffer(spec.Count,spec.Stride,ComputeBufferType.Structured);buffers.Add(spec.Name,buffer);LiveBytes=checked(LiveBytes+spec.Bytes);
                    if(buffer.count!=spec.Count||buffer.stride!=spec.Stride)throw new InvalidOperationException("Actual buffer shape differs from plan.");
                    if(spec.Stride==16)buffer.SetData(new Vector4[spec.Count]);else if(spec.Name=="CellMask"||spec.Name=="Status")buffer.SetData(new uint[spec.Count]);else buffer.SetData(new float[spec.Count]);
                }
                Upload("FixedVolume",geometry.FixedVolumeM3);Upload("IceVolume",geometry.IceVolumeM3);Upload("FluidVolume",geometry.FluidVolumeM3);
                Upload("OpenAreaX",geometry.OpenAreaX);Upload("OpenAreaY",geometry.OpenAreaY);Upload("OpenAreaZ",geometry.OpenAreaZ);Upload("FixedDistance",geometry.FixedDistanceM);Upload("IceDistance",geometry.IceDistanceM);
                var fixedPatches=new Vector4[Grid.Cells.Count];var icePatches=new Vector4[Grid.Cells.Count];var mask=new uint[Grid.Cells.Count];
                for(int q=0;q<mask.Length;q++)
                {D3 a=geometry.FixedNormalArea[q],b=geometry.IceNormalArea[q];fixedPatches[q]=new Vector4((float)a.X,(float)a.Y,(float)a.Z,(float)geometry.FixedAreaM2[q]);icePatches[q]=new Vector4((float)b.X,(float)b.Y,(float)b.Z,(float)geometry.IceAreaM2[q]);Grid.Cells.Coordinates(q,out int i,out int j,out int k);mask[q]=Grid.Cells.Interior(i,j,k)?1u:0u;}
                buffers["FixedInterface"].SetData(fixedPatches);buffers["IceInterface"].SetData(icePatches);buffers["CellMask"].SetData(mask);
                foreach(var spec in Plan.Buffers)
                {if(spec.Stride==16)buffers[spec.Name].SetData(new[]{new Vector4(1234567,1234567,1234567,1234567)},0,spec.Count-1,1);else if(spec.Name=="CellMask"||spec.Name=="Status")buffers[spec.Name].SetData(new[]{0xc0decafeu},0,spec.Count-1,1);else buffers[spec.Name].SetData(new[]{1234567f},0,spec.Count-1,1);}
            }
            catch(Exception error){error.Data["S03_PartialAllocationPeakBytes"]=LiveBytes;Dispose();throw;}
        }
        private void Upload(string name,IEnumerable<double> data)=>buffers[name].SetData(data.Select(v=>(float)v).ToArray());
        public ComputeBuffer Buffer(string name)
        {if(Disposed)throw new ObjectDisposedException(nameof(GridGpuResources));if(!buffers.TryGetValue(name,out var buffer))throw new ArgumentException("Unknown grid buffer.");return buffer;}
        public void Dispatch(ComputeShader shader,string kernelName)
        {
            if(Disposed)throw new ObjectDisposedException(nameof(GridGpuResources));if(shader==null)throw new ArgumentNullException(nameof(shader));
            var bound=new Dictionary<string,GridBufferSpec>();foreach(string name in Plan.Required(kernelName)){var buffer=Buffer(name);bound.Add(name,new GridBufferSpec(name,buffer.count,buffer.stride));}Plan.VerifyBindings(kernelName,bound);
            int kernel=shader.FindKernel(kernelName);foreach(string name in Plan.Required(kernelName))shader.SetBuffer(kernel,name,Buffer(name));
            shader.SetInt("CellCount",Grid.Cells.Count);shader.SetInt("XFaceCount",Grid.XFaces.Count);shader.SetInt("YFaceCount",Grid.YFaces.Count);shader.SetInt("ZFaceCount",Grid.ZFaces.Count);
            shader.SetInt("Nx",Grid.Cells.X);shader.SetInt("Ny",Grid.Cells.Y);shader.SetInt("Nz",Grid.Cells.Z);shader.SetInt("Ghost",Grid.Cells.Ghost);
            shader.SetFloat("CellVolume",(float)Grid.CellVolume);shader.SetFloat("AreaX",(float)Grid.FaceArea(0));shader.SetFloat("AreaY",(float)Grid.FaceArea(1));shader.SetFloat("AreaZ",(float)Grid.FaceArea(2));
            shader.GetKernelThreadGroupSizes(kernel,out uint tx,out uint ty,out uint tz);if(tx!=64||ty!=1||tz!=1)throw new InvalidOperationException("Unexpected grid-kernel thread dimensions.");
            int count=kernelName=="ClearFields"?Math.Max(Grid.Cells.Count,Math.Max(Grid.XFaces.Count,Math.Max(Grid.YFaces.Count,Grid.ZFaces.Count))):Grid.Cells.Count;
            int groups=(int)((count+63L)/64),gx=Math.Min(groups,65535),gy=(groups+gx-1)/gx;shader.SetInt("DispatchWidth",gx*64);shader.Dispatch(kernel,gx,gy,1);
        }
        public void Dispose()
        {if(Disposed)return;foreach(var buffer in buffers.Values)buffer.Release();buffers.Clear();LiveBytes=0;Disposed=true;}
        public bool TailGuardsIntact()
        {
            foreach(var spec in Plan.Buffers)
            {
                if(spec.Stride==16){var value=new Vector4[1];Buffer(spec.Name).GetData(value,0,spec.Count-1,1);if(value[0]!=new Vector4(1234567,1234567,1234567,1234567))return false;}
                else if(spec.Name=="CellMask"||spec.Name=="Status"){var value=new uint[1];Buffer(spec.Name).GetData(value,0,spec.Count-1,1);if(value[0]!=0xc0decafeu)return false;}
                else{var value=new float[1];Buffer(spec.Name).GetData(value,0,spec.Count-1,1);if(value[0]!=1234567f)return false;}
            }
            return true;
        }
    }
}
