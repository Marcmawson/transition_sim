using System;
using System.Collections.Generic;
using System.Linq;

namespace PhysicsSim.ThreePhase3D.Core
{
    public sealed class GridBufferSpec
    {
        public readonly string Name;public readonly int Count,Stride;
        public long Bytes=>checked((long)Count*Stride);
        public GridBufferSpec(string name,int count,int stride)
        {if(string.IsNullOrWhiteSpace(name)||count<1||stride<4||stride%4!=0||stride>=2048)throw new ArgumentException("Invalid structured-buffer name/count/stride.");Name=name;Count=count;Stride=stride;}
    }
    // Physics-ID: CORE-GRID-GPU-RESOURCE-CONTRACT
    // Implements: R055 R053
    // Model: Specification.md section9; per-kernel structured-buffer binding registry
    // Sources: Unity2022.3 ComputeBuffer and ComputeShader APIs; project resource contract
    // Source-Locators: S03-Grid-Geometry.md GPU resources; Unity ComputeBufferType.Structured
    // Inputs-SI: grid extents [1], allocation budget [byte]
    // Outputs-SI: explicit count/stride/owned-byte plan and required bindings
    // State-Ownership: one plan per grid allocation; runtime owner disposes all buffers
    // Conservation: no physical inventory altered; all extents include allocated ghost cells
    // Discretisation: scalar float/uint stride4, normal-area/results float4 stride16
    // Validity: checked index/byte arithmetic, no partial allocation on failed preflight
    // Validation: V35
    // Evidence: PENDING
    // Status: IMPLEMENTED_UNVERIFIED
    // Limitations: S03 fields/check kernels only; future kernels must declare their own bindings
    public sealed class GridGpuPlan
    {
        public readonly IReadOnlyList<GridBufferSpec> Buffers;
        public readonly long OwnedBytes;
        private readonly Dictionary<string,string[]> kernels=new Dictionary<string,string[]> {
            {"ClearFields",new[]{"CellField","VelocityX","VelocityY","VelocityZ"}},
            {"ProbeMetrics",new[]{"FixedVolume","IceVolume","FluidVolume","OpenAreaX","OpenAreaY","OpenAreaZ","FixedInterface","IceInterface","FixedDistance","IceDistance","CellMask","Status","Results"}}
        };
        public GridGpuPlan(Grid3D grid,long budgetBytes=512L*1024*1024)
        {
            var specs=new List<GridBufferSpec>();int cells=grid.Cells.Count;
            Action<string,int,int> add=(name,logical,stride)=>specs.Add(new GridBufferSpec(name,checked(logical+1),stride));
            // Every allocation includes one tail canary beyond its logical grid extent.
            foreach(string name in new[]{"FixedVolume","IceVolume","FluidVolume","FixedDistance","IceDistance","CellMask","Status","CellField"})add(name,cells,4);
            foreach(string name in new[]{"FixedInterface","IceInterface","Results"})add(name,cells,16);
            add("OpenAreaX",grid.XFaces.Count,4);add("OpenAreaY",grid.YFaces.Count,4);add("OpenAreaZ",grid.ZFaces.Count,4);
            add("VelocityX",grid.XFaces.Count,4);add("VelocityY",grid.YFaces.Count,4);add("VelocityZ",grid.ZFaces.Count,4);
            long bytes=0;foreach(var spec in specs)bytes=checked(bytes+spec.Bytes);if(budgetBytes<0||bytes>budgetBytes)throw new InvalidOperationException("Grid GPU allocation exceeds declared budget: "+bytes+" bytes required.");OwnedBytes=bytes;Buffers=specs.AsReadOnly();
        }
        public IReadOnlyList<string> Required(string kernel)
        {if(!kernels.TryGetValue(kernel,out var required))throw new ArgumentException("Unregistered kernel: "+kernel);return Array.AsReadOnly(required);}
        public void VerifyBindings(string kernel,IReadOnlyDictionary<string,GridBufferSpec> actual)
        {
            foreach(string name in Required(kernel))
            {var expected=Buffers.First(s=>s.Name==name);if(actual==null||!actual.TryGetValue(name,out var bound)||bound.Count!=expected.Count||bound.Stride!=expected.Stride)throw new InvalidOperationException("Missing or incompatible "+kernel+" buffer: "+name);}
        }
    }
}
