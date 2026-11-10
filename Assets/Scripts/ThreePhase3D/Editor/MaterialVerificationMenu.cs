using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.Serialization;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Materials;
using PhysicsSim.ThreePhase3D.Verification;
using UnityEditor;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Editor
{
    [DataContract] public sealed class GpuMaterialReport
    {
        [DataMember] public string schema="S02-GPU-1",utc,device,api,unity,error;
        [DataMember] public bool gpu_executed,passed;
        [DataMember] public string shader_sha256;
        [DataMember] public string[] table_sha256;
        [DataMember] public long peak_owned_buffer_bytes;
        [DataMember] public MaterialComparison[] comparisons;
    }
    public static class MaterialVerificationMenu
    {
        [MenuItem("Physics Sim/Three Phase 3D/Run S02 CPU material checks")]
        public static void RunCpu()
        {Debug.Log("S02 material evidence: "+MaterialChecks.Execute(Directory.GetParent(Application.dataPath).FullName));}
        public static void RunCpuBatch()
        {try{RunCpu();EditorApplication.Exit(0);}catch(Exception e){Debug.LogException(e);EditorApplication.Exit(1);}}

        [MenuItem("Physics Sim/Three Phase 3D/Run S02 GPU table checks")]
        public static void RunGpu()
        {
            string root=Directory.GetParent(Application.dataPath).FullName;
            string output=Path.Combine(root,"ValidationRuns/local","s02-gpu-"+DateTime.UtcNow.ToString("yyyyMMddTHHmmss")+"-"+Guid.NewGuid().ToString("N"));Directory.CreateDirectory(output);
            var report=new GpuMaterialReport {utc=DateTime.UtcNow.ToString("o"),device=SystemInfo.graphicsDeviceName,api=SystemInfo.graphicsDeviceType.ToString(),unity=Application.unityVersion};
            var rows=new List<MaterialComparison>();var hashes=new List<string>();
            try
            {
                if(!SystemInfo.supportsComputeShaders)throw new NotSupportedException("Compute shaders unavailable on this actual device/API.");
                // No GPU results are interpreted until the independent official CPU checks pass.
                MaterialChecks.Execute(root);
                var shader=Resources.Load<ComputeShader>("ThreePhase3D/WaterPropertyLookup");
                if(shader==null)throw new InvalidOperationException("WaterPropertyLookup.compute did not import/load.");
                report.shader_sha256=RecordJson.FileHash(Path.Combine(root,"Assets/Resources/ThreePhase3D/WaterPropertyLookup.compute"));
                foreach(var branch in new[]{WaterBranch.Liquid,WaterBranch.Vapour})
                {
                    var data=WaterPropertyTable.Generate(branch);var table=new WaterPropertyTable(data);hashes.Add(RecordJson.Hash(RecordJson.Encode(data)));
                    File.WriteAllBytes(Path.Combine(output,branch+"-table.json"),RecordJson.Encode(data));
                    // 67 queries require two thread groups: endpoint checks and a partial final group.
                    var queries=new Vector2[67];
                    queries[0]=new Vector2((float)table.MinimumK,101325);queries[1]=new Vector2((float)table.MaximumK,101325);
                    for(int i=2;i<63;i++)queries[i]=new Vector2((float)(table.MinimumK+(table.MaximumK-table.MinimumK)*(i-1.73)/62),101325);
                    queries[63]=new Vector2((float)table.MinimumK-.25f,101325);queries[64]=new Vector2((float)table.MaximumK+.25f,101325);
                    queries[65]=new Vector2((float)table.MinimumK,100000);queries[66]=new Vector2(float.NaN,101325);
                    ComputeBuffer values=null,input=null,result=null,status=null;
                    try
                    {
                        values=new ComputeBuffer(data.values.Length,4);input=new ComputeBuffer(queries.Length,8);result=new ComputeBuffer(queries.Length*WaterPropertyTable.Width,4);status=new ComputeBuffer(queries.Length,4);
                        report.peak_owned_buffer_bytes=Math.Max(report.peak_owned_buffer_bytes,data.values.Length*4L+queries.Length*(8L+WaterPropertyTable.Width*4L+4L));
                        values.SetData(data.values);input.SetData(queries);result.SetData(new float[queries.Length*WaterPropertyTable.Width]);status.SetData(Enumerable.Repeat(uint.MaxValue,queries.Length).ToArray());
                        int kernel=shader.FindKernel("Lookup");
                        shader.SetBuffer(kernel,"TableValues",values);shader.SetBuffer(kernel,"Queries",input);shader.SetBuffer(kernel,"Results",result);shader.SetBuffer(kernel,"Status",status);
                        shader.SetInt("QueryCount",queries.Length);shader.SetInt("TableCount",table.Count);shader.SetFloat("MinimumK",(float)table.MinimumK);shader.SetFloat("StepK",(float)table.StepK);shader.SetFloat("PressurePa",101325);
                        shader.GetKernelThreadGroupSizes(kernel,out uint threads,out uint ty,out uint tz);
                        if(threads!=64||ty!=1||tz!=1)throw new InvalidOperationException("Unexpected Lookup kernel group size.");
                        shader.Dispatch(kernel,(queries.Length+63)/64,1,1);
                        float[] actual=new float[queries.Length*WaterPropertyTable.Width];uint[] flags=new uint[queries.Length];result.GetData(actual);status.GetData(flags);report.gpu_executed=true;
                        for(int i=0;i<queries.Length;i++)
                        {
                            bool valid=i<63;double[] expected=valid?WaterPropertyTable.EvaluateReference(new Water95(),queries[i].x,101325,branch):new double[WaterPropertyTable.Width];
                            rows.Add(new MaterialComparison {case_id=branch+"-"+i,quantity="status",unit="1",source="GPU range/dispatch coverage",actual=flags[i],expected=valid?0:1,tolerance=0,absolute_error=Math.Abs(flags[i]-(valid?0.0:1.0)),passed=flags[i]==(valid?0u:1u)});
                            for(int j=0;j<WaterPropertyTable.Width;j++)
                            {
                                double measured=actual[i*WaterPropertyTable.Width+j],error=Math.Abs(measured-expected[j]),tolerance=valid?MaterialChecks.TableTolerance(j,expected[j]):0;
                                rows.Add(new MaterialComparison {case_id=branch+"-"+i,quantity=WaterPropertyTable.Quantities[j],unit=WaterPropertyTable.Units[j],source="Official-check-verified CPU oracle",actual=measured,expected=expected[j],absolute_error=error,tolerance=tolerance,passed=!double.IsNaN(error)&&error<=tolerance});
                            }
                        }
                    }
                    finally {values?.Release();input?.Release();result?.Release();status?.Release();}
                }
                report.passed=rows.Count>0&&rows.All(r=>r.passed);
                if(!report.passed)throw new InvalidOperationException("GPU interpolation/guard comparison failed.");
            }
            catch(Exception e) {report.error=e.ToString();throw;}
            finally
            {
                report.comparisons=rows.ToArray();report.table_sha256=hashes.ToArray();
                File.WriteAllBytes(Path.Combine(output,"gpu-results.json"),RecordJson.Encode(report));
                Debug.Log("S02 GPU evidence: "+output+"; executed="+report.gpu_executed+"; passed="+report.passed);
            }
        }
        public static void RunGpuBatch()
        {try{RunGpu();EditorApplication.Exit(0);}catch(Exception e){Debug.LogException(e);EditorApplication.Exit(1);}}
    }
}
