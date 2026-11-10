using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using PhysicsSim.ThreePhase3D.Core;
using PhysicsSim.ThreePhase3D.Materials;
using PhysicsSim.ThreePhase3D.Verification;
using UnityEditor;
using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Editor
{
    public static class MaterialSurfaceVerificationMenu
    {
        [MenuItem("Physics Sim/Three Phase 3D/Run S02 expanded GPU material checks")]
        public static void RunGpu()
        {
            string root=Directory.GetParent(Application.dataPath).FullName;
            string output=Path.Combine(root,"ValidationRuns/local","s02-expanded-gpu-"+DateTime.UtcNow.ToString("yyyyMMddTHHmmss")+"-"+Guid.NewGuid().ToString("N"));Directory.CreateDirectory(output);
            var report=new GpuMaterialReport {schema="S02-GPU-TP-1",utc=DateTime.UtcNow.ToString("o"),device=SystemInfo.graphicsDeviceName,api=SystemInfo.graphicsDeviceType.ToString(),unity=Application.unityVersion};
            var rows=new List<MaterialComparison>();var hashes=new List<string>();
            try
            {
                if(!SystemInfo.supportsComputeShaders)throw new NotSupportedException("Compute shaders unavailable on actual device/API.");
                MaterialChecks.Execute(root);
                var shader=Resources.Load<ComputeShader>("ThreePhase3D/MaterialSurfaceLookup");
                if(shader==null)throw new InvalidOperationException("MaterialSurfaceLookup.compute did not import/load.");
                report.shader_sha256=RecordJson.FileHash(Path.Combine(root,"Assets/Resources/ThreePhase3D/MaterialSurfaceLookup.compute"));
                foreach(MaterialSurface surface in Enum.GetValues(typeof(MaterialSurface)))
                {
                    var d=MaterialSurfaceTable.Generate(surface);var table=new MaterialSurfaceTable(d);int width=table.Width;
                    byte[] bytes=RecordJson.Encode(d);hashes.Add(RecordJson.Hash(bytes));File.WriteAllBytes(Path.Combine(output,surface+"-TP.json"),bytes);
                    var queries=new Vector4[67];float min=(float)d.minimum_k,max=(float)(d.minimum_k+(d.temperature_count-1)*d.step_k),y=(float)d.water_mass_fraction;
                    queries[0]=new Vector4(min,80000,y,0);queries[1]=new Vector4(max,120000,y,0);
                    for(int i=2;i<63;i++)queries[i]=new Vector4(min+(max-min)*(i-1.73f)/62,80000+40000*((i*17)%61+.271f)/62,y,0);
                    queries[63]=new Vector4(min-.25f,101325,y,0);queries[64]=new Vector4(max,120001,y,0);
                    queries[65]=new Vector4(min,101325,y+.001f,0);queries[66]=new Vector4(float.NaN,101325,y,0);
                    ComputeBuffer values=null,input=null,result=null,status=null;
                    try
                    {
                        values=new ComputeBuffer(d.values.Length,4);input=new ComputeBuffer(queries.Length,16);result=new ComputeBuffer(queries.Length*width,4);status=new ComputeBuffer(queries.Length,4);
                        report.peak_owned_buffer_bytes=Math.Max(report.peak_owned_buffer_bytes,d.values.Length*4L+queries.Length*(16L+width*4L+4L));
                        values.SetData(d.values);input.SetData(queries);result.SetData(new float[queries.Length*width]);status.SetData(Enumerable.Repeat(uint.MaxValue,queries.Length).ToArray());
                        int kernel=shader.FindKernel("LookupSurface");
                        shader.SetBuffer(kernel,"TableValues",values);shader.SetBuffer(kernel,"Queries",input);shader.SetBuffer(kernel,"Results",result);shader.SetBuffer(kernel,"Status",status);
                        shader.SetInt("QueryCount",queries.Length);shader.SetInt("TemperatureCount",d.temperature_count);shader.SetInt("PressureCount",d.pressure_count);shader.SetInt("PropertyCount",width);
                        shader.SetFloat("MinimumK",min);shader.SetFloat("StepK",(float)d.step_k);shader.SetFloat("MinimumPa",80000);shader.SetFloat("StepPa",2000);shader.SetFloat("WaterMassFraction",y);
                        shader.GetKernelThreadGroupSizes(kernel,out uint threads,out uint ty,out uint tz);
                        if(threads!=64||ty!=1||tz!=1)throw new InvalidOperationException("Unexpected surface kernel group size.");
                        shader.Dispatch(kernel,(queries.Length+63)/64,1,1);
                        float[] actual=new float[queries.Length*width];uint[] flags=new uint[queries.Length];result.GetData(actual);status.GetData(flags);report.gpu_executed=true;
                        for(int i=0;i<queries.Length;i++)
                        {
                            bool valid=i<63;double[] expected=valid?MaterialSurfaceTable.Reference(surface,queries[i].x,queries[i].y,d.water_mass_fraction):new double[width];
                            Add(rows,surface+"-"+i,"status","1",flags[i],valid?0:1,0);
                            for(int j=0;j<width;j++)Add(rows,surface+"-"+i,d.quantities[j],d.units[j],actual[i*width+j],expected[j],valid?MaterialChecks.TableTolerance(j,expected[j]):0);
                        }
                    }
                    finally{values?.Release();input?.Release();result?.Release();status?.Release();}
                }
                report.passed=rows.Count>0&&rows.All(r=>r.passed);
                if(!report.passed)throw new InvalidOperationException("Expanded GPU material comparisons failed.");
            }
            catch(Exception e){report.error=e.ToString();throw;}
            finally
            {
                report.comparisons=rows.ToArray();report.table_sha256=hashes.ToArray();File.WriteAllBytes(Path.Combine(output,"gpu-results.json"),RecordJson.Encode(report));
                Debug.Log("S02 expanded GPU evidence: "+output+"; executed="+report.gpu_executed+"; passed="+report.passed);
            }
        }
        private static void Add(List<MaterialComparison> rows,string id,string quantity,string unit,double actual,double expected,double tolerance)
        {double error=Math.Abs(actual-expected);rows.Add(new MaterialComparison{case_id=id,quantity=quantity,unit=unit,source="Expanded GPU table vs verified CPU oracle",actual=actual,expected=expected,absolute_error=error,tolerance=tolerance,passed=!double.IsNaN(error)&&error<=tolerance});}
        public static void RunGpuBatch()
        {try{RunGpu();EditorApplication.Exit(0);}catch(Exception e){Debug.LogException(e);EditorApplication.Exit(1);}}
    }
}
