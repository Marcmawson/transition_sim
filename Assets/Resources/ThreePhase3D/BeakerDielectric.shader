Shader "PhysicsSim/BeakerDielectric"
{
    Properties
    {
        _NormalReflectance ("Normal-incidence reflectance", Range(0,1)) = 0.03658
    }
    SubShader
    {
        Tags { "Queue"="Transparent" "RenderType"="Transparent" }
        CGPROGRAM
        #pragma surface surf StandardSpecular alpha:premul
        #pragma target 3.0

        half _NormalReflectance;
        struct Input { float3 worldPos; };

        void surf(Input input, inout SurfaceOutputStandardSpecular output)
        {
            output.Albedo = 0;
            output.Specular = _NormalReflectance;
            output.Smoothness = 1;
            output.Alpha = 0;
        }
        ENDCG
    }
}
