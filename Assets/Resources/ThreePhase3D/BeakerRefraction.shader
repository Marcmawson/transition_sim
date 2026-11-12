Shader "PhysicsSim/BeakerRefraction"
{
    Properties
    {
        _NormalReflectance ("Normal-incidence reflectance", Range(0,1)) = 0.03658
        _RefractiveIndex ("Refractive index", Float) = 1.473
    }
    SubShader
    {
        Tags { "Queue"="Transparent" "RenderType"="Transparent" }
        GrabPass { "_BeakerOpticalBackground" }
        CGPROGRAM
        #pragma surface surf StandardSpecular alpha:premul
        #pragma target 3.0

        sampler2D _BeakerOpticalBackground;
        float _RefractiveIndex;
        half _NormalReflectance;

        struct Input
        {
            float3 worldPos;
            float3 worldNormal;
        };

        void surf(Input input, inout SurfaceOutputStandardSpecular output)
        {
            float3 incident = normalize(input.worldPos - _WorldSpaceCameraPos);
            float3 normal = normalize(input.worldNormal);
            float3 transmitted = refract(incident, normal, 1.0 / _RefractiveIndex);
            // A short virtual ray produces a screen-space offset. It is not a wall thickness.
            float4 clip = UnityWorldToClipPos(input.worldPos + (transmitted - incident) * 0.01);
            float4 grab = ComputeGrabScreenPos(clip);
            float2 uv = saturate(grab.xy / max(grab.w, 1e-5));
            half grazing = 1 - saturate(dot(-incident, normal));
            half fresnel = _NormalReflectance + (1 - _NormalReflectance) * pow(grazing, 5);
            output.Albedo = 0;
            output.Specular = _NormalReflectance;
            output.Smoothness = 1;
            output.Emission = tex2D(_BeakerOpticalBackground, uv).rgb * (1 - fresnel);
            output.Alpha = 1;
        }
        ENDCG
    }
}
