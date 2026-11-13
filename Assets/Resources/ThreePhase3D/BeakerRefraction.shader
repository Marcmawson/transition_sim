Shader "PhysicsSim/BeakerRefraction"
{
    Properties
    {
        _StudioReflections ("Studio reflections", Float) = 0
        _NormalReflectance ("Normal-incidence reflectance", Range(0,1)) = 0.03658
        _RefractiveIndex ("Refractive index", Float) = 1.473
        _AbsorptionPerM ("RGB absorption per metre", Vector) = (0,0,0,0)
        _WaterCylinder ("Water centre XZ, bottom Y, radius", Vector) = (0,0,0,0)
        _WaterTopY ("Water top Y", Float) = 0
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
        float4 _AbsorptionPerM;
        float4 _WaterCylinder;
        float _WaterTopY;
        half _NormalReflectance;
        float _StudioReflections;

        struct Input
        {
            float3 worldPos;
            float3 worldNormal;
        };

        float WaterPathM(float3 position, float3 direction)
        {
            float2 offset = position.xz - _WaterCylinder.xy;
            float radialSpeed = dot(direction.xz, direction.xz);
            float radialDot = dot(offset, direction.xz);
            float discriminant = radialDot * radialDot + radialSpeed *
                (_WaterCylinder.w * _WaterCylinder.w - dot(offset, offset));
            float side = radialSpeed > 1e-8
                ? (-radialDot + sqrt(max(0, discriminant))) / radialSpeed : 1e6;
            float cap = abs(direction.y) > 1e-6
                ? ((direction.y > 0 ? _WaterTopY : _WaterCylinder.z) - position.y) / direction.y : 1e6;
            return max(0, min(side, cap));
        }

        // Evaluate the reflected ray against finite rectangular studio emitters.
        float Softbox(float3 ray, float3 centre, float2 halfSize)
        {
            centre = normalize(centre);
            float3 right = normalize(cross(float3(0, 1, 0), centre));
            float3 up = cross(centre, right);
            float depth = dot(ray, centre);
            if (depth <= 0) return 0;
            float2 projected = abs(float2(dot(ray, right), dot(ray, up)) / depth);
            float2 fade = 1 - smoothstep(halfSize * .85, halfSize * 1.08, projected);
            return fade.x * fade.y;
        }

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
            float3 transmission = 1;
            if (_WaterCylinder.w > 0)
                transmission = exp(-_AbsorptionPerM.rgb * WaterPathM(input.worldPos, transmitted));
            output.Emission = tex2D(_BeakerOpticalBackground, uv).rgb * (1 - fresnel) * transmission;
            if (_StudioReflections > 0)
            {
                float3 reflected = reflect(incident, normal);
                float3 studio = float3(3.8, 3.3, 2.65) *
                    Softbox(reflected, float3(-.65, .35, -.75), float2(.15, .65));
                studio += float3(1.2, 1.65, 2.3) *
                    Softbox(reflected, float3(.8, .2, .25), float2(.095, .55));
                studio += float3(1.9, 1.8, 1.65) *
                    Softbox(reflected, float3(.15, 1, -.22), float2(.48, .23));
                output.Emission += studio * fresnel * _StudioReflections;
            }
            output.Alpha = 1;
        }
        ENDCG
    }
}
