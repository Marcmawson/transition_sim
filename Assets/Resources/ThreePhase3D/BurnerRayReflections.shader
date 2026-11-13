Shader "PhysicsSim/BurnerRayReflections"
{
    Properties
    {
        _Color ("Steel colour", Color) = (.72,.74,.77,1)
        _Metallic ("Metallic", Range(0,1)) = 1
        _Glossiness ("Smoothness", Range(0,1)) = .82
        _BumpMap ("Brushed grain", 2D) = "bump" {}
        [HideInInspector] _BenchMin ("Bench min", Vector) = (0,0,0,0)
        [HideInInspector] _BenchMax ("Bench max", Vector) = (0,0,0,0)
        [HideInInspector] _GroundY ("Ground height", Float) = -.84
        [HideInInspector] _BenchColour ("Bench colour", Color) = (1,1,1,1)
        [HideInInspector] _GroundColour ("Ground colour", Color) = (1,1,1,1)
    }

    SubShader
    {
        Tags { "RenderType"="Opaque" }
        CGPROGRAM
        #pragma surface surf Standard fullforwardshadows noforwardadd
        #pragma target 3.0
        #include "UnityCG.cginc"

        sampler2D _BumpMap;
        half4 _Color;
        half _Metallic;
        half _Glossiness;
        float3 _BenchMin, _BenchMax;
        float _GroundY;
        half4 _BenchColour, _GroundColour;

        struct Input
        {
            float2 uv_BumpMap;
            float3 worldPos;
            INTERNAL_DATA
        };

        // Slab intersection in the same coordinates as the generated bench.
        float BenchHit(float3 origin, float3 ray)
        {
            float3 signRay = step(0, ray) * 2 - 1;
            float3 inverseRay = signRay / max(abs(ray), 1e-6);
            float3 a = (_BenchMin - origin) * inverseRay;
            float3 b = (_BenchMax - origin) * inverseRay;
            float3 lo = min(a, b);
            float3 hi = max(a, b);
            float entry = max(max(lo.x, lo.y), lo.z);
            float exit = min(min(hi.x, hi.y), hi.z);
            return exit >= max(entry, .0001) && entry > .0001 ? entry : 1e6;
        }

        void surf(Input input, inout SurfaceOutputStandard output)
        {
            output.Normal = UnpackNormal(tex2D(_BumpMap, input.uv_BumpMap));
            output.Albedo = _Color.rgb;
            output.Metallic = _Metallic;
            output.Smoothness = _Glossiness;
            output.Alpha = 1;

            // The rasterized steel fragment is the primary camera-ray hit.
            float3 incoming = normalize(input.worldPos - _WorldSpaceCameraPos);
            float3 reflected = reflect(incoming, normalize(WorldNormalVector(input, output.Normal)));
            float3 origin = mul(unity_WorldToObject, float4(input.worldPos, 1)).xyz;
            float3 ray = normalize(mul((float3x3)unity_WorldToObject, reflected));

            float benchDistance = BenchHit(origin, ray);
            float groundDistance = ray.y < -1e-5 ? (_GroundY - origin.y) / ray.y : 1e6;
            float2 floorPoint = origin.xz + ray.xz * groundDistance;
            bool hitsGround = groundDistance > 0 && groundDistance < 1e6 && all(abs(floorPoint) <= 16);
            half3 reflection;
            if (benchDistance < (hitsGround ? groundDistance : 1e6))
                reflection = _BenchColour.rgb * .45;
            else if (hitsGround)
                reflection = _GroundColour.rgb * .42;
            else
                reflection = lerp(half3(.47,.59,.75), half3(.14,.32,.62), saturate(ray.y)) * .42;
            output.Emission = reflection * _Color.rgb * _Metallic;
        }
        ENDCG
    }
    FallBack "Standard"
}
