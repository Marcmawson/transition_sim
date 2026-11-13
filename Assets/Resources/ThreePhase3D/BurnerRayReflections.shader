Shader "PhysicsSim/BurnerRayReflections"
{
    Properties
    {
        _Color ("Steel colour", Color) = (.72,.74,.77,1)
        _Metallic ("Metallic", Range(0,1)) = 1
        _Glossiness ("Smoothness", Range(0,1)) = .82
        _BumpMap ("Brushed grain", 2D) = "bump" {}
    }

    SubShader
    {
        Tags { "RenderType"="Opaque" }
        CGPROGRAM
        #pragma surface surf Standard fullforwardshadows
        #pragma target 3.0
        #include "UnityCG.cginc"

        sampler2D _BumpMap;
        half4 _Color;
        half _Metallic;
        half _Glossiness;

        struct Input
        {
            float2 uv_BumpMap;
            float3 worldPos;
            INTERNAL_DATA
        };

        // Trace against a finite rectangle in stage coordinates. The nearest hit wins.
        float Panel(float3 origin, float3 ray, float3 centre, float2 halfSize, inout float nearest)
        {
            float3 normal = normalize(centre);
            float denominator = dot(ray, normal);
            if (denominator <= 1e-5) return 0;
            float distance = dot(centre - origin, normal) / denominator;
            if (distance <= 0 || distance >= nearest) return 0;

            float3 right = normalize(cross(float3(0, 1, 0), normal));
            float3 up = cross(normal, right);
            float3 offset = origin + distance * ray - centre;
            float2 uv = abs(float2(dot(offset, right), dot(offset, up))) / halfSize;
            if (any(uv >= 1)) return 0;
            nearest = distance;
            float2 edge = 1 - smoothstep(.76, 1, uv);
            return edge.x * edge.y;
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

            // Clip reflections at the worktop and at the back wall approximation.
            float nearest = 100;
            if (ray.y < -1e-5)
            {
                float distance = -origin.y / ray.y;
                float3 hit = origin + ray * distance;
                if (distance > 0 && abs(hit.x) <= 4 && hit.z >= -4 && hit.z <= .55)
                    nearest = distance;
            }
            if (ray.z > 1e-5)
            {
                float distance = (.91 - origin.z) / ray.z;
                float3 hit = origin + ray * distance;
                if (distance > 0 && abs(hit.x) <= 4 && hit.y >= .36 && hit.y <= 3)
                    nearest = min(nearest, distance);
            }

            half3 reflection = 0;
            float fill = Panel(origin, ray, float3(.33, .20, .45), float2(.09, .27), nearest);
            reflection = lerp(reflection, half3(.28,.36,.51), fill);
            float window = Panel(origin, ray, float3(-.45, .30, -.40), float2(.13, .29), nearest);
            reflection = lerp(reflection, half3(.76,.64,.47), window);
            output.Emission = reflection * _Color.rgb * _Metallic;
        }
        ENDCG
    }
    FallBack "Standard"
}
