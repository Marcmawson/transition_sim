Shader "PhysicsSim/BeakerIceSurface"
{
    SubShader
    {
        Tags { "Queue"="Transparent-45" "RenderType"="Transparent" }
        GrabPass { "_IceBackground" }
        Pass
        {
            Tags { "LightMode"="ForwardBase" }
            ZWrite On
            CGPROGRAM
            #pragma target 3.0
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"
            #include "UnityLightingCommon.cginc"

            sampler2D _IceBackground;
            struct Varyings
            {
                float4 clip : SV_POSITION;
                float3 world : TEXCOORD0;
                float3 local : TEXCOORD1;
                float3 normal : TEXCOORD2;
            };

            Varyings vert(appdata_base input)
            {
                Varyings output;
                output.clip = UnityObjectToClipPos(input.vertex);
                output.local = input.vertex.xyz;
                output.world = mul(unity_ObjectToWorld, input.vertex).xyz;
                output.normal = UnityObjectToWorldNormal(input.normal);
                return output;
            }

            float4 frag(Varyings i) : SV_Target
            {
                float3 n = normalize(i.normal);
                float3 view = normalize(_WorldSpaceCameraPos - i.world);
                float3 reflected = reflect(-view, n);
                float3 bent = refract(-view, n, 1.0 / 1.31);
                float fresnel = .018 + .982 * pow(1 - saturate(dot(n, view)), 5);
                float scale = length(mul((float3x3)unity_ObjectToWorld, float3(1,0,0)));
                float4 grab = ComputeGrabScreenPos(UnityWorldToClipPos(i.world + (bent + view) * scale * .55));
                float2 uv = saturate(grab.xy / max(grab.w, 1e-5));

                // Estimate scattering from the refracted ray's chord through the cloudy core.
                float3 localRay = normalize(mul((float3x3)unity_WorldToObject, bent));
                float3 coreOffset = i.local - float3(.025,-.035,.02);
                float alongRay = max(0, -dot(coreOffset, localRay));
                float3 closest = coreOffset + localRay * alongRay;
                float grain = sin(41 * closest.x + 13 * closest.z) * sin(57 * closest.y - 21 * closest.z);
                float radius = .34 + .015 * grain;
                float chord = 2 * sqrt(max(0, radius * radius - dot(closest, closest)));
                float cloud = 1 - exp(-chord * 3.5 * min(scale / .048, 2));
                float3 throughIce = tex2D(_IceBackground, uv).rgb * float3(.96,.985,.99);
                throughIce = lerp(throughIce, float3(.72,.81,.86), cloud);

                float3 sky = lerp(float3(.41,.45,.49), float3(.49,.68,.92), saturate(reflected.y));
                float3 sun = normalize(_WorldSpaceLightPos0.xyz + float3(0,1e-6,0));
                float alignment = saturate(dot(reflected, sun));
                float feather = max(fwidth(alignment), .0001);
                float glint = smoothstep(.997 - feather, .9996 + feather, alignment);
                sky += _LightColor0.rgb * glint * saturate(dot(n, sun)) * 10;
                return float4(lerp(throughIce, sky, fresnel), 1);
            }
            ENDCG
        }
    }
}
