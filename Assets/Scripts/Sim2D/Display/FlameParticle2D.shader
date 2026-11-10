Shader "Instanced/FlameParticle2D"
{
    SubShader
    {
        Tags { "Queue"="Transparent+20" "RenderType"="Transparent" }
        Blend SrcAlpha One
        ZWrite Off
        Cull Off

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 4.5
            #include "UnityCG.cginc"

            StructuredBuffer<float4> FlameState;
            float particleSize;
            float airCollar;
            float burnerEnabled;

            struct appdata { float4 vertex : POSITION; float2 uv : TEXCOORD0; };
            struct v2f { float4 pos : SV_POSITION; float2 uv : TEXCOORD0; float4 data : TEXCOORD1; };

            v2f vert(appdata v, uint instanceID : SV_InstanceID)
            {
                float4 state = FlameState[instanceID];
                float ageT = saturate(state.z / max(state.w, 0.001));
                float size = particleSize * lerp(1.0, 0.55, ageT);
                float3 centre = float3(state.xy, -0.35);
                float3 world = centre + mul(unity_ObjectToWorld, v.vertex * size);

                v2f o;
                o.pos = UnityObjectToClipPos(mul(unity_WorldToObject, float4(world, 1)));
                o.uv = v.uv;
                o.data = float4(ageT, state.w, airCollar, burnerEnabled);
                return o;
            }

            fixed4 frag(v2f i) : SV_Target
            {
                if (i.data.w < 0.5) return 0;
                float2 p = (i.uv - 0.5) * 2;
                float r2 = dot(p, p);
                float glow = exp(-r2 * 3.5) * (1 - smoothstep(0.65, 1.0, r2));
                float lifeFade = 1.0 - smoothstep(0.45, 1.0, i.data.x);
                float3 orange = float3(1.0, 0.16, 0.015);
                float3 blue = float3(0.12, 0.48, 1.0);
                float3 hotCore = float3(1.0, 0.82, 0.34);
                float focus = saturate(i.data.z);
                float3 fuelColour = lerp(orange, blue, focus);
                float3 colour = lerp(fuelColour, hotCore, glow * 0.62);
                return float4(colour, glow * lifeFade * 0.78);
            }
            ENDCG
        }
    }
}
