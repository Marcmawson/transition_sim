Shader "Fluid/BillboardFoam"
{
    Properties
    {
        [HideInInspector] _SrcBlend ("Source blend", Float) = 1
        [HideInInspector] _DstBlend ("Destination blend", Float) = 0
        [HideInInspector] _ZWrite ("Depth write", Float) = 1
        [HideInInspector] _VisibleSpray ("Visible spray", Float) = 0
    }
    SubShader
    {
        Tags
        {
            "Queue"="Geometry"
        }
        ZWrite [_ZWrite]
        Blend [_SrcBlend] [_DstBlend]
        ZTest LEqual
        Cull Off

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 4.5
            #include "UnityCG.cginc"

            struct FoamParticle
            {
                float3 position;
                float3 velocity;
                float lifetime;
                float scale;
            float _VisibleSpray;
            };

            StructuredBuffer<FoamParticle> Particles;
            float scale;
            float debugParam;
            int bubbleClassifyThreshold;
            int sprayClassifyThreshold;

            float Remap01(float val, float minVal, float maxVal)
            {
                return saturate((val - minVal) / (maxVal - minVal));
            }

            struct v2f
            {
                float4 pos : SV_POSITION;
                float2 uv : TEXCOORD0;
                float3 posWorld : TEXCOORD1;
                float life : TEXCOORD2;
            };

            v2f vert(appdata_base v, uint instanceID : SV_InstanceID)
            {
                v2f o;
                FoamParticle particle = Particles[instanceID];

                // Scale particle based on age
                const float remainingLifetimeDissolveStart = 3;
                float dissolveScaleT = saturate(particle.lifetime / (_VisibleSpray > .5 ? .12 : remainingLifetimeDissolveStart));
                float speed = length(particle.velocity);
                float velScale = lerp(0.6, 1, Remap01(speed, 1, 3));
                float vertScale = scale * 2 * dissolveScaleT * particle.scale * velScale;

                // Quad face camera
                float3 worldCentre = particle.position;
                float3 vertOffset = v.vertex * vertScale;
                float3 camUp = unity_CameraToWorld._m01_m11_m21;
                float3 camRight = unity_CameraToWorld._m00_m10_m20;
                float3 vertPosWorld = worldCentre + camRight * vertOffset.x + camUp * vertOffset.y;
                
                o.pos = mul(UNITY_MATRIX_VP, float4(vertPosWorld, 1));
                o.uv = v.texcoord;
                o.posWorld = worldCentre;
                o.life = dissolveScaleT;

                return o;
            }

            float LinearDepthToUnityDepth(float linearDepth)
            {
                float depth01 = (linearDepth - _ProjectionParams.y) / (_ProjectionParams.z - _ProjectionParams.y);
                return (1.0 - (depth01 * _ZBufferParams.y)) / (depth01 * _ZBufferParams.x);
            }

            float4 frag(v2f i) : SV_Target
            {
                float2 centreOffset = (i.uv - 0.5) * 2;
                float sqrDst = dot(centreOffset, centreOffset);
                if (sqrDst > 1) discard;

                if (_VisibleSpray > .5)
                {
                    float edge = saturate((1 - sqrDst) * 3);
                    float highlight = saturate(1 - length(centreOffset + float2(.28,.28)) * 1.8);
                    return float4(lerp(float3(.57,.75,.85), float3(.97,.99,1), highlight),
                        edge * i.life * .65);
                }

                float linearDepth = abs(mul(unity_MatrixV, float4(i.posWorld, 1)).z);
                return float4(1, LinearDepthToUnityDepth(linearDepth), linearDepth, 1);
            }
            ENDCG
        }
    }
}