Shader "PhysicsSim/WornOak"
{
    Properties
    {
        _Color ("Oak tone", Color) = (.47,.32,.19,1)
        _Glossiness ("Smoothness", Range(0,1)) = .23
        _Metallic ("Metallic", Range(0,1)) = 0
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        CGPROGRAM
        #pragma surface surf Standard fullforwardshadows
        #pragma target 3.0

        half4 _Color;
        half _Glossiness;
        float4x4 _StageWorldToLocal;

        struct Input { float3 worldPos; };

        float Hash(float2 p)
        {
            return frac(sin(dot(p, float2(127.1, 311.7))) * 43758.5453);
        }

        float Noise(float2 p)
        {
            float2 cell = floor(p);
            float2 f = frac(p);
            f = f * f * (3 - 2 * f);
            return lerp(lerp(Hash(cell), Hash(cell + float2(1,0)), f.x),
                lerp(Hash(cell + float2(0,1)), Hash(cell + 1), f.x), f.y);
        }

        float Scorch(float2 position, float2 centre, float radius)
        {
            float2 offset = (position - centre) * float2(1, 1.13);
            float r = length(offset) / radius + (Noise(position * 180) - .5) * .13;
            float ring = 1 - smoothstep(.06, .25, abs(r - .83));
            return (.17 * (1 - smoothstep(.5, .98, r)) + .46 * ring)
                * lerp(.48, 1, Noise(position * 97));
        }

        void surf(Input input, inout SurfaceOutputStandard output)
        {
            float3 stage = mul(_StageWorldToLocal, float4(input.worldPos, 1)).xyz;
            float2 p = stage.xz + float2(0, stage.y * .72);
            float board = floor((stage.z + .33) / .132);
            float2 grainPosition = p + float2(0, board * .37);
            float2 knotOffset = p - float2(.18, -.087);
            float knotRadius = length(knotOffset * float2(.35, 1));
            float bend = .013 * exp(-knotRadius * 34);
            float grainPhase = 190 * (grainPosition.y + bend + .008 * Noise(grainPosition * float2(4,27)));
            float rings = .5 + .5 * sin(grainPhase);
            float ringContrast = 1 - smoothstep(.15, .65, fwidth(grainPhase));
            float pores = Noise(grainPosition * float2(44, 850));
            float worn = Noise(p * float2(8, 25));
            float scratch = smoothstep(.76, .94, Noise(p * float2(22, 440)));
            float scorch = max(Scorch(stage.xz, float2(-.102, -.062), .037),
                max(Scorch(stage.xz, float2(.156, .058), .049) * .69,
                    Scorch(stage.xz, float2(.081, -.118), .024) * .62));
            scorch *= smoothstep(-.003, 0, stage.y);
            half3 oak = _Color.rgb * (.77 + .27 * worn + .12 * Hash(float2(board, 3)));
            oak *= 1 - .21 * rings * ringContrast - .11 * pores - .17 * exp(-knotRadius * 95);
            oak = lerp(oak, half3(.49,.40,.29), .23 * worn + .16 * scratch);
            output.Albedo = lerp(oak, oak * half3(.31,.24,.18), scorch);
            output.Smoothness = _Glossiness * lerp(.58, 1.09, worn) * (1 - scorch * .4);
            output.Alpha = 1;
        }
        ENDCG
    }
    FallBack "Standard"
}
