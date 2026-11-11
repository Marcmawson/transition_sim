Shader "PhysicsSim/WaterTemperatureSurface"
{
    SubShader
    {
        Tags { "Queue"="Geometry" "RenderType"="Opaque" }
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            struct Attributes
            {
                float4 position : POSITION;
                float3 normal : NORMAL;
                fixed4 colour : COLOR;
            };

            struct Interpolators
            {
                float4 position : SV_POSITION;
                fixed4 colour : COLOR;
            };

            Interpolators vert(Attributes input)
            {
                Interpolators output;
                output.position = UnityObjectToClipPos(input.position);
                // Subtle contrast between the flat top and the curved side.
                output.colour = fixed4(input.colour.rgb * lerp(.88, 1, abs(input.normal.y)), 1);
                return output;
            }

            fixed4 frag(Interpolators input) : SV_Target { return input.colour; }
            ENDCG
        }
    }
}
