Shader "PhysicsSim/BeakerWaterSurface"
{
    Properties
    {
        _Color ("Water colour", Color) = (0.05, 0.55, 0.82, 1)
        _Beaker ("Inner radius, outer radius, floor, rim", Vector) = (0.033, 0.064, 0.151, 0.211)
        _BaseY ("Exterior bottom", Float) = 0.12
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        Cull Off
        CGPROGRAM
        #pragma surface surf Standard
        #pragma target 3.0

        fixed4 _Color;
        float4 _Beaker;
        float _BaseY;
        struct Input { float3 worldPos; };

        void surf(Input IN, inout SurfaceOutputStandard output)
        {
            float3 p = IN.worldPos;
            // The revolved solver glass uses 24 straight segments. Measure
            // against the nearest side plane rather than an ideal cylinder.
            const float sector = 6.28318530718 / 24;
            float angle = dot(p.xz, p.xz) > 0 ? atan2(p.z, p.x) : 0;
            float normalAngle = (floor(angle / sector) + 0.5) * sector;
            float side = dot(p.xz, float2(cos(normalAngle), sin(normalAngle)));
            const float apothemScale = 0.99144486137;
            float outer = _Beaker.y * apothemScale;
            float inner = _Beaker.x * apothemScale;
            if (p.y >= _BaseY && p.y < _Beaker.w && side < outer)
                clip(p.y < _Beaker.z || side > inner ? -1 : 1);

            output.Albedo = _Color.rgb;
            output.Emission = _Color.rgb * 0.24;
            output.Smoothness = 0.86;
            output.Alpha = 1;
        }
        ENDCG
    }
    FallBack Off
}
