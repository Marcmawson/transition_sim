Shader "PhysicsSim/BeakerWaterSurface"
{
	Properties
	{
		_MainTex ("Texture", 2D) = "white" {}
	}
	SubShader
	{
		Tags { "Queue"="Transparent-50" "RenderType"="Transparent" }
		GrabPass { "_BeakerWaterBackground" }

		Pass
		{
			Tags { "LightMode"="ForwardBase" }
			ZWrite On
			CGPROGRAM
			#pragma target 4.5
			#pragma vertex vert
			#pragma fragment frag
			#include "UnityCG.cginc"
			#include "UnityLightingCommon.cginc"

			struct v2f
			{
				float4 vertex : SV_POSITION;
				float3 normal : TEXCOORD0;
                float3 world : TEXCOORD1;
			};


			struct appdata
			{
				uint vertexID : SV_VertexID;
			};

			struct Vertex {
				float3 position;
				float3 normal;
			};

			StructuredBuffer<Vertex> VertexBuffer;
			float4 col;
            sampler2D _BeakerWaterBackground;
            float3 _SurfaceCentre;

			v2f vert (appdata v)
			{
				v2f o;
				float3 vertPos = VertexBuffer[v.vertexID].position;
				float3 normal = VertexBuffer[v.vertexID].normal;
				o.world = vertPos + _SurfaceCentre;
                o.vertex = UnityWorldToClipPos(o.world);
				o.normal = normal;
				return o;
			}

			float4 frag (v2f i) : SV_Target
			{
				float3 n = normalize(i.normal);
                float3 view = normalize(_WorldSpaceCameraPos - i.world);
                const float waterIndex = 1.333;
                const float normalReflectance = .02037;
                float fresnel = normalReflectance + (1 - normalReflectance)
                    * pow(1 - saturate(dot(n, view)), 5);
                float3 reflected = reflect(-view, n);
                float3 sky = lerp(float3(.72, .81, .91), float3(.18, .39, .71),
                    saturate(reflected.y));
                float3 environment = lerp(float3(.25, .21, .16), sky,
                    smoothstep(-.09, .04, reflected.y));

                // Widen the sun glint by the pixel footprint to limit sparkle on moving triangles.
                float3 toSun = normalize(_WorldSpaceLightPos0.xyz);
                float sunAlignment = saturate(dot(reflected, toSun));
                float feather = max(fwidth(sunAlignment), .0001);
                float sunGlint = smoothstep(.9986 - feather, .9998 + feather, sunAlignment)
                    * smoothstep(0, .05, dot(n, toSun));
                environment += _LightColor0.rgb * sunGlint * 12;
                // Use water's refractive index and approximate the refracted ray in screen space.
                float3 transmitted = refract(-view, n, 1.0 / waterIndex);
                float4 baseGrab = ComputeGrabScreenPos(UnityWorldToClipPos(i.world));
                float4 bentGrab = ComputeGrabScreenPos(UnityWorldToClipPos(
                    i.world + (transmitted + view) * .018));
                float2 uv = bentGrab.xy / max(bentGrab.w, 1e-5);
                float2 edge = min(uv, 1 - uv);
                float edgeFade = smoothstep(0, .025, min(edge.x, edge.y));
                float3 background = tex2D(_BeakerWaterBackground,
                    lerp(baseGrab.xy / max(baseGrab.w, 1e-5), uv, edgeFade)).rgb;
                float3 transmission = lerp(float3(.99, .997, 1), float3(.045, .19, .58), saturate(col.a));
                return float4(background * transmission * (1 - fresnel)
                    + environment * fresnel, 1);
			}

			ENDCG
		}
	}
}
