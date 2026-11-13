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
			ZWrite On
			CGPROGRAM
			#pragma target 4.5
			#pragma vertex vert
			#pragma fragment frag
			#include "UnityCG.cginc"

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
                float fresnel = .02 + .98 * pow(1 - saturate(dot(n, view)), 5);
                float3 light = normalize(float3(-.65, .75, -.75));
                float spec = pow(saturate(dot(n, normalize(light + view))), 96);
                // Use water's refractive index and approximate the refracted ray in screen space.
                float3 transmitted = refract(-view, n, 1.0 / 1.333);
                float4 grab = ComputeGrabScreenPos(UnityWorldToClipPos(
                    i.world + (transmitted + view) * .012));
                float3 background = tex2D(_BeakerWaterBackground, saturate(grab.xy / max(grab.w, 1e-5))).rgb;
                float3 transmission = lerp(float3(.99, .997, 1), float3(.045, .19, .58), saturate(col.a));
                return float4(background * transmission * (1 - fresnel)
                    + float3(.48, .55, .63) * fresnel + spec * .32, 1);
			}

			ENDCG
		}
	}
}
