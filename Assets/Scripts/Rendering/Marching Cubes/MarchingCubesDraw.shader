Shader "Fluid/MarchingCubesDraw"
{
	Properties
	{
		_MainTex ("Texture", 2D) = "white" {}
	}
	SubShader
	{
		Tags { "RenderType"="Opaque" }
		LOD 100

		Pass
		{
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
                return float4(lerp(col.rgb * (.55 + .45 * saturate(dot(n, light))),
                    float3(.58, .78, .89), fresnel) + spec * .5, 1);
			}

			ENDCG
		}
	}
}
