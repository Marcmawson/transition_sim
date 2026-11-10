Shader "Instanced/Particle2D" {
	Properties {
		
	}
	SubShader {

		Tags { "RenderType"="Transparent" "Queue"="Transparent" }
		Blend SrcAlpha OneMinusSrcAlpha
		ZWrite Off

		Pass {

			CGPROGRAM

			#pragma vertex vert
			#pragma fragment frag
			#pragma target 4.5

			#include "UnityCG.cginc"
			
			StructuredBuffer<float2> Positions2D;
			StructuredBuffer<float2> Velocities;
			StructuredBuffer<float2> DensityData;
			StructuredBuffer<float> ThermalEnergy;
			StructuredBuffer<uint> ParticlePhase;
			float scale;
			float4 colA;
			Texture2D<float4> ColourMap;
			SamplerState linear_clamp_sampler;
			float velocityMax;
			float ambientTemperatureC;
			float boilingTemperatureC;

			struct v2f
			{
				float4 pos : SV_POSITION;
				float2 uv : TEXCOORD0;
				float3 colour : TEXCOORD1;
				float alpha : TEXCOORD2;
			};

			v2f vert (appdata_full v, uint instanceID : SV_InstanceID)
			{
				float speed = length(Velocities[instanceID]);
				float speedT = saturate(speed / velocityMax);
				float colT = speedT;
				
				float3 centreWorld = float3(Positions2D[instanceID], 0);
				uint phase = ParticlePhase[instanceID];
				// Keep liquid and vapour markers the same size so newly nucleated
				// bubbles remain visible as clusters instead of disappearing.
				float particleScale = scale;
				float3 worldVertPos = centreWorld + mul(unity_ObjectToWorld, v.vertex * particleScale);
				float3 objectVertPos = mul(unity_WorldToObject, float4(worldVertPos.xyz, 1));

				v2f o;
				o.uv = v.texcoord;
				o.pos = UnityObjectToClipPos(objectVertPos);
				float3 speedColour = ColourMap.SampleLevel(linear_clamp_sampler, float2(colT, 0.5), 0).rgb;
				// Keep the full thermal proxy range visible so the conduction gradient
				// does not saturate to red as soon as a particle reaches 8 units.
				float heat = saturate((ThermalEnergy[instanceID] - ambientTemperatureC) / max(1.0, boilingTemperatureC - ambientTemperatureC));
				o.colour = phase == 1 ? float3(0.78, 0.91, 1.0) : lerp(speedColour, float3(1.0, 0.20, 0.025), heat);
				o.alpha = 1.0;

				return o;
			}


			float4 frag (v2f i) : SV_Target
			{
				float2 centreOffset = (i.uv.xy - 0.5) * 2;
				float sqrDst = dot(centreOffset, centreOffset);
				float delta = fwidth(sqrt(sqrDst));
				float alpha = 1 - smoothstep(1 - delta, 1 + delta, sqrDst);

				float3 colour = i.colour;
				return float4(colour, alpha * i.alpha);
			}

			ENDCG
		}
	}
}
