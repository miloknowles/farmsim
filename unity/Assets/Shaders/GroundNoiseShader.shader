Shader "PeerPlay/GroundNoiseShader" {
  Properties {
    _TessellationLvl ("Tessellation Level", Range(1, 8)) = 4
    _Color ("Color", Color) = (1,1,1,1)
    _MainTex ("Albedo (RGB)", 2D) = "white" {}
    _NormalMap ("Normal Map", 2D) = "bump" {}
    _Glossiness ("Smoothness", Range(0,1)) = 0.5
    _Metallic ("Metallic", Range(0,1)) = 0.0
    _NoiseScale ("Noise Scale", float) = 1
    _NoiseFrequency ("Noise Frequency", float) = 1
    _NoiseOffset ("Noise Offset", Vector) = (0, 0, 0, 0)
  }
  SubShader {
    Tags { "RenderType"="Opaque" }

    CGPROGRAM
    // Physically based Standard lighting model, and enable shadows on all light types
    #pragma surface surf Standard fullforwardshadows tessellate:tess vertex:vert

    // Use shader model 3.0 target, to get nicer looking lighting
    #pragma target 4.6

    #include "noiseSimplex.cginc"

    // Ask for the data that we need in the vertex shader.
    struct appdata {
      float4 vertex : POSITION;
      float3 normal : NORMAL;
      float4 tangent : TANGENT;
      float2 texcoord : TEXCOORD0;
    };

    sampler2D _MainTex, _NormalMap;

    struct Input
    {
      float2 uv_MainTex;
    };

    float _TessellationLvl;
    half _Glossiness;
    half _Metallic;
    fixed4 _Color;

    float _NoiseScale, _NoiseFrequency;
    float4 _NoiseOffset;

    float4 tess()
    {
      return _TessellationLvl;
    }

    void vert(inout appdata v)
    {
      float3 v0 = v.vertex.xyz;                               // Vertex location.
      float3 bitangent = cross(v.normal, v.tangent.xyz);
      float3 v1 = v0 + (v.tangent.xyz) * 0.01;                // Tangent offset.
      float3 v2 = v0 + (bitangent * 0.01);                    // Bitangent offset.

      float ns0 = _NoiseScale * snoise(float3(v0.x + _NoiseOffset.x, v0.y + _NoiseOffset.y, v0.z + _NoiseOffset.z) * _NoiseFrequency);
      v0.xyz += ((ns0 + 1)/2) * v.normal;     // Noise between [0, 1].

      float ns1 = _NoiseScale * snoise(float3(v1.x + _NoiseOffset.x, v1.y + _NoiseOffset.y, v1.z + _NoiseOffset.z) * _NoiseFrequency);
      v1.xyz += ((ns1 + 1)/2) * v.normal;     // Noise between [0, 1].

      float ns2 = _NoiseScale * snoise(float3(v2.x + _NoiseOffset.x, v2.y + _NoiseOffset.y, v2.z + _NoiseOffset.z) * _NoiseFrequency);
      v2.xyz += ((ns2 + 1)/2) * v.normal;     // Noise between [0, 1].

      // Normal is the cross product of tangent and cotangent.
      float3 vn = cross(v2 - v0, v1 - v0);

      v.normal = normalize(-vn);
      v.vertex.xyz = v0;
    }

    void surf(Input IN, inout SurfaceOutputStandard o)
    {
      // Grabs an albedo value from the texture map.
      fixed4 c = tex2D(_MainTex, IN.uv_MainTex) * _Color;
      o.Albedo = c.rgb;
      o.Metallic = _Metallic;
      o.Smoothness = _Glossiness;

      // Adds support for a custom normalmap.
      o.Normal = UnpackNormal(tex2D(_NormalMap, IN.uv_MainTex));
    }
    ENDCG
  }
  FallBack "Diffuse"
}
