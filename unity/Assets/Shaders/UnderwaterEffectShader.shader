Shader "PeerPlay/UnderwaterEffectShader" {
  Properties
  {
    // Noise properties.
    _MainTex ("Texture", 2D) = "white" {}
    _NoiseScale ("Noise Scale", float) = 1
    _NoiseFrequency ("Noise Frequency", float) = 1
    _NoiseSpeed ("Noise Speed", float) = 1
    _PixelOffset ("Pixel Offset", float) = 0.005
    _DepthStart ("Depth Start", float) = 0
    _DepthDistance ("Depth Distance", float) = 10
  }
  SubShader {
    // No culling or depth
    Cull Off ZWrite Off ZTest Always

    Pass
    {
      CGPROGRAM
      #pragma vertex vert
      #pragma fragment frag

      #include "UnityCG.cginc"
      #include "noiseSimplex.cginc"
      #define M_PI 3.1415926

      // Declare properties from above.
      uniform float _NoiseFrequency, _NoiseScale, _NoiseSpeed, _PixelOffset;
      float _DepthStart, _DepthDistance;
      sampler2D _CameraDepthTexture;

      struct appdata
      {
        float4 vertex : POSITION;
        float2 uv : TEXCOORD0;
      };

      struct v2f
      {
        float2 uv : TEXCOORD0;
        float4 vertex : SV_POSITION;
        float4 scrPos : TEXCOORD1;
      };

      v2f vert (appdata v)
      {
        v2f o;
        o.vertex = UnityObjectToClipPos(v.vertex);
        o.scrPos = ComputeScreenPos(o.vertex);
        o.uv = v.uv;
        return o;
      }

      sampler2D _MainTex;

      fixed4 frag (v2f i) : COLOR
      {
        float depthValue = Linear01Depth(tex2Dproj(_CameraDepthTexture, UNITY_PROJ_COORD(i.scrPos)).r) * _ProjectionParams.z;
        depthValue = saturate((depthValue - _DepthStart) / _DepthDistance); // Clamps between 0 and 1.
        float inverseDepthValue = 1 - depthValue;

        float3 screenpos = float3(i.scrPos.x, i.scrPos.y, 0) * _NoiseFrequency;
        screenpos.z += _Time.x * _NoiseSpeed;
        float noise = _NoiseScale * ((snoise(screenpos) + 1) / 2);
        float4 direction = float4(cos(noise*M_PI*2), sin(noise*M_PI*2), 0, 0);

        // _MainTex is the rendered camera image.
        fixed4 col = tex2Dproj(_MainTex, i.scrPos + _PixelOffset * inverseDepthValue * normalize(direction));
        return col;
      }
      ENDCG
    }
  }
}
