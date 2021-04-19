Shader "Custom/CameraGrayscale" {
  Properties
  {
    _MainTex ("Texture", 2D) = "white" {}
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

      struct appdata
      {
        float4 vertex : POSITION;
        float2 uv : TEXCOORD0;
        float4 color : COLOR;
      };

      struct v2f
      {
        float2 uv : TEXCOORD0;
        float4 vertex : SV_POSITION;
        float4 scrPos : TEXCOORD1;
        fixed4 color : COLOR;
      };

      v2f vert (appdata v)
      {
        v2f o;
        o.vertex = UnityObjectToClipPos(v.vertex);
        o.scrPos = ComputeScreenPos(o.vertex);
        o.uv = v.uv;
        o.color = v.color;
        return o;
      }

      sampler2D _MainTex;

      fixed4 frag (v2f i) : COLOR
      {
        fixed4 col = tex2Dproj(_MainTex, i.scrPos);
        col.rgb = dot(col, float3(0.3, 0.59, 0.11));
        col.a = 255;
        return col;
      }
      ENDCG
    }
  }
}
