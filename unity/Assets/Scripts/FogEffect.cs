using System.Collections;
using System.Collections.Generic;
using UnityEngine;


[ExecuteInEditMode, ImageEffectAllowedInSceneView]
public class FogEffect : MonoBehaviour {
  public Material _material;
  public Color _fogColor;
  // private float _depthStart = 0;
  public float _depthDistance = 30.0f;

  // TODO(milo): Implement color-dependent attenuation for more realistic underwater images.
  public float _betaRed = 0.4f;
  public float _betaGreen = 0.3f;
  public float _betaBlue = 0.2f;

  void Start()
  {
    GetComponent<Camera>().depthTextureMode = DepthTextureMode.Depth;
  }

  void Update()
  {
    _material.SetColor("_FogColor", _fogColor);
    // _material.SetFloat("_DepthStart", _depthStart);
    _material.SetFloat("_DepthDistance", _depthDistance);
  }

  private void OnRenderImage(RenderTexture source, RenderTexture destination)
  {
    Graphics.Blit(source, destination, this._material);
  }
}
