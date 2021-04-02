using System.Collections;
using System.Collections.Generic;
using UnityEngine;


[ExecuteInEditMode, ImageEffectAllowedInSceneView]
public class UnderwaterEffect : MonoBehaviour {
  public Material _material;

  [Range(0.001f, 0.1f)]
  public float _pixelOffset = 0.005f;

  [Range(0.1f, 20f)]
  public float _noiseScale = 1.0f;

  [Range(0.1f, 20f)]
  public float _noiseFrequency = 15.0f;

  [Range(0.1f, 30f)]
  public float _noiseSpeed = 15.0f;

  // private float _depthStart = 0;
  public float _depthDistance = 10;

  private int interval = 10;

  // Update is called once per frame
  void Update()
  {
    if (Time.frameCount % this.interval != 0) {
      return;
    }
    // Push parameter updates to the shader.
    _material.SetFloat("_NoiseFrequency", _noiseFrequency);
    _material.SetFloat("_NoiseSpeed", _noiseSpeed);
    _material.SetFloat("_NoiseScale", _noiseScale);
    _material.SetFloat("_PixelOffset", _pixelOffset);
    // _material.SetFloat("_DepthStart", _depthStart);
    _material.SetFloat("_DepthDistance", _depthDistance);
  }

  private void OnRenderImage(RenderTexture source, RenderTexture destination)
  {
    Graphics.Blit(source, destination, this._material);
  }
}
