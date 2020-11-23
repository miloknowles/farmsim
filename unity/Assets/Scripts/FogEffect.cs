using System.Collections;
using System.Collections.Generic;
using UnityEngine;


[ExecuteInEditMode, ImageEffectAllowedInSceneView]
public class FogEffect : MonoBehaviour {
    public Material _material;
    public Color _fogColor;
    private float _depthStart = 0;
    public float _depthDistance = 15;

    // Start is called before the first frame update
    void Start()
    {
        GetComponent<Camera>().depthTextureMode = DepthTextureMode.Depth;
    }

    // Update is called once per frame
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
