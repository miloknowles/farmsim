using System.Collections;
using System.Collections.Generic;
using UnityEngine;


[ExecuteInEditMode]
public class DepthCamera : MonoBehaviour
{
  public Material material;
  private RenderTexture preallocRenderTexture;
  private Camera _camera;

  void Start()
  {
    this._camera = GetComponent<Camera>();
    this._camera.depthTextureMode = DepthTextureMode.Depth;

    RenderTexture.allowThreadedTextureCreation = true;
    // TODO(milo): RGB24?
    this.preallocRenderTexture = new RenderTexture(
        SimulationParams.AUV_CAMERA_WIDTH,
        SimulationParams.AUV_CAMERA_HEIGHT,
        16, RenderTextureFormat.ARGB32);

    // https://docs.unity3d.com/ScriptReference/RenderTexture.html
    this.preallocRenderTexture.DiscardContents();
  }

  void OnRenderImage(RenderTexture source, RenderTexture destination)
  {
    Graphics.Blit(source, destination, this.material);
  }

  // Grabs a rendered texture from a _camera. The passed output image must be initialized to the
  // correct height, width, and pixel format.
  public void Capture(ref Texture2D image)
  {
    RenderTexture currentActiveRT = RenderTexture.active; // Placeholder for active render texture.
    RenderTexture originalTexture = this._camera.targetTexture;

    this._camera.targetTexture = this.preallocRenderTexture;

    this._camera.Render();
    RenderTexture.active = this._camera.targetTexture;

    // This will read pixels from the ACTIVE render texture.
    image.ReadPixels(new Rect(0, 0, this._camera.targetTexture.width, this._camera.targetTexture.height), 0, 0, false);
    image.Apply();

    this._camera.targetTexture = originalTexture;

    RenderTexture.active = currentActiveRT; // Reset the active render texture.
  }
}
