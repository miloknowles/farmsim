using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class StereoRig : MonoBehaviour {

  // Left and right stereo cameras.
  public Camera leftCamera;
  public Camera rightCamera;
  private RenderTexture preallocRenderTexture;

  // Start is called before the first frame update
  void Start()
  {
    // TODO(milo): RGB24?
    this.preallocRenderTexture = new RenderTexture(
        SimulationParams.AUV_CAMERA_WIDTH,
        SimulationParams.AUV_CAMERA_HEIGHT,
        16, RenderTextureFormat.ARGB32);
  }

  // Grabs a rendered texture from a camera. The passed output image must be initialized to the
  // correct height, width, and pixel format.
  private void GetImageFromCamera(ref Camera camera, ref Texture2D image)
  {
    RenderTexture currentActiveRT = RenderTexture.active; // Placeholder for active render texture.
    RenderTexture originalTexture = camera.targetTexture;

    // NOTE(milo): Using the ARGB32 format since it's in tutorials, not sure what the best option is
    // here though. It uses 8 bits per RGB channel, which seems standard.
    // NOTE(milo): Documentation on camera rendering is a little confusing.
    // We instantiate a RenderTexture above, which is basically just a buffer that cameras render
    // into. Then, we call Render() and read the rendered image into a Texture2D with ReadPixels().
    camera.targetTexture = this.preallocRenderTexture;

    camera.Render();
    RenderTexture.active = camera.targetTexture;

    // Make a new (empty) image and read the camera image into it.
    // Texture2D image = new Texture2D(camera.targetTexture.width, camera.targetTexture.height,
    //                                 TextureFormat.RGB24, false);

    // This will read pixels from the ACTIVE render texture.
    image.ReadPixels(new Rect(0, 0, camera.targetTexture.width, camera.targetTexture.height), 0, 0);
    image.Apply();

    camera.targetTexture = originalTexture;

    RenderTexture.active = currentActiveRT; // Reset the active render texture.
  }

  // Returns a stereo pair from the rig.
  public void CaptureStereoPair(ref Texture2D img_left, ref Texture2D img_right)
  {
    GetImageFromCamera(ref this.leftCamera, ref img_left);
    GetImageFromCamera(ref this.rightCamera, ref img_right);
  }
}
