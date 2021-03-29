using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class StereoRig : MonoBehaviour {

  // Left and right stereo cameras.
  public Camera leftCamera;
  public Camera rightCamera;
  private RenderTexture _preallocRT;

  // Start is called before the first frame update
  void Start()
  {
    this._preallocRT = new RenderTexture(
        SimulationParams.AUV_CAMERA_WIDTH,
        SimulationParams.AUV_CAMERA_HEIGHT,
        16, RenderTextureFormat.ARGB32);
  }

  // Update is called once per frame
  void Update()
  {

  }

  // Grabs a rendered texture from a camera.
  private Texture2D GetImageFromCamera(Camera camera)
  {
    RenderTexture currentActiveRT = RenderTexture.active; // Placeholder for active render texture.

    RenderTexture originalTexture = camera.targetTexture;

    // NOTE(milo): Using the ARGB32 format since it's in tutorials, not sure what the best option is
    // here though. It uses 8 bits per RGB channel, which seems standard.
    // NOTE(milo): Documentation on camera rendering is a little confusing.
    // We instantiate a RenderTexture above, which is basically just a buffer that cameras render
    // into. Then, we call Render() and read the rendered image into a Texture2D with ReadPixels().
    camera.targetTexture = this._preallocRT;

    camera.Render();
    RenderTexture.active = camera.targetTexture;

    // Make a new (empty) image and read the camera image into it.
    Texture2D image = new Texture2D(camera.targetTexture.width, camera.targetTexture.height,
                                    TextureFormat.RGB24, false);

    // This will read pixels from the ACTIVE render texture.
    image.ReadPixels(new Rect(0, 0, camera.targetTexture.width, camera.targetTexture.height), 0, 0);
    image.Apply();

    camera.targetTexture = originalTexture;

    RenderTexture.active = currentActiveRT; // Reset the active render texture.

    return image;
  }

  // Returns a stereo pair from the rig.
  public void CaptureStereoPair(out Texture2D iml, out Texture2D imr)
  {
    iml = GetImageFromCamera(this.leftCamera);
    imr = GetImageFromCamera(this.rightCamera);
  }
}
