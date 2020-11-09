using UnityEngine;
using System.Collections;


public class WaterController : MonoBehaviour {
  public static WaterController current;

  public bool isDynamic = false;

  public float scale = 0.1f;
  public float speed = 1.0f;
  public float waveDistance = 1.0f;
  public float noiseStrength = 1.0f;
  public float noiseWalk = 1.0f;

  void Start()
  {
    current = this;
  }

  /**
   * Returns the height of the ocean at a particular XZ location.
   */
  public float GetWaveHeight(Vector3 position, float timeSinceStart)
  {
    // if (isDynamic) {
    //   return WaveTypes.SinXWave(position, speed, scale, waveDistance, noiseStrength, noiseWalk, timeSinceStart);
    // }

    return 0.0f;
  }
}
