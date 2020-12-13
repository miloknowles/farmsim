using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class SimulationController : MonoBehaviour {
  public static int RESERVED_APRILTAGS = 1;
  public static int ROS_BRIDGE_PORT = 9090;
  public static float WATER_DENSITY_KG_M3 = 1027.3f;
  public static float CAMERA_PUBLISH_HZ = 10.0f;
  public static float SENSOR_PUBLISH_HZ = 20.0f;
  public static int AUV_CAMERA_WIDTH = 752;
  public static int AUV_CAMERA_HEIGHT = 480;

  // Environmental parameters.
  public float WATER_VISIBILITY_METERS = 20.0f;
  public int MARINE_SNOW_PARTICLES = 500;

  private FlowFieldSimulator _marineSnow;
  private FogEffect[] _fogEffectScripts;

  void Start()
  {
    this._marineSnow = GameObject.Find("AUVMarineSnow").GetComponent<FlowFieldSimulator>();
    this._fogEffectScripts = Object.FindObjectsOfType<FogEffect>();

    StartCoroutine(UpdateParameters());
  }

  // Update simulation parameters at a slow rate.
  IEnumerator UpdateParameters()
  {
    while (true) {
      yield return new WaitForSeconds(1.0f);

      this._marineSnow._numberOfParticles = this.MARINE_SNOW_PARTICLES;
      foreach (FogEffect f in this._fogEffectScripts) {
        f._depthDistance = this.WATER_VISIBILITY_METERS;
      }
    }
  }
}
