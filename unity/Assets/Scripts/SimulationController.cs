using System.Collections;
using System.Collections.Generic;
using UnityEngine;


// Global parameters to control the simulation.
public struct SimulationController
{
  public static int RESERVED_APRILTAGS = 1;
  public static int ROS_BRIDGE_PORT = 9090;
  public static float WATER_DENSITY_KG_M3 = 1027.3f;
  public static float CAMERA_PUBLISH_HZ = 10.0f;
  public static float SENSOR_PUBLISH_HZ = 20.0f;
  public static int AUV_CAMERA_WIDTH = 752;
  public static int AUV_CAMERA_HEIGHT = 480;
}
