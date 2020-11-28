﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;


/**
 * Global parameters that are used throughout the simulator.
 */
public class Config
{
  public static int RESERVED_APRILTAGS = 1;
  public static int ROS_BRIDGE_PORT = 9090;
  public static float WATER_DENSITY = 1027.3f;          // kg/m3
  public static float CAMERA_PUBLISH_HZ = 10.0f;
  public static float SENSOR_PUBLISH_HZ = 20.0f;
}
