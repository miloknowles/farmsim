using System.Collections;
using System.Collections.Generic;
using UnityEngine;


// Global parameters and constants to control the simulation.
// TODO(milo): Make these menu-editable eventually.
public class SimulationParams : MonoBehaviour {
  public static int RESERVED_APRILTAGS = 1;
  public static float WATER_DENSITY_KG_M3 = 1027.3f;

  // Sensor publishing rates.
  public static float CAMERA_PUBLISH_HZ = 10.0f;

  // Camera configuration.
  public static int AUV_CAMERA_WIDTH = 672;
  public static int AUV_CAMERA_HEIGHT = 376;

  // LCM channels.
  public static string CHANNEL_AUV_IMU = "sim/auv/imu";
  public static string CHANNEL_AUV_MAG = "sim/auv/mag";
  public static string CHANNEL_AUV_DEPTH = "sim/auv/depth";
  public static string CHANNEL_AUV_STEREO = "sim/auv/stereo";
  public static string CHANNEL_AUV_STEREO_SHMEM = "sim/auv/stereo_shm";
  public static string CHANNEL_AUV_RANGE_ALL = "sim/auv/range";
  public static string CHANNEL_AUV_RANGE0 = "sim/auv/range0";
  public static string CHANNEL_AUV_RANGE1 = "sim/auv/range1";
  public static string CHANNEL_AUV_WORLD_P_IMU = "sim/auv/pose/world_P_body";
  public static string CHANNEL_AUV_WORLD_P_IMU_INITIAL = "sim/auv/pose/world_P_body_initial";
  public static string SYSTEM_SHM_FOLDER = "/tmp/shm";
}
