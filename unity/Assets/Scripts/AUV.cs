using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using System.IO;

using UnityEngine;

using ROSBridgeLib.std_msgs;
using ROSBridgeLib.sensor_msgs;
using ROSBridgeLib.geometry_msgs;
using ROSBridgeLib.CustomMessages;
using ROSBridgeLib;

using Simulator;


namespace Simulator {

public enum IOMode
{
  NO_OUTPUT = 0,
  PUBLISH_TO_ROS = 1,
  SAVE_TO_DISK = 2
}

}

public class AUV : MonoBehaviour {
  private int msgPublishCount;
  DateTime lastFrame;
  DateTime camStart;

  // Cameras attached to the vehicle.
  public Camera camera_forward_left;
  public Camera camera_forward_right;
  public Camera camera_downward_left;
  public Camera camera_upward_left;
  private RenderTexture rt;

  public Rigidbody imuBody;
  public FarmController farm;

  // Used to calculate accleration with finite-differencing.
  private Vector3 lastImuBodyVelocity;
  private Vector3 imuBodyAccel;
  private float depthSensorSigma = 0.1f;       // Standard deviation of depth noise.

  private List<IEnumerator> routines;
  private Quaternion rotationFaceWinch = Quaternion.identity;
  public bool doRoutine = false;

  private ROSMessageHolder roslink;
  public Simulator.IOMode simulatorIOMode = Simulator.IOMode.NO_OUTPUT;
  public string outputDatasetName = "default_dataset";

  // NOTE(milo): This needs to be an absolute path!
  public string outputDatasetRoot = "/home/milo/datasets/Unity3D/farmsim/";
  private string leftImageSubfolder = "image_0";
  private string rightImageSubfolder = "image_1";

  private RenderTexture _preallocRT;

  void Start()
  {
    this._preallocRT = new RenderTexture(
        SimulationController.AUV_CAMERA_WIDTH,
        SimulationController.AUV_CAMERA_HEIGHT,
        16, RenderTextureFormat.ARGB32);

    this.roslink = GameObject.Find("ROSMessageHolder").GetComponent<ROSMessageHolder>();

    // Add a publisher for each of the cameras that we want to support.
    this.roslink.ros.AddPublisher(typeof(CameraForwardLeftPublisher));
    this.roslink.ros.AddPublisher(typeof(CameraForwardRightPublisher));
    this.roslink.ros.AddPublisher(typeof(CameraDownwardLeftPublisher));
    this.roslink.ros.AddPublisher(typeof(CameraUpwardLeftPublisher));

    this.roslink.ros.AddPublisher(typeof(DepthPublisher));
    this.roslink.ros.AddPublisher(typeof(GroundtruthDepthPublisher));
    this.roslink.ros.AddPublisher(typeof(PoseStampedPublisher));

    this.roslink.ros.AddPublisher(typeof(ImuPublisher));
    this.roslink.ros.AddPublisher(typeof(HeadingPublisher));
    this.roslink.ros.AddPublisher(typeof(GroundtruthHeadingPublisher));

    msgPublishCount = 0;
    lastFrame = DateTime.Now;
    camStart = DateTime.Now;

    //============================== DEMO SETUP ================================
    if (this.doRoutine) {
      this.routines = new List<IEnumerator>{
        this.AnimateWaypoint(this.gameObject, this.farm.GetWinchLocation(0, 'A').position + new Vector3(0, 0, -2.0f), rotationFaceWinch, 6, 2),
        this.AnimateFollowWinch(this.gameObject, 0, 'A'),
        this.AnimateWaypoint(this.gameObject, this.farm.GetWinchLocation(0, 'B').position + new Vector3(0, 0, -2.0f), rotationFaceWinch, 6, 2),
        this.AnimateFollowWinch(this.gameObject, 0, 'B'),
        this.AnimateWaypoint(this.gameObject, this.farm.GetWinchLocation(0, 'C').position + new Vector3(0, 0, -2.0f), rotationFaceWinch, 6, 2),
        this.AnimateFollowWinch(this.gameObject, 0, 'C')
      };
      StartCoroutine(ChainCoroutines(this.routines));
    }

    if (this.simulatorIOMode == Simulator.IOMode.PUBLISH_TO_ROS) {
      StartCoroutine(PublishCameraSyncedMessages());
      StartCoroutine(PublishImu());
    } else if (this.simulatorIOMode == Simulator.IOMode.SAVE_TO_DISK) {
      StartCoroutine(SaveStereoImageDataset());
    }
  }

  void FixedUpdate()
  {
    imuBodyAccel = (imuBody.velocity - lastImuBodyVelocity) / Time.fixedDeltaTime;
    lastImuBodyVelocity = imuBody.velocity;
  }

  /**
   * Grabs an image from a camera and returns it.
   */
  Texture2D GetImageFromCamera(Camera camera)
  {
    RenderTexture currentActiveRT = RenderTexture.active; // Placeholder for active render texture.

    RenderTexture originalTexture = camera.targetTexture;

    // NOTE(milo): Using the ARGB32 format since it's in tutorials, not sure what the best option is
    // here though. It uses 8 bits per RGB channel, which seems standard.
    // NOTE(milo): Documentation on camera rendering is a little confusing.
    // We instantiate a RenderTexture above, which is basically just a buffer that cameras render
    // into. Then, we call Render() and read the rendered image into a Texture2D with ReadPixels().
    // camera.targetTexture = new RenderTexture(
    //     SimulationController.AUV_CAMERA_WIDTH,
    //     SimulationController.AUV_CAMERA_HEIGHT,
    //     16, RenderTextureFormat.ARGB32);
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

  /**
   * Publishes a CompressedImageMsg on 'topic' using the given image 'im'.
   */
  void PublishCameraImage(Texture2D im, TimeMsg timeMessage, HeaderMsg headerMessage, string topic)
  {
    byte[] data = im.EncodeToJPG();
    string format = "jpeg";
    var compressedImageMsg = new CompressedImageMsg(headerMessage, format, data);
    this.roslink.ros.Publish(topic, compressedImageMsg);
    Destroy(im);
    this.roslink.ros.Render();
  }

  /**
   * Any message that is meant to be synchronized with camera images should be published here.
   */
  IEnumerator PublishCameraSyncedMessages()
  {
    while (true) {
      // yield return new WaitForEndOfFrame();
      yield return new WaitForSeconds(1.0f / SimulationController.CAMERA_PUBLISH_HZ);
      yield return new WaitForEndOfFrame();

      var now = DateTime.Now;
      var timeSinceStart = now - camStart;
      var timeMessage = new TimeMsg(timeSinceStart.Seconds, timeSinceStart.Milliseconds);

      PublishCameraImage(
          GetImageFromCamera(camera_forward_left),
          timeMessage,
          new HeaderMsg(msgPublishCount, timeMessage, "auv_camera_fl"),
          CameraForwardLeftPublisher.GetMessageTopic());

      PublishCameraImage(
          GetImageFromCamera(camera_forward_right),
          timeMessage,
          new HeaderMsg(msgPublishCount, timeMessage, "auv_camera_fr"),
          CameraForwardRightPublisher.GetMessageTopic());

      // PublishCameraImage(7.90639384423901
      //     GetImageFromCamera(camera_downward_left),
      //     timeMessage,
      //     new HeaderMsg(msgPublishCount, timeMessage, "auv_camera_dl"),
      //     CameraDownwardLeft.GetMessageTopic());

      // PublishCameraImage(
      //     GetImageFromCamera(camera_upward_left),
      //     timeMessage,
      //     new HeaderMsg(msgPublishCount, timeMessage, "auv_camera_ul"),
      //     CameraUpwardLeftPublisher.GetMessageTopic());

      // Publish the pose of the vehicle in the world.
      Vector3 t_auv_world = imuBody.transform.position;
      Quaternion q_auv_world = imuBody.transform.rotation;

      QuaternionMsg q_auv_world_msg = new QuaternionMsg(
          q_auv_world.x, q_auv_world.y, q_auv_world.z, q_auv_world.w);
      PointMsg t_auv_world_msg = new PointMsg(t_auv_world.x, t_auv_world.y, t_auv_world.z);
      PoseStampedMsg msg = new PoseStampedMsg(new HeaderMsg(msgPublishCount, timeMessage, "auv_imu"),
                                              new PoseMsg(t_auv_world_msg, q_auv_world_msg));
      this.roslink.ros.Publish(PoseStampedPublisher.GetMessageTopic(), msg);
      this.roslink.ros.Render();
    }
  }

  IEnumerator PublishImu() {
    while (true) {
      // yield return new WaitForEndOfFrame();
      yield return new WaitForSeconds(1.0f / SimulationController.SENSOR_PUBLISH_HZ);

      var now = DateTime.Now;
      var timeSinceStart = now - camStart;
      var timeMessage = new TimeMsg(timeSinceStart.Seconds, timeSinceStart.Milliseconds);
      var headerMessage = new HeaderMsg(msgPublishCount, timeMessage, "imu");

      Quaternion vehicleImu;
      vehicleImu = imuBody.transform.rotation;
      double xey = vehicleImu.x;
      double yey = vehicleImu.y;
      double zey = vehicleImu.z;
      double wey = vehicleImu.w;
      var imuData = new QuaternionMsg(xey, yey, zey, wey);

      // NOTE(milo): Sending empty covariance matrix for now (denoted with -1 in the 0th position).
      double[] zeroMatrix3x3 = new double[]{-1, 0, 0, 0, 0, 0, 0, 0, 0};

      var zeroVect3 = new Vector3Msg(0.0, 0.0, 0.0);
      var linearAccelMsg = new Vector3Msg(imuBodyAccel.x, imuBodyAccel.y, imuBodyAccel.z);
      var msg = new ImuMessage(headerMessage, imuData, zeroMatrix3x3, zeroVect3, zeroMatrix3x3,
                               linearAccelMsg, zeroMatrix3x3);

      this.roslink.ros.Publish(ImuPublisher.GetMessageTopic(), msg);

      // Publish the heading (yaw) of the vehicle.
      float theta = imuBody.transform.rotation.eulerAngles.z;
      this.roslink.ros.Publish(HeadingPublisher.GetMessageTopic(), new Float64Msg(theta));
      this.roslink.ros.Publish(GroundtruthHeadingPublisher.GetMessageTopic(), new Float64Msg(theta));
      this.roslink.ros.Render();

      // Publish the current barometer depth (groundtruth and depth with simulated noise).
      float depth = imuBody.transform.position.y;
      Float64Msg gtDepthMessage = new Float64Msg(depth);
      Float64Msg depthMessage = new Float64Msg(depth + Utils.Gaussian(0, this.depthSensorSigma));
      this.roslink.ros.Publish(DepthPublisher.GetMessageTopic(), depthMessage);
      this.roslink.ros.Publish(GroundtruthDepthPublisher.GetMessageTopic(), gtDepthMessage);
    }
  }

  /**
   * Animates the vehicle moving between two waypoints.
   */
  IEnumerator AnimateMotion(GameObject vehicle,
                            Vector3 t_start,
                            Quaternion q_start,
                            Vector3 t_end,
                            Quaternion q_end,
                            float transit_sec)
  {
    float startTime = Time.time;
    float elap = (Time.time - startTime);
    while (elap < transit_sec) {
      elap = (Time.time - startTime);
      float T = Mathf.Clamp(elap / transit_sec, 0, 1); // Compute interpolation amount.

      // Linear interpolation between the two endpoints, slerp between quaternions.
      vehicle.transform.position = (1 - T)*t_start + T*t_end;
      vehicle.transform.rotation = Quaternion.Slerp(q_start, q_end, T);

      // Yield progress.
      yield return T;
    }
  }

  /**
   * Animates the vehicle moving from its current pose to a waypoint pose.
   */
  IEnumerator AnimateWaypoint(GameObject vehicle, Vector3 t_end, Quaternion q_end, float transit_sec, float rotate_sec)
  {
    // NOTE(milo): Need to grab the start transform HERE so that it's up-to-date when this coroutine
    // starts. If it was an argument, it would be pinned to whatever location the vehicle was at
    // upon instantiating the coroutine.
    Vector3 t_start = vehicle.transform.position;
    Quaternion q_start = vehicle.transform.rotation;

    // Align the vehicle with the direction of motion.
    Quaternion q_transit = Simulator.Utils.RotateAlignVectors(new Vector3(0, 0, 1), t_end - t_start);

    // Rotate in place.
    float startTime = Time.time;
    while ((Time.time - startTime) < rotate_sec) {
      float elap = (Time.time - startTime);
      float T = Mathf.Clamp(elap / rotate_sec, 0, 1); // Compute interpolation amount.
      vehicle.transform.position = t_start;
      vehicle.transform.rotation = Quaternion.Slerp(q_start, q_transit, T);

      yield return T;
    }

    startTime = Time.time;
    while ((Time.time - startTime) < transit_sec) {
      float elap = (Time.time - startTime);

      // First half of the trajectory (accelerating).
      float T = 0;
      if (elap < (transit_sec / 2.0f)) {
        T = (2.0f / Mathf.Pow(transit_sec, 2.0f)) * Mathf.Pow(elap, 2.0f);

      // Second half of the trajectory (decelerating).
      } else {
        T = 1 - (2.0f / Mathf.Pow(transit_sec, 2.0f)) * Mathf.Pow((transit_sec - elap), 2.0f);
      }

      T = Mathf.Clamp(T, 0, 1); // Compute interpolation amount.

      // Linear interpolation between the two endpoints, slerp between quaternions.
      vehicle.transform.position = (1 - T)*t_start + T*t_end;
      vehicle.transform.rotation = q_transit;

      // Yield progress.
      yield return T;
    }

    // Rotate to goal orientation.
    startTime = Time.time;
    while ((Time.time - startTime) < rotate_sec) {
      float elap = (Time.time - startTime);
      float T = Mathf.Clamp(elap / rotate_sec, 0, 1); // Compute interpolation amount.
      vehicle.transform.position = t_end;
      vehicle.transform.rotation = Quaternion.Slerp(q_transit, q_end, T);
      yield return T;
    }
  }

  IEnumerator AnimateFollowWinch(GameObject vehicle, int row, char buoy)
  {
    GameObject winch_to_follow = this.farm.GetWinchesAtAddress(row, buoy)[0];
    this.farm.ToggleDepth(row, buoy);
    float prev_depth = 123;
    while (this.farm.winchInProgress) {
      prev_depth = winch_to_follow.transform.position.y;
      vehicle.transform.position = new Vector3(vehicle.transform.position.x, prev_depth, vehicle.transform.position.z);
      yield return null;
    }
  }

  /**
   * Executes a sequence of coroutines. As soon as one finishes, the next one is started.
   */
  IEnumerator ChainCoroutines(List<IEnumerator> routines)
  {
    foreach (IEnumerator r in routines) {
      yield return StartCoroutine(r);
    }
  }

  IEnumerator SaveStereoImageDataset()
  {
    string datasetFolder = Path.Combine(this.outputDatasetRoot, this.outputDatasetName);

    if (Directory.Exists(datasetFolder)) {
      Debug.Log("WARNING: Deleting existing dataset folder: " + datasetFolder);
      Directory.Delete(datasetFolder, true);
    }

    string leftImageFolder = Path.Combine(datasetFolder, this.leftImageSubfolder);
    string rightImageFolder = Path.Combine(datasetFolder, this.rightImageSubfolder);
    Debug.Log(leftImageFolder);
    Debug.Log(rightImageFolder);

    Directory.CreateDirectory(leftImageFolder);
    Directory.CreateDirectory(rightImageFolder);

    int frame_id = 0;

    // NOTE(milo): Maximum number of frames to save. Avoids using up all disk space by accident.
    while (frame_id < 5000) {
      // yield return new WaitForSeconds(1.0f / SimulationController.CAMERA_PUBLISH_HZ);
      yield return new WaitForEndOfFrame();

      // Writes the time in seconds with 5 decimal places.
      string sec = Time.fixedTime.ToString("F6");
      File.AppendAllLines(Path.Combine(datasetFolder, "times.txt"), new string[] { sec });

      Transform T_world_cam = this.camera_forward_left.transform;
      Quaternion q_world_cam = T_world_cam.rotation;
      Vector3 t_world_cam = T_world_cam.position;
      List<string> pose_line = new List<string>{
        sec,
        q_world_cam.w.ToString(),
        q_world_cam.x.ToString(),
        q_world_cam.y.ToString(),
        q_world_cam.z.ToString(),
        t_world_cam.x.ToString(),
        t_world_cam.y.ToString(),
        t_world_cam.z.ToString(),
        "\n"
      };

      File.AppendAllText(Path.Combine(datasetFolder, "poses_0.txt"), string.Join(" ", pose_line));

      Texture2D leftImage = GetImageFromCamera(camera_forward_left);
      Texture2D rightImage = GetImageFromCamera(camera_forward_right);

      byte[] leftPng = leftImage.EncodeToPNG();
      byte[] rightPng = rightImage.EncodeToPNG();

      string imagePath = $"{frame_id:000000}.png";

      File.WriteAllBytes(Path.Combine(leftImageFolder, imagePath), leftPng);
      File.WriteAllBytes(Path.Combine(rightImageFolder, imagePath), rightPng);

      ++frame_id;
    }
  }
}
