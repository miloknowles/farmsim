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
  private string leftImageSubfolder = "cam0";
  private string rightImageSubfolder = "cam1";
  private string imuSubfolder = "imu0";

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
      StartCoroutine(SaveImuDataset());
    }
  }

  void FixedUpdate()
  {
    // Get the velocity in the local IMU frame.
    Quaternion q_imu_world = Quaternion.Inverse(this.imuBody.transform.rotation);
    Vector3 v_imu = q_imu_world * this.imuBody.velocity;

    v_imu.y *= -1; // Convert to right-handed frame.

    imuBodyAccel = (v_imu - lastImuBodyVelocity) / Time.fixedDeltaTime;
    lastImuBodyVelocity = v_imu;
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

      // PublishCameraImage(
      //     GetImageFromCamera(camera_downward_left),
      //     timeMessage,
      //     new HeaderMsg(msgPublishCount, timeMessage, "auv_camera_dl"),
      //     CameraDownwardLeft.GetMessageTopic());

      // PublishCameraImage(
      //     GetImageFromCamera(camera_upward_left),
      //     timeMessage,
      //     new HeaderMsg(msgPublishCount, timeMessage, "auv_camera_ul"),
      //     CameraUpwardLeftPublisher.GetMessageTopic());

      // Publish the pose of the vehicle in the world (right-handed!).
      Transform T_world_imu = imuBody.transform;
      Vector3 t_world_imu;
      Quaternion q_world_imu;
      Utils.ToRightHandedTransform(T_world_imu, out t_world_imu, out q_world_imu);

      QuaternionMsg q_world_imu_msg = new QuaternionMsg(
          q_world_imu.x, q_world_imu.y, q_world_imu.z, q_world_imu.w);
      PointMsg t_world_imu_msg = new PointMsg(t_world_imu.x, t_world_imu.y, t_world_imu.z);
      PoseStampedMsg msg = new PoseStampedMsg(new HeaderMsg(msgPublishCount, timeMessage, "imu"),
                                              new PoseMsg(t_world_imu_msg, q_world_imu_msg));
      this.roslink.ros.Publish(PoseStampedPublisher.GetMessageTopic(), msg);
      this.roslink.ros.Render();
    }
  }

  IEnumerator SaveImuDataset() {
    string datasetFolder = Path.Combine(this.outputDatasetRoot, this.outputDatasetName);

    // Wait until the outer dataset folder has been created (done by the image saving routine).
    // while (!Directory.Exists(datasetFolder)) {
    //   yield return new WaitForEndOfFrame();
    // }

    string imuFolder = Path.Combine(datasetFolder, this.imuSubfolder);

    if (Directory.Exists(imuFolder)) {
      Debug.Log("WARNING: Deleting existing IMU folder: " + imuFolder);
      Directory.Delete(imuFolder, true);
    }

    Directory.CreateDirectory(imuFolder);

    string csv = Path.Combine(imuFolder, "data.csv");

    // NOTE(milo): Maximum number of frames to save. Avoids using up all disk space by accident.
    while (true) {
      // yield return new WaitForSeconds(1.0f / SimulationController.CAMERA_PUBLISH_HZ);
      yield return new WaitForFixedUpdate();

      string nsec = ((long)(Time.fixedTime * 1e9)).ToString("D19");

      // Rotate the gravity vector into the IMU's frame, then add it to acceleration.
      // NOTE(milo): Really important that we transform gravity into the local frame and THEN flip
      // the sign of the y-component.
      Quaternion q_imu_world = Quaternion.Inverse(this.imuBody.transform.rotation);
      Vector3 localGravityRH = q_imu_world * Physics.gravity;
      localGravityRH.y *= -1;
      Vector3 imuBodyAccelPlusGravity = this.imuBodyAccel + localGravityRH;

      // timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]
      List<string> imu_line = new List<string>{
        nsec,
        this.imuBody.angularVelocity.x.ToString("F18"),
        this.imuBody.angularVelocity.y.ToString("F18"),
        this.imuBody.angularVelocity.z.ToString("F18"),
        imuBodyAccelPlusGravity.x.ToString("F18"),
        imuBodyAccelPlusGravity.y.ToString("F18"),
        imuBodyAccelPlusGravity.z.ToString("F18"),
        "\n"
      };

      File.AppendAllText(csv, string.Join(",", imu_line));
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

      Vector3 t_world_imu;
      Quaternion q_world_imu;
      Utils.ToRightHandedTransform(imuBody.transform, out t_world_imu, out q_world_imu);
      QuaternionMsg q_world_imu_msg = new QuaternionMsg(q_world_imu.x, q_world_imu.y, q_world_imu.z, q_world_imu.w);

      // NOTE(milo): Sending empty covariance matrix for now (denoted with -1 in the 0th position).
      double[] Zero_3x3 = new double[]{-1, 0, 0, 0, 0, 0, 0, 0, 0};

      var zero3 = new Vector3Msg(0.0, 0.0, 0.0);

      // NOTE(milo): The imuBodyAccel is already in a right-handed frame, so need to convert here.
      var accel_msg = new Vector3Msg(imuBodyAccel.x, imuBodyAccel.y, imuBodyAccel.z);
      var msg = new ImuMessage(headerMessage, q_world_imu_msg, Zero_3x3, zero3, Zero_3x3,
                               accel_msg, Zero_3x3);

      this.roslink.ros.Publish(ImuPublisher.GetMessageTopic(), msg);

      // Publish the heading (yaw) of the vehicle.
      // float theta = imuBody.transform.rotation.eulerAngles.z;
      // this.roslink.ros.Publish(HeadingPublisher.GetMessageTopic(), new Float64Msg(theta));
      // this.roslink.ros.Publish(GroundtruthHeadingPublisher.GetMessageTopic(), new Float64Msg(theta));
      // this.roslink.ros.Render();

      // Publish the current barometer depth (groundtruth and depth with simulated noise).
      float depth = t_world_imu.y;
      Float64Msg depth_msg_gt = new Float64Msg(depth);
      Float64Msg depth_msg = new Float64Msg(depth + Utils.Gaussian(0, this.depthSensorSigma));
      this.roslink.ros.Publish(DepthPublisher.GetMessageTopic(), depth_msg);
      this.roslink.ros.Publish(GroundtruthDepthPublisher.GetMessageTopic(), depth_msg_gt);
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
    string leftImageDataFolder = Path.Combine(leftImageFolder, "data");
    string rightImageDataFolder = Path.Combine(rightImageFolder, "data");
    // Debug.Log(leftImageFolder);
    // Debug.Log(rightImageFolder);

    Directory.CreateDirectory(leftImageFolder);
    Directory.CreateDirectory(rightImageFolder);
    Directory.CreateDirectory(leftImageDataFolder);
    Directory.CreateDirectory(rightImageDataFolder);

    int frame_id = 0;

    // NOTE(milo): Maximum number of frames to save. Avoids using up all disk space by accident.
    while (frame_id < 5000) {
      // yield return new WaitForSeconds(1.0f / SimulationController.CAMERA_PUBLISH_HZ);
      yield return new WaitForEndOfFrame();

      string nsec = ((long)(Time.fixedTime * 1e9)).ToString("D19");
      File.AppendAllLines(Path.Combine(datasetFolder, "timestamps.txt"), new string[] { nsec });

      Transform T_world_cam = this.camera_forward_left.transform;

      Quaternion q_world_cam;
      Vector3 t_world_cam;
      Utils.ToRightHandedTransform(T_world_cam, out t_world_cam, out q_world_cam);

      List<string> pose_line = new List<string>{
        nsec,
        q_world_cam.w.ToString(),
        q_world_cam.x.ToString(),
        q_world_cam.y.ToString(),
        q_world_cam.z.ToString(),
        t_world_cam.x.ToString(),
        t_world_cam.y.ToString(),
        t_world_cam.z.ToString(),
        "\n"
      };

      File.AppendAllText(Path.Combine(datasetFolder, "pose0.txt"), string.Join(",", pose_line));

      Texture2D leftImage = GetImageFromCamera(camera_forward_left);
      Texture2D rightImage = GetImageFromCamera(camera_forward_right);

      byte[] leftPng = leftImage.EncodeToPNG();
      byte[] rightPng = rightImage.EncodeToPNG();

      // string imagePath = $"{frame_id:000000}.png";
      string imagePath = $"{nsec}.png";

      List<string> img_line = new List<string>{ nsec, imagePath, "\n" };
      File.AppendAllText(Path.Combine(leftImageFolder, "data.csv"), string.Join(",", img_line));
      File.AppendAllText(Path.Combine(rightImageFolder, "data.csv"), string.Join(",", img_line));

      File.WriteAllBytes(Path.Combine(leftImageDataFolder, imagePath), leftPng);
      File.WriteAllBytes(Path.Combine(rightImageDataFolder, imagePath), rightPng);

      ++frame_id;
    }
  }
}
