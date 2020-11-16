using System;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using ROSBridgeLib.std_msgs;
using ROSBridgeLib.sensor_msgs;
using ROSBridgeLib.geometry_msgs;
using ROSBridgeLib.CustomMessages;
using ROSBridgeLib;
using System.Text;
using System.IO;


public class AUV : MonoBehaviour {
  private ROSBridgeWebSocketConnection ros = null;
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

  private List<IEnumerator> routines;
  private Quaternion rotationFaceWinch = Quaternion.identity;

  void Start()
  {
    ros = new ROSBridgeWebSocketConnection("ws://localhost", 9090);

    // Add a publisher for each of the cameras that we want to support.
    ros.AddPublisher(typeof(CameraForwardLeftPublisher));
    ros.AddPublisher(typeof(CameraForwardRightPublisher));
    ros.AddPublisher(typeof(CameraDownwardLeftPublisher));
    ros.AddPublisher(typeof(CameraUpwardLeftPublisher));

    ros.AddPublisher(typeof(DepthPublisher));
    ros.AddPublisher(typeof(GroundtruthDepthPublisher));
    ros.AddPublisher(typeof(PoseStampedPublisher));

    ros.AddPublisher(typeof(ImuPublisher));
    ros.AddPublisher(typeof(HeadingPublisher));

    ros.Connect();

    msgPublishCount = 0;
    lastFrame = DateTime.Now;
    camStart = DateTime.Now;

    //============================== DEMO SETUP ================================
    this.routines = new List<IEnumerator>{
      this.AnimateWaypoint(this.gameObject, this.farm.GetWinchLocation(0, 'A').position + new Vector3(0, 0, -1.5f), rotationFaceWinch, 10),
      this.AnimateWaypoint(this.gameObject, this.farm.GetWinchLocation(0, 'B').position + new Vector3(0, 0, -1.5f), rotationFaceWinch, 10),
      this.AnimateWaypoint(this.gameObject, this.farm.GetWinchLocation(0, 'C').position + new Vector3(0, 0, -1.5f), rotationFaceWinch, 10),
    };

    StartCoroutine(ChainCoroutines(this.routines));
  }

  void Update()
  {
    StartCoroutine(PublishCameraSyncedMessages());
    StartCoroutine(SendDepth());
    StartCoroutine(PublishImu());
    StartCoroutine(SendHeading());
  }

  void FixedUpdate()
  {
    imuBodyAccel = (imuBody.velocity - lastImuBodyVelocity) / Time.fixedDeltaTime;
    lastImuBodyVelocity = imuBody.velocity;
  }

  void OnApplicationQuit()
  {
    if (ros != null) {
      ros.Disconnect();
    }
  }

  /**
   * Grabs an image from a camera and returns it.
   */
  Texture2D GetImageFromCamera(Camera camera)
  {
    // Create the RenderTexture for the first time if it hasn't been created.
    // NOTE(milo): Using the ARGB32 format since it's in tutorials, not sure what the best option is
    // here though. It uses 8 bits per RGB channel, which seems standard.
    if (rt == null) {
      rt = new RenderTexture(640, 480, 16, RenderTextureFormat.ARGB32);
      rt.Create();
    }

    // NOTE(milo): Documentation on camera rendering is a little confusing.
    // We instantiate a RenderTexture above, which is basically just a buffer that cameras render
    // into. Then, we call Render() and read the rendered image into a Texture2D with ReadPixels().
    camera.targetTexture = rt;
    camera.Render();

    // Make a new (empty) image and read the camera image into it.
    Texture2D image = new Texture2D(camera.targetTexture.width, camera.targetTexture.height,
                                    TextureFormat.RGB24, false);
    image.ReadPixels(new Rect(0, 0, camera.targetTexture.width, camera.targetTexture.height), 0, 0);
    image.Apply();

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

    ros.Publish(topic, compressedImageMsg);
    Destroy(im);
    ros.Render();
  }

  /**
   * Any message that is meant to be synchronized with camera images should be published here.
   */
  IEnumerator PublishCameraSyncedMessages()
  {
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

    PublishCameraImage(
        GetImageFromCamera(camera_upward_left),
        timeMessage,
        new HeaderMsg(msgPublishCount, timeMessage, "auv_camera_ul"),
        CameraUpwardLeftPublisher.GetMessageTopic());

    // Publish the pose of the vehicle in the world.
    Vector3 t_auv_world = imuBody.transform.position;
    Quaternion q_auv_world = imuBody.transform.rotation;

    QuaternionMsg q_auv_world_msg = new QuaternionMsg(
        q_auv_world.x, q_auv_world.y, q_auv_world.z, q_auv_world.w);
    PointMsg t_auv_world_msg = new PointMsg(t_auv_world.x, t_auv_world.y, t_auv_world.z);
    PoseStampedMsg msg = new PoseStampedMsg(new HeaderMsg(msgPublishCount, timeMessage, "auv_imu"),
                                            new PoseMsg(t_auv_world_msg, q_auv_world_msg));
    ros.Publish(PoseStampedPublisher.GetMessageTopic(), msg);
    ros.Render();
  }

  // TODO(milo): Add simulated noise to the depth value.
  IEnumerator SendDepth() {
    yield return new WaitForEndOfFrame();

    float depth = imuBody.transform.position.y;
    var depthMessage = new Float64Msg(depth);

    ros.Publish(DepthPublisher.GetMessageTopic(), depthMessage);
    ros.Publish(GroundtruthDepthPublisher.GetMessageTopic(), depthMessage);
    ros.Render();
  }


  IEnumerator PublishImu() {
    yield return new WaitForEndOfFrame();

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

    ros.Publish(ImuPublisher.GetMessageTopic(), msg);
    ros.Render();
  }

  /**
   * Publish the heading (yaw) of the vehicle.
   */
  IEnumerator SendHeading() {
    yield return new WaitForEndOfFrame();

    float theta = imuBody.transform.rotation.eulerAngles.z;
    ros.Publish(HeadingPublisher.GetMessageTopic(), new Float64Msg(theta));
    ros.Render();
  }

  /**
   * Animates the vehicle moving between two waypoints.
   */
  IEnumerator AnimateMotion(GameObject vehicle,
                            Vector3 t_start,
                            Quaternion q_start,
                            Vector3 t_end,
                            Quaternion q_end,
                            float sec)
  {
    float startTime = Time.time;
    float elap = (Time.time - startTime);
    while (elap < sec) {
      elap = (Time.time - startTime);
      float T = Mathf.Clamp(elap / sec, 0, 1); // Compute interpolation amount.

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
   IEnumerator AnimateWaypoint(GameObject vehicle, Vector3 t_end, Quaternion q_end, float sec)
   {
    float startTime = Time.time;
    float elap = (Time.time - startTime);

    // NOTE(milo): Need to grab the start transform HERE so that it's up-to-date when this coroutine
    // starts. If it was an argument, it would be pinned to whatever location the vehicle was at
    // upon instantiating the coroutine.
    Vector3 t_start = vehicle.transform.position;
    Quaternion q_start = vehicle.transform.rotation;

    while (elap < sec) {
      elap = (Time.time - startTime);

      // First half of the trajectory (accelerating).
      float T = 0;
      if (elap < (sec / 2.0f)) {
        T = (2.0f / Mathf.Pow(sec, 2.0f)) * Mathf.Pow(elap, 2.0f);

      // Second half of the trajectory (decelerating).
      } else {
        T = 1 - (2.0f / Mathf.Pow(sec, 2.0f)) * Mathf.Pow((sec - elap), 2.0f);
      }

      T = Mathf.Clamp(T, 0, 1); // Compute interpolation amount.

      // Linear interpolation between the two endpoints, slerp between quaternions.
      vehicle.transform.position = (1 - T)*t_start + T*t_end;
      vehicle.transform.rotation = Quaternion.Slerp(q_start, q_end, T);

      // Yield progress.
      yield return T;
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
}
