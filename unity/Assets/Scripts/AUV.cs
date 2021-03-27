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


public class AUV : MonoBehaviour
{
  private int msgPublishCount;
  DateTime camStart;

  // Cameras attached to the vehicle.
  public Camera camera_forward_left;
  public Camera camera_forward_right;

  public Rigidbody imu_rigidbody;
  public ImuSensor imu_sensor;
  public RangeSensor aps_sensor;
  public DepthSensor depth_sensor;

  private ROSMessageHolder roslink;
  public Simulator.IOMode simulatorIOMode = Simulator.IOMode.NO_OUTPUT;
  public string outputDatasetName = "default_dataset";

  // NOTE(milo): This needs to be an absolute path!
  public string outputDatasetRoot = "/home/milo/datasets/Unity3D/farmsim/";
  private string leftImageSubfolder = "cam0";
  private string rightImageSubfolder = "cam1";
  private string imuSubfolder = "imu0";
  private string apsSubfolder = "aps0";
  private string depthSubfolder = "depth0";

  private RenderTexture _preallocRT;

  void Start()
  {
    this._preallocRT = new RenderTexture(
        SimulationController.AUV_CAMERA_WIDTH,
        SimulationController.AUV_CAMERA_HEIGHT,
        16, RenderTextureFormat.ARGB32);

    this.roslink = GameObject.Find("ROSMessageHolder").GetComponent<ROSMessageHolder>();

    // Add a publisher for each of the cameras that we want to support.
    this.roslink.ros.AddPublisher(typeof(StereoCamLeftPublisherCmp));
    this.roslink.ros.AddPublisher(typeof(StereoCamRightPublisherCmp));

    // this.roslink.ros.AddPublisher(typeof(StereoCamLeftPublisher));
    // this.roslink.ros.AddPublisher(typeof(StereoCamRightPublisher));

    // this.roslink.ros.AddPublisher(typeof(DepthPublisher));
    // this.roslink.ros.AddPublisher(typeof(GroundtruthDepthPublisher));
    this.roslink.ros.AddPublisher(typeof(ImuPosePublisher));
    // this.roslink.ros.AddPublisher(typeof(StereoCamLeftPosePublisher));
    // this.roslink.ros.AddPublisher(typeof(StereoCamRightPosePublisher));

    // this.roslink.ros.AddPublisher(typeof(ImuPublisher));
    // this.roslink.ros.AddPublisher(typeof(HeadingPublisher));
    // this.roslink.ros.AddPublisher(typeof(GroundtruthHeadingPublisher));

    this.msgPublishCount = 0;
    this.camStart = DateTime.Now;

    if (this.simulatorIOMode == Simulator.IOMode.PUBLISH_TO_ROS) {
      StartCoroutine(PublishCameraSyncedMessages());
      // StartCoroutine(PublishImu());
    } else if (this.simulatorIOMode == Simulator.IOMode.SAVE_TO_DISK) {
      StartCoroutine(SaveStereoImageDataset());
      StartCoroutine(SaveImuDataset());
      StartCoroutine(SaveApsDataset());
      StartCoroutine(SaveDepthDataset());
    }
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

  // NOTE(milo): This doesn't work! Don't use yet.
  void PublishRawImage(Texture2D im, TimeMsg timeMessage, HeaderMsg headerMessage, string topic)
  {
    // https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
    bool is_bigendian = true;               // https://stackoverflow.com/questions/30423737/bitmap-from-byte-array
    uint row_step = (uint)(im.width * 24);  // Row length in bytes.

    // NOTE(milo): The row order is flipped! Not sure how to fix this.
    byte[] data = im.GetRawTextureData();

    // http://docs.ros.org/en/jade/api/sensor_msgs/html/image__encodings_8h_source.html
    var msg = new ImageMsg(headerMessage, (uint)im.height, (uint)im.width, "rgb8", is_bigendian, row_step, data);
    this.roslink.ros.Publish(topic, msg);
    Destroy(im);

    this.roslink.ros.Render();
  }

  // Publishes a CompressedImageMsg on 'topic' using the given image 'im'.
  void PublishCameraImage(Texture2D im, TimeMsg timeMessage, HeaderMsg headerMessage, string topic)
  {
    byte[] data = im.EncodeToPNG();
    string format = "png";
    var compressedImageMsg = new CompressedImageMsg(headerMessage, format, data);
    this.roslink.ros.Publish(topic, compressedImageMsg);
    Destroy(im);
    this.roslink.ros.Render();
  }

  // Any message that is meant to be synchronized with camera images should be published here.
  IEnumerator PublishCameraSyncedMessages()
  {
    while (true) {
      yield return new WaitForSeconds(1.0f / SimulationController.CAMERA_PUBLISH_HZ);
      yield return new WaitForEndOfFrame();

      var now = DateTime.Now;
      var timeSinceStart = now - camStart;
      var timeMessage = new TimeMsg(timeSinceStart.Seconds, timeSinceStart.Milliseconds);

      PublishCameraImage(
          GetImageFromCamera(camera_forward_left),
          timeMessage,
          new HeaderMsg(msgPublishCount, timeMessage, "cam0"),
          StereoCamLeftPublisherCmp.GetMessageTopic());

      PublishCameraImage(
          GetImageFromCamera(camera_forward_right),
          timeMessage,
          new HeaderMsg(msgPublishCount, timeMessage, "cam1"),
          StereoCamRightPublisherCmp.GetMessageTopic());

      // Publish the pose of the vehicle in the world (right-handed!).
      Transform T_world_imu = imu_rigidbody.transform;
      Vector3 t_world_imu;
      Quaternion q_world_imu;
      TransformUtils.ToRightHandedTransform(T_world_imu, out t_world_imu, out q_world_imu);

      QuaternionMsg q_world_imu_msg = new QuaternionMsg(q_world_imu.x, q_world_imu.y, q_world_imu.z, q_world_imu.w);
      PointMsg t_world_imu_msg = new PointMsg(t_world_imu.x, t_world_imu.y, t_world_imu.z);
      PoseStampedMsg msg = new PoseStampedMsg(new HeaderMsg(msgPublishCount, timeMessage, "imu"),
                                              new PoseMsg(t_world_imu_msg, q_world_imu_msg));
      this.roslink.ros.Publish(ImuPosePublisher.GetMessageTopic(), msg);
      this.roslink.ros.Render();
    }
  }

  IEnumerator SaveImuDataset() {
    string dataset_folder = Path.Combine(this.outputDatasetRoot, this.outputDatasetName);
    string imu_folder = Path.Combine(dataset_folder, this.imuSubfolder);

    if (Directory.Exists(imu_folder)) {
      Debug.Log("WARNING: Deleting existing IMU folder: " + imu_folder);
      Directory.Delete(imu_folder, true);
    }

    Directory.CreateDirectory(imu_folder);

    string csv = Path.Combine(imu_folder, "data.csv");

    // NOTE(milo): Maximum number of frames to save. Avoids using up all disk space by accident.
    while (true) {
      yield return new WaitForFixedUpdate();

      ImuMeasurement data = this.imu_sensor.Read();

      // We use the EuRoC MAV format:
      // timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]
      List<string> imu_line = new List<string>{
        data.timestamp.ToString("D19"),
        data.imu_w_rh.x.ToString("F18"),
        data.imu_w_rh.y.ToString("F18"),
        data.imu_w_rh.z.ToString("F18"),
        data.imu_a_rh.x.ToString("F18"),
        data.imu_a_rh.y.ToString("F18"),
        data.imu_a_rh.z.ToString("F18"),
        "\n"
      };

      File.AppendAllText(csv, string.Join(",", imu_line));
    }
  }

  IEnumerator SaveDepthDataset()
  {
    string dataset_folder = Path.Combine(this.outputDatasetRoot, this.outputDatasetName);
    string depth_folder = Path.Combine(dataset_folder, this.depthSubfolder);

    if (Directory.Exists(depth_folder)) {
      Debug.Log("WARNING: Deleting existing depth folder: " + depth_folder);
      Directory.Delete(depth_folder, true);
    }

    Directory.CreateDirectory(depth_folder);

    string csv = Path.Combine(depth_folder, "data.csv");

    // NOTE(milo): Maximum number of frames to save. Avoids using up all disk space by accident.
    while (true) {
      yield return new WaitForFixedUpdate();

      DepthMeasurement data = this.depth_sensor.Read();

      // timestamp [ns], depth [m]
      List<string> line = new List<string>{
        data.timestamp.ToString("D19"),
        data.depth.ToString("F18"),
        "\n"
      };

      File.AppendAllText(csv, string.Join(",", line));
    }
  }

  IEnumerator SaveApsDataset()
  {
    string dataset_folder = Path.Combine(this.outputDatasetRoot, this.outputDatasetName);
    string aps_folder = Path.Combine(dataset_folder, this.apsSubfolder);

    if (Directory.Exists(aps_folder)) {
      Debug.Log("WARNING: Deleting existing APS folder: " + aps_folder);
      Directory.Delete(aps_folder, true);
    }

    Directory.CreateDirectory(aps_folder);

    string csv = Path.Combine(aps_folder, "data.csv");

    while (true) {
      // ping_time = max_range / speed_of_sound = 100m / 343 m/s = 0.29 sec
      // Therefore can run at most 3ish Hz at 100m max range.
      yield return new WaitForSeconds(0.333f);
      RangeMeasurement data = this.aps_sensor.Read();

      // timestamp [ns], range [m], world_t_beacon.x [m], world_t_beacon.y [m], world_t_beacon.z [m],
      List<string> line = new List<string>{
        data.timestamp.ToString("D19"),
        data.range.ToString("F18"),
        data.world_t_beacon.x.ToString("F18"),
        data.world_t_beacon.y.ToString("F18"),
        data.world_t_beacon.z.ToString("F18"),
        "\n"
      };

      File.AppendAllText(csv, string.Join(",", line));
    }
  }

  // TODO(milo): Redo this with correct right-handed transforms!
  // IEnumerator PublishImu() {
  //   while (true) {
  //     // yield return new WaitForEndOfFrame();
  //     yield return new WaitForSeconds(1.0f / SimulationController.SENSOR_PUBLISH_HZ);

  //     var now = DateTime.Now;
  //     var timeSinceStart = now - camStart;
  //     var timeMessage = new TimeMsg(timeSinceStart.Seconds, timeSinceStart.Milliseconds);
  //     var headerMessage = new HeaderMsg(msgPublishCount, timeMessage, "imu");

  //     Vector3 t_world_imu;
  //     Quaternion q_world_imu;
  //     Utils.ToRightHandedTransform(imu_rigidbody.transform, out t_world_imu, out q_world_imu);
  //     QuaternionMsg q_world_imu_msg = new QuaternionMsg(q_world_imu.x, q_world_imu.y, q_world_imu.z, q_world_imu.w);

  //     // NOTE(milo): Sending empty covariance matrix for now (denoted with -1 in the 0th position).
  //     double[] Zero_3x3 = new double[]{-1, 0, 0, 0, 0, 0, 0, 0, 0};

  //     var zero3 = new Vector3Msg(0.0, 0.0, 0.0);

  //     // NOTE(milo): The imu_linear_accel is already in a right-handed frame, so need to convert here.
  //     var accel_msg = new Vector3Msg(imu_linear_accel.x, imu_linear_accel.y, imu_linear_accel.z);
  //     var msg = new ImuMessage(headerMessage, q_world_imu_msg, Zero_3x3, zero3, Zero_3x3,
  //                              accel_msg, Zero_3x3);

  //     this.roslink.ros.Publish(ImuPublisher.GetMessageTopic(), msg);

  //     // Publish the heading (yaw) of the vehicle.
  //     // float theta = imu_rigidbody.transform.rotation.eulerAngles.z;
  //     // this.roslink.ros.Publish(HeadingPublisher.GetMessageTopic(), new Float64Msg(theta));
  //     // this.roslink.ros.Publish(GroundtruthHeadingPublisher.GetMessageTopic(), new Float64Msg(theta));
  //     // this.roslink.ros.Render();

  //     // Publish the current barometer depth (groundtruth and depth with simulated noise).
  //     float depth = t_world_imu.y;
  //     Float64Msg depth_msg_gt = new Float64Msg(depth);
  //     Float64Msg depth_msg = new Float64Msg(depth + Gaussian.Sample1D(0, this.depth_sensor_sigma));
  //     this.roslink.ros.Publish(DepthPublisher.GetMessageTopic(), depth_msg);
  //     this.roslink.ros.Publish(GroundtruthDepthPublisher.GetMessageTopic(), depth_msg_gt);
  //   }
  // }

  IEnumerator SaveStereoImageDataset()
  {
    string dataset_folder = Path.Combine(this.outputDatasetRoot, this.outputDatasetName);

    if (Directory.Exists(dataset_folder)) {
      Debug.Log("WARNING: Deleting existing dataset folder: " + dataset_folder);
      Directory.Delete(dataset_folder, true);
    }

    string leftImageFolder = Path.Combine(dataset_folder, this.leftImageSubfolder);
    string rightImageFolder = Path.Combine(dataset_folder, this.rightImageSubfolder);
    string leftImageDataFolder = Path.Combine(leftImageFolder, "data");
    string rightImageDataFolder = Path.Combine(rightImageFolder, "data");

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
      File.AppendAllLines(Path.Combine(dataset_folder, "timestamps.txt"), new string[] { nsec });

      //==================================== LEFT CAMERA POSE ======================================
      Transform T_world_cam = this.camera_forward_left.transform;

      Quaternion q_world_cam;
      Vector3 t_world_cam;
      TransformUtils.ToRightHandedTransform(T_world_cam, out t_world_cam, out q_world_cam);

      List<string> pose_line = new List<string>{
        nsec,
        q_world_cam.w.ToString("F5"),
        q_world_cam.x.ToString("F5"),
        q_world_cam.y.ToString("F5"),
        q_world_cam.z.ToString("F5"),
        t_world_cam.x.ToString("F5"),
        t_world_cam.y.ToString("F5"),
        t_world_cam.z.ToString("F5"),
        "\n"
      };

      File.AppendAllText(Path.Combine(dataset_folder, "cam0_poses.txt"), string.Join(",", pose_line));

      //======================================== BODY POSE =========================================
      Transform T_world_body = this.imu_rigidbody.transform;

      Quaternion q_world_body;
      Vector3 t_world_body;
      TransformUtils.ToRightHandedTransform(T_world_body, out t_world_body, out q_world_body);

      pose_line = new List<string>{
        nsec,
        q_world_body.w.ToString("F5"),
        q_world_body.x.ToString("F5"),
        q_world_body.y.ToString("F5"),
        q_world_body.z.ToString("F5"),
        t_world_body.x.ToString("F5"),
        t_world_body.y.ToString("F5"),
        t_world_body.z.ToString("F5"),
        "\n"
      };

      File.AppendAllText(Path.Combine(dataset_folder, "imu0_poses.txt"), string.Join(",", pose_line));

      Texture2D leftImage = GetImageFromCamera(camera_forward_left);
      Texture2D rightImage = GetImageFromCamera(camera_forward_right);

      byte[] leftPng = leftImage.EncodeToPNG();
      byte[] rightPng = rightImage.EncodeToPNG();

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
