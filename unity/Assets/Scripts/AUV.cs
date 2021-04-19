using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using System.IO;

using UnityEngine;
using Simulator;


namespace Simulator {
public enum IOMode
{
  NONE = 0,
  LCM = 1,
  DATASET = 2
}
}


public class AUV : MonoBehaviour {
  DateTime camStart = DateTime.Now;

  // Cameras attached to the vehicle.
  public Camera camera_forward_left;
  public DepthCamera depthCameraLeft;

  // Attach sensors here.
  // TODO(milo): Eventually add ability to drop messages.
  public bool publishImu = true;
  public bool publishRange = true;
  public bool publishDepth = true;
  public bool publishStereo = true;
  public float rangePublishRate = 10;
  public Rigidbody imu_rigidbody;
  public ImuSensor imu_sensor;
  public RangeSensor range_sensor_A;
  public RangeSensor range_sensor_B;
  public RangeSensor range_sensor_C;
  public RangeSensor range_sensor_D;
  public DepthSensor depth_sensor;
  public StereoRig stereo_rig;

  public Simulator.IOMode simulatorIOMode = Simulator.IOMode.NONE;
  public bool disableImageOutput = false;
  public string outputDatasetName = "default_dataset";

  // NOTE(milo): This needs to be an absolute path!
  public string outputDatasetRoot = "/home/milo/datasets/Unity3D/farmsim/";
  private string leftImageSubfolder = "cam0";
  private string rightImageSubfolder = "cam1";
  private string imuSubfolder = "imu0";
  private string apsSubfolderA = "aps0";
  private string apsSubfolderB = "aps1";
  private string apsSubfolderC = "aps2";
  private string apsSubfolderD = "aps3";
  private string depthSubfolder = "depth0";
  private string depthImageSubfolder = "depth_image0";

  // http://lcm-proj.github.io/tut_dotnet.html
  // NOTE(milo): Don't initialize this here! It will crash the Unity editor.
  private LCM.LCM.LCM lcmHandle;

  void Start()
  {
    try {
      this.lcmHandle = LCM.LCM.LCM.Singleton;
    } catch (System.ArgumentException e) {
      // https://answers.unity.com/questions/485595/uncaught-exception-doesnt-kill-the-application.html
      Debug.Log(e);
      Debug.Break();
    }

    if (this.simulatorIOMode == Simulator.IOMode.LCM) {
      if (!this.disableImageOutput) {
        StartCoroutine(PublishStereoImages());
      }
      StartCoroutine(PublishImu());
      StartCoroutine(PublishDepth());
      StartCoroutine(PublishRange());
      StartCoroutine(PublishPose());
    } else if (this.simulatorIOMode == Simulator.IOMode.DATASET) {
      if (!this.disableImageOutput) {
        StartCoroutine(SaveStereoImageDataset());
      }
      StartCoroutine(SaveImuDataset());
      StartCoroutine(SaveApsDataset());
      StartCoroutine(SaveDepthDataset());
    }
  }

  IEnumerator PublishImu()
  {
    long seq = 0;
    vehicle.imu_measurement_t imu_msg = new vehicle.imu_measurement_t();
    imu_msg.header = new vehicle.header_t();
    imu_msg.linear_acc = new vehicle.vector3_t();
    imu_msg.angular_vel = new vehicle.vector3_t();

    while (true) {
      yield return new WaitForFixedUpdate();

      ImuMeasurement data = this.imu_sensor.data;

      LCMUtils.pack_header_t(data.timestamp, seq, "imu0", ref imu_msg.header);
      LCMUtils.pack_vector3_t(data.imu_a_rh, ref imu_msg.linear_acc);
      LCMUtils.pack_vector3_t(data.imu_w_rh, ref imu_msg.angular_vel);

      this.lcmHandle.Publish(SimulationParams.CHANNEL_AUV_IMU, imu_msg);

      ++seq;
    }
  }

  IEnumerator PublishDepth()
  {
    long seq = 0;
    vehicle.depth_measurement_t depth_msg = new vehicle.depth_measurement_t();
    depth_msg.header = new vehicle.header_t();

    while (true) {
      yield return new WaitForEndOfFrame();

      this.depth_sensor.Read();
      DepthMeasurement data = this.depth_sensor.data;

      LCMUtils.pack_header_t(data.timestamp, seq, "depth0", ref depth_msg.header);
      depth_msg.depth = data.depth;

      this.lcmHandle.Publish(SimulationParams.CHANNEL_AUV_DEPTH, depth_msg);

      ++seq;
    }
  }

  IEnumerator PublishRange()
  {
    long seq = 0;
    vehicle.range_measurement_t msg0 = new vehicle.range_measurement_t();
    vehicle.range_measurement_t msg1 = new vehicle.range_measurement_t();
    vehicle.range_measurement_t msg2 = new vehicle.range_measurement_t();
    vehicle.range_measurement_t msg3 = new vehicle.range_measurement_t();
    msg0.header = new vehicle.header_t();
    msg1.header = new vehicle.header_t();
    msg2.header = new vehicle.header_t();
    msg3.header = new vehicle.header_t();
    msg0.point = new vehicle.vector3_t();
    msg1.point = new vehicle.vector3_t();
    msg2.point = new vehicle.vector3_t();
    msg3.point = new vehicle.vector3_t();

    float range_wait_time = 1.0f / this.rangePublishRate;

    while (true) {
      yield return new WaitForSeconds(range_wait_time);

      if (this.range_sensor_A) {
        this.range_sensor_A.Read();
        RangeMeasurement data0 = this.range_sensor_A.data;
        LCMUtils.pack_header_t(data0.timestamp, seq, "aps_receiver", ref msg0.header);
        LCMUtils.pack_vector3_t(data0.world_t_beacon, ref msg0.point);
        msg0.range = data0.range;
        this.lcmHandle.Publish(SimulationParams.CHANNEL_AUV_RANGE_ALL, msg0);
      }

      if (this.range_sensor_B) {
        this.range_sensor_B.Read();
        RangeMeasurement data1 = this.range_sensor_B.data;
        LCMUtils.pack_header_t(data1.timestamp, seq, "aps_receiver", ref msg1.header);
        LCMUtils.pack_vector3_t(data1.world_t_beacon, ref msg1.point);
        msg1.range = data1.range;
        this.lcmHandle.Publish(SimulationParams.CHANNEL_AUV_RANGE_ALL, msg1);
      }

      if (this.range_sensor_C) {
        this.range_sensor_C.Read();
        RangeMeasurement data2 = this.range_sensor_C.data;
        LCMUtils.pack_header_t(data2.timestamp, seq, "aps_receiver", ref msg2.header);
        LCMUtils.pack_vector3_t(data2.world_t_beacon, ref msg2.point);
        msg2.range = data2.range;
        this.lcmHandle.Publish(SimulationParams.CHANNEL_AUV_RANGE_ALL, msg2);
      }

      if (this.range_sensor_D) {
        this.range_sensor_D.Read();
        RangeMeasurement data3 = this.range_sensor_D.data;
        LCMUtils.pack_header_t(data3.timestamp, seq, "aps_receiver", ref msg3.header);
        LCMUtils.pack_vector3_t(data3.world_t_beacon, ref msg3.point);
        msg3.range = data3.range;
        this.lcmHandle.Publish(SimulationParams.CHANNEL_AUV_RANGE_ALL, msg3);
      }

      ++seq;
    }
  }

  IEnumerator PublishPose()
  {
    long seq = 0;
    vehicle.pose3_stamped_t msg = new vehicle.pose3_stamped_t();
    msg.header = new vehicle.header_t();
    msg.pose = new vehicle.pose3_t();
    msg.pose.orientation = new vehicle.quaternion_t();
    msg.pose.position = new vehicle.vector3_t();

    Quaternion world_q_body = Quaternion.identity;
    Vector3 world_t_body = Vector3.zero;

    while (true) {
      yield return new WaitForEndOfFrame();

      Transform world_T_body = this.imu_rigidbody.transform;

      TransformUtils.ToRightHandedTransform(world_T_body, ref world_t_body, ref world_q_body);

      LCMUtils.pack_header_t(Timestamp.UnityNanoseconds(), seq, "body", ref msg.header);
      LCMUtils.pack_pose3_t(world_q_body, world_t_body, ref msg.pose);

      this.lcmHandle.Publish(SimulationParams.CHANNEL_AUV_WORLD_P_IMU, msg);
      this.lcmHandle.Publish(SimulationParams.CHANNEL_AUV_WORLD_P_IMU_INITIAL, msg);

      ++seq;
    }
  }

  IEnumerator PublishStereoImages()
  {
    long seq = 0;

    // Only allocate these things once.
    Texture2D leftImage = new Texture2D(SimulationParams.AUV_CAMERA_WIDTH, SimulationParams.AUV_CAMERA_HEIGHT, TextureFormat.RGB24, false);
    Texture2D rightImage = new Texture2D(SimulationParams.AUV_CAMERA_WIDTH, SimulationParams.AUV_CAMERA_HEIGHT, TextureFormat.RGB24, false);
    Texture2D.allowThreadedTextureCreation = true;

    vehicle.stereo_image_t msg = new vehicle.stereo_image_t();
    msg.header = new vehicle.header_t();
    msg.img_left = new vehicle.image_t();
    msg.img_right = new vehicle.image_t();

    SharedImagePublisher shmem_pub = new SharedImagePublisher(
        SimulationParams.CHANNEL_AUV_STEREO_SHMEM,
        SimulationParams.AUV_CAMERA_HEIGHT,
        SimulationParams.AUV_CAMERA_WIDTH, 3);

    while (true) {
      // yield return new WaitForSeconds(1.0f / SimulationParams.CAMERA_PUBLISH_HZ);
      yield return new WaitForEndOfFrame();

      this.stereo_rig.CaptureStereoPair(ref leftImage, ref rightImage);

      LCMUtils.pack_header_t(Timestamp.UnityNanoseconds(), seq, "stereo_cam", ref msg.header);
      // LCMUtils.pack_image_t(ref leftImage, ref msg.img_left);
      // LCMUtils.pack_image_t(ref rightImage, ref msg.img_right);

      // this.lcmHandle.Publish(SimulationParams.CHANNEL_AUV_STEREO, msg);
      // SharedImagePublisher.PublishStereoImage(SimulationParams.CHANNEL_AUV_STEREO, ref msg, 1);
      shmem_pub.PublishStereo(ref msg.header, ref leftImage, ref rightImage);

      ++seq;
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

      // Don't need to call Read() because sensor is updated during FixedUpdate().
      ImuMeasurement data = this.imu_sensor.data;

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

      this.depth_sensor.Read();
      DepthMeasurement data = this.depth_sensor.data;

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
    string aps_folder_A = Path.Combine(dataset_folder, this.apsSubfolderA);
    string aps_folder_B = Path.Combine(dataset_folder, this.apsSubfolderB);

    if (Directory.Exists(aps_folder_A)) {
      Debug.Log("WARNING: Deleting existing APS folder: " + aps_folder_A);
      Directory.Delete(aps_folder_A, true);
    }
    if (Directory.Exists(aps_folder_B)) {
      Debug.Log("WARNING: Deleting existing APS folder: " + aps_folder_B);
      Directory.Delete(aps_folder_B, true);
    }

    Directory.CreateDirectory(aps_folder_A);
    Directory.CreateDirectory(aps_folder_B);

    string csv_A = Path.Combine(aps_folder_A, "data.csv");
    string csv_B = Path.Combine(aps_folder_B, "data.csv");

    float range_wait_time = 1.0f / this.rangePublishRate;

    while (true) {
      // ping_time = max_range / speed_of_sound = 100m / 343 m/s = 0.29 sec
      // Therefore can run at most 3ish Hz at 100m max range.
      yield return new WaitForSeconds(range_wait_time);
      this.range_sensor_A.Read();
      this.range_sensor_B.Read();
      RangeMeasurement data_A = this.range_sensor_A.data;
      RangeMeasurement data_B = this.range_sensor_B.data;

      // timestamp [ns], range [m], world_t_beacon.x [m], world_t_beacon.y [m], world_t_beacon.z [m],
      List<string> line_A = new List<string>{
        data_A.timestamp.ToString("D19"),
        data_A.range.ToString("F18"),
        data_A.world_t_beacon.x.ToString("F18"),
        data_A.world_t_beacon.y.ToString("F18"),
        data_A.world_t_beacon.z.ToString("F18"),
        "\n"
      };

      List<string> line_B = new List<string>{
        data_B.timestamp.ToString("D19"),
        data_B.range.ToString("F18"),
        data_B.world_t_beacon.x.ToString("F18"),
        data_B.world_t_beacon.y.ToString("F18"),
        data_B.world_t_beacon.z.ToString("F18"),
        "\n"
      };

      File.AppendAllText(csv_A, string.Join(",", line_A));
      File.AppendAllText(csv_B, string.Join(",", line_B));
    }
  }

  IEnumerator SaveStereoImageDataset()
  {
    string dataset_folder = Path.Combine(this.outputDatasetRoot, this.outputDatasetName);

    if (Directory.Exists(dataset_folder)) {
      Debug.Log("WARNING: Deleting existing dataset folder: " + dataset_folder);
      Directory.Delete(dataset_folder, true);
    }

    string leftImageFolder = Path.Combine(dataset_folder, this.leftImageSubfolder);
    string rightImageFolder = Path.Combine(dataset_folder, this.rightImageSubfolder);
    string depthImageFolder = Path.Combine(dataset_folder, this.depthImageSubfolder);
    string leftImageDataFolder = Path.Combine(leftImageFolder, "data");
    string rightImageDataFolder = Path.Combine(rightImageFolder, "data");
    string depthImageDataFolder = Path.Combine(depthImageFolder, "data");

    Directory.CreateDirectory(leftImageFolder);
    Directory.CreateDirectory(rightImageFolder);
    Directory.CreateDirectory(leftImageDataFolder);
    Directory.CreateDirectory(rightImageDataFolder);
    Directory.CreateDirectory(depthImageDataFolder);

    int frame_id = 0;
    Texture2D leftImage = new Texture2D(SimulationParams.AUV_CAMERA_WIDTH, SimulationParams.AUV_CAMERA_HEIGHT, TextureFormat.RGB24, false);
    Texture2D rightImage = new Texture2D(SimulationParams.AUV_CAMERA_WIDTH, SimulationParams.AUV_CAMERA_HEIGHT, TextureFormat.RGB24, false);
    Texture2D leftDepthImage = new Texture2D(SimulationParams.AUV_CAMERA_WIDTH, SimulationParams.AUV_CAMERA_HEIGHT, TextureFormat.RGB24, false);
    byte[] leftPng, rightPng, depthPng;

    Quaternion q_world_cam = Quaternion.identity;
    Vector3 t_world_cam = Vector3.zero;

    Quaternion q_world_body = Quaternion.identity;
    Vector3 t_world_body = Vector3.zero;

    // NOTE(milo): Maximum number of frames to save. Avoids using up all disk space by accident.
    while (frame_id < 5000) {
      yield return new WaitForEndOfFrame();

      string nsec = ((long)(Time.fixedTime * 1e9)).ToString("D19");
      File.AppendAllLines(Path.Combine(dataset_folder, "timestamps.txt"), new string[] { nsec });

      //==================================== LEFT CAMERA POSE ======================================
      Transform T_world_cam = this.camera_forward_left.transform;

      TransformUtils.ToRightHandedTransform(T_world_cam, ref t_world_cam, ref q_world_cam);

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

      TransformUtils.ToRightHandedTransform(T_world_body, ref t_world_body, ref q_world_body);

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

      this.stereo_rig.CaptureStereoPair(ref leftImage, ref rightImage);
      this.depthCameraLeft.Capture(ref leftDepthImage);

      leftPng = leftImage.EncodeToPNG();
      rightPng = rightImage.EncodeToPNG();
      depthPng = leftDepthImage.EncodeToPNG();

      string imagePath = $"{nsec}.png";

      List<string> img_line = new List<string>{ nsec, imagePath, "\n" };
      File.AppendAllText(Path.Combine(leftImageFolder, "data.csv"), string.Join(",", img_line));
      File.AppendAllText(Path.Combine(rightImageFolder, "data.csv"), string.Join(",", img_line));

      File.WriteAllBytes(Path.Combine(leftImageDataFolder, imagePath), leftPng);
      File.WriteAllBytes(Path.Combine(rightImageDataFolder, imagePath), rightPng);
      File.WriteAllBytes(Path.Combine(depthImageDataFolder, imagePath), depthPng);

      ++frame_id;
    }

    Debug.Log("WARNING: Stopped saving camera images! Reached maximum.");
  }
}
