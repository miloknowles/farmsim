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
  public Camera camera_forward_right;

  // Attach sensors here.
  public Rigidbody imu_rigidbody;
  public ImuSensor imu_sensor;
  public RangeSensor range_sensor_A;
  public RangeSensor range_sensor_B;
  public DepthSensor depth_sensor;
  public StereoRig stereo_rig;

  public Simulator.IOMode simulatorIOMode = Simulator.IOMode.NONE;
  public string outputDatasetName = "default_dataset";

  // NOTE(milo): This needs to be an absolute path!
  public string outputDatasetRoot = "/home/milo/datasets/Unity3D/farmsim/";
  private string leftImageSubfolder = "cam0";
  private string rightImageSubfolder = "cam1";
  private string imuSubfolder = "imu0";
  private string apsSubfolderA = "aps0";
  private string apsSubfolderB = "aps1";
  private string depthSubfolder = "depth0";

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
      // StartCoroutine(PublishCameraSyncedMessages());
      StartCoroutine(PublishImu());
    } else if (this.simulatorIOMode == Simulator.IOMode.DATASET) {
      StartCoroutine(SaveStereoImageDataset());
      StartCoroutine(SaveImuDataset());
      StartCoroutine(SaveApsDataset());
      StartCoroutine(SaveDepthDataset());
    }
  }

  IEnumerator PublishImu()
  {
    long seq = 0;

    while (true) {
      yield return new WaitForFixedUpdate();

      vehicle.header_t header_msg = new vehicle.header_t();
      header_msg.timestamp = Timestamp.UnityNanoseconds();
      header_msg.seq = seq;
      header_msg.frame_id = "imu0";

      vehicle.imu_measurement_t imu_msg = new vehicle.imu_measurement_t();
      imu_msg.header = header_msg;
      imu_msg.linear_acc = new vehicle.point3_t();
      imu_msg.angular_vel = new vehicle.point3_t();

      this.lcmHandle.Publish(SimulationParams.CHANNEL_AUV_IMU, imu_msg);

      ++seq;
    }
  }

  // Any message that is meant to be synchronized with camera images should be published here.
  IEnumerator PublishCameraSyncedMessages()
  {
    Texture2D leftImage, rightImage;

    while (true) {
      yield return new WaitForSeconds(1.0f / SimulationParams.CAMERA_PUBLISH_HZ);
      yield return new WaitForEndOfFrame();

      var now = DateTime.Now;
      var timeSinceStart = now - camStart;
      // var timeMessage = new TimeMsg(timeSinceStart.Seconds, timeSinceStart.Milliseconds);

      this.stereo_rig.CaptureStereoPair(out leftImage, out rightImage);

      // PublishCameraImage(
      //     leftImage,
      //     timeMessage,
      //     new HeaderMsg(msgPublishCount, timeMessage, "cam0"),
      //     StereoCamLeftPublisherCmp.GetMessageTopic());

      // PublishCameraImage(
      //     rightImage,
      //     timeMessage,
      //     new HeaderMsg(msgPublishCount, timeMessage, "cam1"),
      //     StereoCamRightPublisherCmp.GetMessageTopic());

      // Publish the pose of the vehicle in the world (right-handed!).
      Transform T_world_imu = imu_rigidbody.transform;
      Vector3 t_world_imu;
      Quaternion q_world_imu;
      TransformUtils.ToRightHandedTransform(T_world_imu, out t_world_imu, out q_world_imu);
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

      // NOTE(milo): Need to correct the IMU timestamp (shift backwards one timestamp)?
      // long timestamp_shifted = data.timestamp - (long)(1e9*Time.fixedDeltaTime);

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

    while (true) {
      // ping_time = max_range / speed_of_sound = 100m / 343 m/s = 0.29 sec
      // Therefore can run at most 3ish Hz at 100m max range.
      yield return new WaitForSeconds(0.3f);
      RangeMeasurement data_A = this.range_sensor_A.Read();
      RangeMeasurement data_B = this.range_sensor_B.Read();

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
    string leftImageDataFolder = Path.Combine(leftImageFolder, "data");
    string rightImageDataFolder = Path.Combine(rightImageFolder, "data");

    Directory.CreateDirectory(leftImageFolder);
    Directory.CreateDirectory(rightImageFolder);
    Directory.CreateDirectory(leftImageDataFolder);
    Directory.CreateDirectory(rightImageDataFolder);

    int frame_id = 0;
    Texture2D leftImage, rightImage;

    // NOTE(milo): Maximum number of frames to save. Avoids using up all disk space by accident.
    while (frame_id < 5000) {
      // yield return new WaitForSeconds(1.0f / SimulationParams.CAMERA_PUBLISH_HZ);
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

      this.stereo_rig.CaptureStereoPair(out leftImage, out rightImage);

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
