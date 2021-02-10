using ROSBridgeLib;
using ROSBridgeLib.sensor_msgs;
using System.Collections;
using SimpleJSON;
using UnityEngine;


public class PoseStampedPublisher : ROSBridgePublisher {
	public static new string GetMessageTopic()
	{
		return "/simulator/groundtruth/pose_default";
	}

	public static new string GetMessageType()
	{
		return "geometry_msgs/PoseStamped";
	}

	public static string ToYAMLString(ImuMessage msg)
	{
		return msg.ToYAMLString();
	}
}


public class ImuPosePublisher : PoseStampedPublisher {
	public static new string GetMessageTopic()
	{
		return "/sim/gt_pose/imu0";
	}
}


public class StereoCamLeftPosePublisher : PoseStampedPublisher {
	public static new string GetMessageTopic()
	{
		return "/sim/gt_pose/cam0";
	}
}


public class StereoCamRightPosePublisher : PoseStampedPublisher {
	public static new string GetMessageTopic()
	{
		return "/sim/gt_pose/cam1";
	}
}
