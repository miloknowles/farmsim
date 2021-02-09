using ROSBridgeLib;
using ROSBridgeLib.sensor_msgs;
using System.Collections;
using SimpleJSON;
using UnityEngine;


public class CompressedImagePublisher : ROSBridgePublisher {
	public static new string GetMessageTopic()
	{
		return "/image/compressed";
	}

	public static new string GetMessageType()
	{
		return "sensor_msgs/CompressedImage";
	}

	public static string ToYAMLString(CompressedImageMsg msg)
	{
		return msg.ToYAMLString();
	}
}


public class CameraForwardLeftPublisher : CompressedImagePublisher {
	public static new string GetMessageTopic()
	{
		return "/simulator/sensors/cam0/compressed";
	}
}


public class CameraForwardRightPublisher : CompressedImagePublisher {
	public static new string GetMessageTopic()
	{
		return "/simulator/sensors/cam1/compressed";
	}
}


public class CameraDownwardLeftPublisher : CompressedImagePublisher {
	public static new string GetMessageTopic()
	{
		return "/simulator/sensors/camera_dl/compressed";
	}
}


public class CameraUpwardLeftPublisher : CompressedImagePublisher {
	public static new string GetMessageTopic()
	{
		return "/simulator/sensors/camera_ul/compressed";
	}
}
