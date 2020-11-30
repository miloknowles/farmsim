using ROSBridgeLib;
using ROSBridgeLib.sensor_msgs;
using System.Collections;
using SimpleJSON;
using UnityEngine;


public class ImagePublisher : ROSBridgePublisher {
	public static string GetMessageTopic()
	{
		return "/image";
	}

	public static string GetMessageType()
	{
		return "sensor_msgs/Image";
	}

	public static string ToYAMLString(ImageMsg msg)
	{
		return msg.ToYAMLString();
	}
}


// public class CameraForwardLeftPublisher : ImagePublisher {
// 	public static string GetMessageTopic()
// 	{
// 		return "/simulator/sensors/camera_fl";
// 	}
// }


// public class CameraForwardRightPublisher : ImagePublisher {
// 	public static string GetMessageTopic()
// 	{
// 		return "/simulator/sensors/camera_fr";
// 	}
// }


// public class CameraDownwardLeftPublisher : ImagePublisher {
// 	public static string GetMessageTopic()
// 	{
// 		return "/simulator/sensors/camera_dl";
// 	}
// }


// public class CameraUpwardLeftPublisher : ImagePublisher {
// 	public static string GetMessageTopic()
// 	{
// 		return "/simulator/sensors/camera_ul";
// 	}
// }
