using ROSBridgeLib;
using ROSBridgeLib.std_msgs;
using System.Collections;
using SimpleJSON;
using UnityEngine;


public class HeadingPublisher : ROSBridgePublisher {
  public static new string GetMessageTopic()
  {
    return "/imu/HeadingTrue_degree";
  }

  public static new string GetMessageType()
  {
    return "std_msgs/Float64";
  }

  public static string ToYAMLString(Float64Msg msg)
  {
    return msg.ToYAMLString();
  }
}
