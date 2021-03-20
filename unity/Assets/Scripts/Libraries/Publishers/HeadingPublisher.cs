using ROSBridgeLib;
using ROSBridgeLib.std_msgs;
using System.Collections;
using SimpleJSON;
using UnityEngine;


public class HeadingPublisher : ROSBridgePublisher {
  public static new string GetMessageTopic()
  {
    return "/sim/heading0";
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


public class GroundtruthHeadingPublisher : HeadingPublisher {
  public static new string GetMessageTopic()
  {
    return "/sim/gt_pose/heading0";
  }
}
