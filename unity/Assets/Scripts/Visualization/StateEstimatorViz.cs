using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using LCM;
using vehicle;
using Simulator;

// Avoid LCM's comically deep namespace.
using LcmHandle = LCM.LCM.LCM;
using LcmStream = LCM.LCM.LCMDataInputStream;
using LcmSubscriber = LCM.LCM.LCMSubscriber;

public class StateEstimatorViz : MonoBehaviour, LcmSubscriber
{
  public GameObject ghostObject;
  private LcmHandle lcmHandle;
  private Quaternion _world_q_body = Quaternion.identity;
  private Vector3 _world_t_body = Vector3.zero;

  void Start()
  {
    this.lcmHandle = LcmHandle.Singleton;
    this.lcmHandle.Subscribe("vio/filter/world_P_body", this);
    Debug.Log("Subscribed to VIO channels");
  }

  void Update()
  {
    this.ghostObject.transform.SetPositionAndRotation(this._world_t_body, this._world_q_body);
  }

  public void MessageReceived(LcmHandle lcm, string channel, LcmStream dins)
  {
    if (channel == "vio/smoother/world_P_body") {
    } else if (channel == "vio/filter/world_P_body") {
      pose3_stamped_t msg = new pose3_stamped_t(dins);
      LCMUtils.unpack_pose3_t(msg.pose, ref this._world_q_body, ref this._world_t_body);
      TransformUtils.ToLeftHandedTranslation(this._world_t_body, ref this._world_t_body);
      TransformUtils.ToLeftHandedQuaternion(this._world_q_body, ref this._world_q_body);
    } else {
      Debug.Log("Received pose on unrecognized channel: " + channel);
    }
  }
}
