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
  public GameObject axesObject;
  public bool showGhost = true;
  public bool showAxes = true;
  private LcmHandle lcmHandle;
  private Quaternion _world_q_body = Quaternion.identity;
  private Vector3 _world_t_body = Vector3.zero;
  private Quaternion _smoother_world_q_body = Quaternion.identity;
  private Vector3 _smoother_world_t_body = Vector3.zero;

  public int poseHistoryLength = 10;
  private Queue<GameObject> poseHistory = new Queue<GameObject>();
  private bool shouldAddNewPose = false;

  void Start()
  {
    this.lcmHandle = LcmHandle.Singleton;
    this.lcmHandle.Subscribe("vio/filter/world_P_body", this);
    this.lcmHandle.Subscribe("vio/smoother/world_P_body", this);
  }

  void Update()
  {
    if (this.showGhost) {
      this.ghostObject.transform.SetPositionAndRotation(this._world_t_body, this._world_q_body);
    }
    if (this.shouldAddNewPose && this.showAxes) {
      AddPose(ref this._world_q_body, ref this._world_t_body);
      this.shouldAddNewPose = false;
    }
  }

  void AddPose(ref Quaternion world_q_body, ref Vector3 world_t_body)
  {
    // Take the oldest pose and jump it to the front of the queue after setting to newest pose.
    if (this.poseHistory.Count >= this.poseHistoryLength) {
      GameObject oldest = this.poseHistory.Dequeue();
      oldest.transform.SetPositionAndRotation(world_t_body, world_q_body);
      this.poseHistory.Enqueue(oldest);
    } else {
      GameObject newPose = Instantiate(this.axesObject, world_t_body, world_q_body);
      this.poseHistory.Enqueue(newPose);
    }
  }

  public void MessageReceived(LcmHandle lcm, string channel, LcmStream dins)
  {
    if (channel == "vio/smoother/world_P_body") {
      pose3_stamped_t msg = new pose3_stamped_t(dins);
      LCMUtils.unpack_pose3_t(msg.pose, ref this._smoother_world_q_body, ref this._smoother_world_t_body);
      TransformUtils.ToLeftHandedTranslation(this._smoother_world_t_body, ref this._smoother_world_t_body);
      TransformUtils.ToLeftHandedQuaternion(this._smoother_world_q_body, ref this._smoother_world_q_body);
    } else if (channel == "vio/filter/world_P_body") {
      pose3_stamped_t msg = new pose3_stamped_t(dins);
      LCMUtils.unpack_pose3_t(msg.pose, ref this._world_q_body, ref this._world_t_body);
      TransformUtils.ToLeftHandedTranslation(this._world_t_body, ref this._world_t_body);
      TransformUtils.ToLeftHandedQuaternion(this._world_q_body, ref this._world_q_body);
      this.shouldAddNewPose = true; // NOTE(milo): Can't call AddPose in the callback - too much work?
    } else {
      Debug.Log("Received pose on unrecognized channel: " + channel);
    }
  }
}
