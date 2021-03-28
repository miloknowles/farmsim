using System;
using System.Linq;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using ROSBridgeLib;
using ROSBridgeLib.CustomMessages;


using ROSCallback = System.Action<ROSBridgeMsg>;


// NOTE(milo): The attached GameObject must be the LAST in the scene hierarchy so that its Start()
// is called last. This ensures that all Publishers/Subscribers are added to the ROSBridgeWebSocket
// before it advertises them.
public class ROSMessageHolder : MonoBehaviour {
  private Dictionary<string, List<ROSCallback>> _callbacks = new Dictionary<string, List<ROSCallback>>();
  private Dictionary<string, ROSBridgeMsg> _latest = new Dictionary<string, ROSBridgeMsg>();

  // Central ROSBridge. All other scripts should use this one for publishing/subscribing.
  public ROSBridgeWebSocketConnection ros = null;

  // Awake is called before Start(). This ensures that ROS is ready before other scripts start up.
  void Awake()
  {
    this.ros = new ROSBridgeWebSocketConnection("ws://localhost", SimulationParams.ROS_BRIDGE_PORT);
  }

  void Start()
  {
    this.ros.Connect();
  }

  void OnApplicationQuit()
  {
    if (this.ros != null) {
      this.ros.Disconnect();
    }
  }

  /**
   * Tell the ROSMessageHolder to keep a copy of the latest message that arrives on a topic.
   */
  public void RequireStorage(string topic, bool shouldStore)
  {
    if (shouldStore) {
      if (!this._latest.ContainsKey(topic)) {
        this._latest.Add(topic, null);
      }
    } else {
      this._latest.Remove(topic);
    }
  }

  /**
   * Update the latest message on a topic. Downstream callbacks that are attached to this topic
   * will be called.
   */
  public void UpdateTopic(string topic, ROSBridgeMsg msg)
  {
    Debug.Log("updating topic: " + topic);
    if (this._callbacks.ContainsKey(topic)) {
      foreach (ROSCallback callback in this._callbacks[topic]) {
        callback(msg);
      }
    }
  }

  /**
   * Register a callback for a given topic. Every time a new message arrives on that topic, the
   * callback will be called.
   */
  public void RegisterCallback(string topic, ROSCallback callback)
  {
    if (!this._callbacks.ContainsKey(topic)) {
      this._callbacks.Add(topic, new List<ROSCallback>());
    }
    this._callbacks[topic].Add(callback);
  }
}
