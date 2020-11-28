using System;
using UnityEngine;
using System.Collections;
using ROSBridgeLib;
using ROSBridgeLib.CustomMessages;


public class ROSMessageHolder : MonoBehaviour {
  private TridentThrustMsg _tridentThrustMsg = null;

  public void UpdateTridentThrustMsg(TridentThrustMsg msg) {
    _tridentThrustMsg = msg;
    Debug.Log("updated");
  }
  public TridentThrustMsg GetTridentThrustMsg() { return _tridentThrustMsg; }
}
