using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class FollowWaveHeight : MonoBehaviour {
  public Simulator.WaterWaves waveController;

  // To simulate some damping due to drag, the floating object only respond to some fraction of
  // wave height changes.
  public float dampingFactor = 0.2f;

  // NOTE(milo): Any other script that wants to change the position of the attached object should
  // change nominalPosition rather than this.transform.position, since this script's Update() method
  // will override.
  public Vector3 nominalPosition;

  // Start is called before the first frame update
  void Start()
  {
    this.nominalPosition = this.transform.position;
  }

  // Update is called once per frame
  void Update()
  {
    if (this.waveController != null) {
      Vector3 position = this.nominalPosition;
      position.y += this.dampingFactor * this.waveController.CalculateHeightOffset(this.nominalPosition.x, this.nominalPosition.z, false);
      this.transform.position = position;
    }
  }
}
