using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class FollowWaveHeight : MonoBehaviour {
  public Simulator.WaterWaves waveController;

  // To simulate some damping due to drag, the floating object only respond to some fraction of
  // wave height changes.
  public float dampingFactor = 0.2f;

  private Vector3 originalPosition;

  // Start is called before the first frame update
  void Start()
  {
    this.originalPosition = this.transform.position;
  }

  // Update is called once per frame
  void Update()
  {
    if (this.waveController != null) {
      Vector3 position = this.originalPosition;
      position.y += this.dampingFactor * this.waveController.CalculateHeightOffset(this.originalPosition.x, this.originalPosition.z, false);
      this.transform.position = position;
    }
  }
}
