using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class MacroalgaeController : MonoBehaviour {
  [Range(0.0f, 1.0f)]
  public float maturity = 1.0f;       // Percentage of maximum size.
  private float _lastMaturity = 1.0f; // Used to detect changes.

  private int MACROALGAE_LAYER = 9;
  private List<GameObject> plants;

  private int interval = 10;

  void Start()
  {
    this.plants = FindGameObjectsInLayer(this.MACROALGAE_LAYER);
    this._lastMaturity = this.maturity;
  }

  void Update()
  {
    if (Time.frameCount % this.interval != 0) {
      return;
    }

    if (this.maturity != this._lastMaturity) {
      foreach (GameObject g in this.plants) {
        g.transform.localScale = new Vector3(this.maturity, this.maturity, this.maturity);
      }
    }
  }

  /**
   * Finds all objects of a given layer type (referred to by its index).
   */
  List<GameObject> FindGameObjectsInLayer(int layer)
  {
    GameObject[] allObjects = FindObjectsOfType<GameObject>();
    List<GameObject> objectsInLayer = new List<GameObject>();

    foreach (GameObject o in allObjects) {
      if (o.layer == layer) {
        objectsInLayer.Add(o);
      }
    }

    return objectsInLayer;
  }
}
