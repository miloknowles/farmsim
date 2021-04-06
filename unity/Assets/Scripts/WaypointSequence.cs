using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WaypointSequence : MonoBehaviour
{
  private int waypointIndex = 0;
  private List<GameObject> waypoints = new List<GameObject>();

  private void GetAttachedWaypoints()
  {
    // Collect all attached waypoints.
    foreach (Transform child in this.transform) {
      this.waypoints.Add(child.gameObject);
      child.gameObject.SetActive(false);
    }
    this.waypoints[0].SetActive(true);
    Debug.Log($"Found {this.waypoints.Count} attached waypoints");
  }

  public GameObject GetNextWaypoint()
  {
    // Hide the previous waypoint, get next, unhide it.
    this.waypoints[this.waypointIndex].SetActive(false);
    this.waypointIndex = (this.waypointIndex + 1) % waypoints.Count;
    this.waypoints[this.waypointIndex].SetActive(true);

    return this.waypoints[this.waypointIndex];
  }

  public GameObject GetCurrentWaypoint()
  {
    if (waypoints.Count == 0) {
      this.GetAttachedWaypoints();
    }
    return this.waypoints[this.waypointIndex];
  }
}
