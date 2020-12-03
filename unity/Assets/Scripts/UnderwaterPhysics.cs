using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Simulator;


public enum CurrentMode
{
  NONE = 0,
  PRESET_DIRECTION_AND_SPEED = 1,
  CONSTANT_RANDOM_DIRECTION_AND_SPEED = 2,
  DYNAMIC_RANDOM_DIRECTION_AND_SPEED = 3
}


public class UnderwaterPhysics : MonoBehaviour {
  // The body on which underwater forces should act.
  public Rigidbody body;
  public MeshFilter meshFilter;

  // If true, apply gravity force based on mass and buoyant force based on mesh volume.
  public bool simulateBuoyancy = false;

  public CurrentMode currentMode = CurrentMode.NONE;

  public float currentSpeedMin = 0.0f;
  public float currentSpeedMax = 1.0f;
  public float currentSpeed = 0.2f;
  public Vector3 currentDirection = new Vector3(1, 0, 0); // m/s

  [Range(0.01f, 0.5f)]
  public float dragCoefficient = 0.2f;

  // We assume that this body is always underwater, so buoyant forces don't change.
  private Vector3 constantBuoyantForce; // N
  private Vector3 constantGravityForce; // N

  private float lastCurrentUpdateTime;
  private float nextCurrentUpdateInterval = 10.0f;

  void Start()
  {
    Mesh mesh = meshFilter.sharedMesh;
    float volume = ComputeMeshVolume(mesh);
    Debug.Log($"[UnderwaterPhysics] The volume of the mesh is {volume} m^3.");

    this.constantBuoyantForce = -1.0f * volume * SimulationController.WATER_DENSITY_KG_M3 * Physics.gravity;
    this.constantGravityForce = Physics.gravity * body.mass;
    Debug.Log($"[UnderwaterPhysics] Gravity={this.constantGravityForce} N | Buoyancy={this.constantBuoyantForce} N");

    // Sample a current direction.
    if (this.currentMode == CurrentMode.CONSTANT_RANDOM_DIRECTION_AND_SPEED ||
        this.currentMode == CurrentMode.DYNAMIC_RANDOM_DIRECTION_AND_SPEED) {
      this.currentDirection = Utils.SampleDirectionShallowAzimuth(-5, 5);
      this.currentSpeed = Random.Range(this.currentSpeedMin, this.currentSpeedMax);
    } else {
      this.currentDirection = Vector3.Normalize(this.currentDirection);
    }

    Debug.Log($"[UnderwaterPhysics] currentSpeed={this.currentSpeed}");
    Debug.Log(this.currentDirection);

    this.lastCurrentUpdateTime = Time.time;
  }

  /**
   * Simulates all of external forces acting on this underwater body: gravity, buoyancy, and
   * (eventually) water currents.
   */
  void FixedUpdate()
  {
    if (this.simulateBuoyancy) {
      body.AddForce(this.constantGravityForce + this.constantBuoyantForce);
    }

    // Maybe update the current direction.
    if (this.currentMode == CurrentMode.DYNAMIC_RANDOM_DIRECTION_AND_SPEED) {
      bool timeForUpdate = (Time.time - this.lastCurrentUpdateTime) > this.nextCurrentUpdateInterval;

      if (timeForUpdate) {
        Debug.Log("[UnderwaterPhysics] Updated random current");
        this.currentDirection = Utils.SampleDirectionShallowAzimuth(-5, 5);
        this.currentSpeed = Random.Range(this.currentSpeedMin, this.currentSpeedMax);
        this.lastCurrentUpdateTime = Time.time;
        this.nextCurrentUpdateInterval = Random.Range(5.0f, 20.0f);
      }
    }

    if (this.currentMode != CurrentMode.NONE) {
      Vector3 flowVel = this.GetComponent<Rigidbody>().velocity - (this.currentSpeed * this.currentDirection);
      // F = C * rho * V^2
      Vector3 F = this.dragCoefficient * SimulationController.WATER_DENSITY_KG_M3 * Vector3.Scale(flowVel, flowVel);

      // To avoid drag forces blowing up at high velocity, clamp magnitude.
      body.AddForce(Vector3.ClampMagnitude(F, 100.0f));
    }

    // Make sure the vehicle can't escape the water.
    if (this.transform.position.y > 0.2f) {
      this.transform.position = new Vector3(this.transform.position.x, 0.2f, this.transform.position.z);
    }
  }

  //============================================================================
  public static float SignedVolumeOfTriangle(Vector3 p1, Vector3 p2, Vector3 p3)
  {
    float v321 = p3.x * p2.y * p1.z;
    float v231 = p2.x * p3.y * p1.z;
    float v312 = p3.x * p1.y * p2.z;
    float v132 = p1.x * p3.y * p2.z;
    float v213 = p2.x * p1.y * p3.z;
    float v123 = p1.x * p2.y * p3.z;
    return (1.0f / 6.0f) * (-v321 + v231 + v312 - v132 - v213 + v123);
  }

  public static float ComputeMeshVolume(Mesh mesh)
  {
    float volume = 0;
    Vector3[] vertices = mesh.vertices;
    int[] triangles = mesh.triangles;
    for (int i = 0; i < mesh.triangles.Length; i += 3) {
      Vector3 p1 = vertices[triangles[i + 0]];
      Vector3 p2 = vertices[triangles[i + 1]];
      Vector3 p3 = vertices[triangles[i + 2]];
      volume += SignedVolumeOfTriangle(p1, p2, p3);
    }
    return Mathf.Abs(volume);
  }
}
