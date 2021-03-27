using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Simulator;


public class FishController : MonoBehaviour {
  public int numberOfFish = 20;
  public float fishSwimSpeedMin = 0.1f;
  public float fishSwimSpeedMax = 5.0f;
  public float fishRotateSpeed = 30.0f;
  public Vector3 aquariumSize = new Vector3(20, 20, 20);

  public GameObject fishPrefab;
  private List<GameObject> fishInstances;
  private List<float> fishSpeeds;
  private List<Vector3> fishDirections;
  private float lastFishUpdateTime;
  private float fishUpdateInterval = 0.1f;
  private float minAzimuth = -20.0f;
  private float maxAzimuth = 20.0f;

  private List<Color> fishColors = new List<Color>{
    Color.red, Color.yellow, Color.blue, Color.green, Color.cyan, Color.magenta
  };

  // Start is called before the first frame update
  void Start()
  {
    fishInstances = new List<GameObject>();
    fishSpeeds = new List<float>();
    fishDirections = new List<Vector3>();

    // Instantiate all of the fish at random positions and orientations.
    for (int i = 0; i < this.numberOfFish; ++i) {
      Vector3 randomXYZ = new Vector3(
          Random.Range(this.transform.position.x, this.transform.position.x + this.aquariumSize.x),
          Random.Range(this.transform.position.y, this.transform.position.y + this.aquariumSize.y),
          Random.Range(this.transform.position.z, this.transform.position.z + this.aquariumSize.z)
      );

      Vector3 randomDir = TransformUtils.SampleDirectionShallowAzimuth(this.minAzimuth, this.maxAzimuth);

      GameObject fish = (GameObject)Instantiate(fishPrefab);
      fish.transform.position = randomXYZ;
      fish.transform.parent = this.transform;
      Color color = this.fishColors[i % this.fishColors.Count];
      fish.GetComponent<MeshRenderer>().material.color = color;

      fishInstances.Add(fish);
      fishSpeeds.Add(Random.Range(this.fishSwimSpeedMin, this.fishSwimSpeedMax));
      fishDirections.Add(randomDir);
    }

    lastFishUpdateTime = Time.time;
  }

  // Update is called once per frame
  void Update()
  {
    // Teleport any fish that exit the aquarium.
    foreach (GameObject f in this.fishInstances) {
      float tx = Utils.CircularWrap(f.transform.position.x, this.transform.position.x, this.transform.position.x + this.aquariumSize.x);
      float ty = Utils.CircularWrap(f.transform.position.y, this.transform.position.y, this.transform.position.y + this.aquariumSize.y);
      float tz = Utils.CircularWrap(f.transform.position.z, this.transform.position.z, this.transform.position.z + this.aquariumSize.z);
      f.transform.position = new Vector3(tx, ty, tz);
    }

    // Change the direction and speed of one fish.
    if ((Time.time - this.lastFishUpdateTime) > this.fishUpdateInterval) {
      this.lastFishUpdateTime = Time.time;
      int randomIndex = Random.Range(0, this.fishInstances.Count);
      this.fishDirections[randomIndex] = TransformUtils.SampleDirectionShallowAzimuth(this.minAzimuth, this.maxAzimuth);
      this.fishSpeeds[randomIndex] = Random.Range(this.fishSwimSpeedMin, this.fishSwimSpeedMax);
    }

    // Simulate the forward motion of the fish.
    for (int i = 0; i < this.fishInstances.Count; ++i) {
      GameObject fish = this.fishInstances[i];
      Vector3 direction = this.fishDirections[i];
      TransformUtils.ApplyRotation(fish, direction, this.fishRotateSpeed);
      fish.transform.position += fish.transform.forward * this.fishSpeeds[i] * Time.deltaTime;
    }
  }

  // Draw a wireframe box around the extend of the aquarium.
  private void OnDrawGizmos()
  {
    Gizmos.color = Color.yellow;
    Vector3 centerOfCube = this.transform.position + 0.5f * this.aquariumSize;
    Gizmos.DrawWireCube(centerOfCube, this.aquariumSize);
  }
}
