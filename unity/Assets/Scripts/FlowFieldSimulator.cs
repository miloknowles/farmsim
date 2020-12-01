using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class FlowFieldSimulator : MonoBehaviour {

  // Library for generating random noise.
  FastNoise _fastNoise;
  public Vector3Int _gridSize = new Vector3Int(50, 50, 1);
  public float _cellSize = 1.0f;      // Size of each grid cell.
  public Vector3 _offset;             // Offset into the random noise field.
  public Vector3 _offsetSpeed;        // Speed at which each offset component changes.

  // Particle controls.
  public int _numberOfParticles = 100;
  public float _particleScale = 1.0f;
  public GameObject _particlePrefab;
  public float _particleMoveSpeed = 10.0f;
  public float _particleRotateSpeed = 100.0f;

  private List<FlowFieldParticle> _particles;

  void Start()
  {
    Initialize();
  }

  void Initialize()
  {
    _fastNoise = new FastNoise(); // Instantiate library.
    _particles = new List<FlowFieldParticle>();

    for (int i = 0; i < _numberOfParticles; ++i) {
      int attempt = 0;

      while (attempt < 10) {
        Vector3 randomXYZ = new Vector3(
            Random.Range(this.transform.position.x, this.transform.position.x + _gridSize.x * _cellSize),
            Random.Range(this.transform.position.y, this.transform.position.y + _gridSize.y * _cellSize),
            Random.Range(this.transform.position.z, this.transform.position.z + _gridSize.z * _cellSize)
        );

        // If location is not in collision, spawn the particle.
        if (IsValidSpawnLocation(randomXYZ)) {
          GameObject particleInstance = (GameObject)Instantiate(_particlePrefab);
          particleInstance.transform.position = randomXYZ;
          particleInstance.transform.parent = this.transform;
          particleInstance.transform.localScale = new Vector3(_particleScale, _particleScale, _particleScale);
          _particles.Add(particleInstance.GetComponent<FlowFieldParticle>());
          break;
        }

        ++attempt;
      }
    }

    Debug.Log("[NoiseFlowField] Instantiated particles:");
    Debug.Log(_particles.Count);
  }

  void Update()
  {
    CalculateFlowField();     // Update flow field.
    SimulateParticleMotion(); // Apply flow field.

    // Re-initialize if the desired number of particles has changed.
    if (this._particles.Count != this._numberOfParticles) {
      Debug.Log("[NoiseFlowField] Number of particles changed!");
      Initialize();
    }
  }

  // Is a particle not in collision with others?
  bool IsValidSpawnLocation(Vector3 position)
  {
    foreach (FlowFieldParticle p in _particles) {
      if (Vector3.Distance(position, p.transform.position) < (2.0f*this._particleScale)) {
        return false;
      }
    }
    return true;
  }

  // Change the randomized flow field by shifting the offset in it.
  void CalculateFlowField()
  {
    float ox = CircularWrap(_offset.x + (_offsetSpeed.x * Time.deltaTime), -1000, 1000);
    float oy = CircularWrap(_offset.y + (_offsetSpeed.y * Time.deltaTime), -1000, 1000);
    float oz = CircularWrap(_offset.z + (_offsetSpeed.z * Time.deltaTime), -1000, 1000);
    _offset = new Vector3(ox, oy, oz);
  }

  // Rotate the particle based on the nearby flow field direction.
  void SimulateParticleMotion()
  {
    foreach (FlowFieldParticle p in _particles) {
      float tx = CircularWrap(p.transform.position.x, this.transform.position.x, this.transform.position.x + (_gridSize.x * _cellSize));
      float ty = CircularWrap(p.transform.position.y, this.transform.position.y, this.transform.position.y + (_gridSize.y * _cellSize));
      float tz = CircularWrap(p.transform.position.z, this.transform.position.z, this.transform.position.z + (_gridSize.z * _cellSize));
      p.transform.position = new Vector3(tx, ty, tz);

      Vector3 positionInBox = p.transform.position - this.transform.position;

      p.ApplyRotation(CalculateFlowDirection(positionInBox), this._particleRotateSpeed);
      p._moveSpeed = _particleMoveSpeed;
      p.transform.localScale = new Vector3(_particleScale, _particleScale, _particleScale);
    }
  }

  Vector3 CalculateFlowDirection(Vector3 position)
  {
    // Shift noise to a [0, 1] range.
    float unitNoise = 0.5f * (1 + _fastNoise.GetSimplex(position.x + _offset.x, position.y + _offset.y, position.z + _offset.z));

    float cos = Mathf.Cos(2.0f*Mathf.PI * unitNoise);
    float sin = Mathf.Sin(2.0f*Mathf.PI * unitNoise);
    Vector3 flow = new Vector3(cos, sin, cos);

    return flow.normalized;
  }

  // Draw a wireframe box around the extend of the grid.
  private void OnDrawGizmos()
  {
    Gizmos.color = Color.white;

    Vector3 centerOfCube = this.transform.position + new Vector3(0.5f*_gridSize.x*_cellSize, 0.5f*_gridSize.y*_cellSize, 0.5f*_gridSize.z*_cellSize);
    Vector3 scaleOfCube = new Vector3(_gridSize.x * _cellSize, _gridSize.y * _cellSize, _gridSize.z * _cellSize);

    Gizmos.DrawWireCube(centerOfCube, scaleOfCube);
  }

  // If a value goes past a bound, set it to the other bound (i.e like the modulo operation).
  private float CircularWrap(float value, float minValue, float maxValue)
  {
    if (value >= minValue && value <= maxValue) {
      return value;
    }
    return (value > maxValue) ? minValue : maxValue;
  }
}
