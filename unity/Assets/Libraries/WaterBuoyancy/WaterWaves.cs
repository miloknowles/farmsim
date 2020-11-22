using UnityEngine;

/**
 * Uses Perlin noise to simulate moving surface waves.
 # From: https://github.com/dbrizov/NaughtyWaterBuoyancy/blob/master/Assets/NaughtyWaterBuoyancy/Scripts/Core/WaterWaves.cs
 */
namespace Simulator {
  public class WaterWaves : MonoBehaviour {
    [SerializeField]
    private float speed = 1f;

    [SerializeField]
    private float height = 0.2f;

    [SerializeField]
    private float noiseWalk = 0.5f;

    [SerializeField]
    private float noiseStrength = 0.1f;

    private Mesh mesh;
    private Vector3[] originalVertices;
    private Vector3[] vertices;

    protected virtual void Awake()
    {
      this.mesh = this.GetComponent<MeshFilter>().mesh;
      this.originalVertices = this.mesh.vertices;
      this.vertices = new Vector3[this.originalVertices.Length];
    }

    protected virtual void Start()
    {
      this.ResizeBoxCollider();
    }

    protected virtual void Update()
    {
      for (int i = 0; i < this.vertices.Length; ++i) {
        Vector3 vertex = this.originalVertices[i];

        // Divide by localScale to make the wave height in global scale.
        float amplitude = this.height / this.transform.localScale.y;
        vertex.y += amplitude * CalculateHeightOffset(this.originalVertices[i].x, this.originalVertices[i].z, true);
        this.vertices[i] = vertex;
      }

      this.mesh.vertices = this.vertices;
      this.mesh.RecalculateNormals();
    }

    private void ResizeBoxCollider()
    {
      var boxCollider = this.GetComponent<BoxCollider>();
      if (boxCollider != null)
      {
        Vector3 center = boxCollider.center;
        center.y = boxCollider.size.y / -2f;
        center.y += (this.height + this.noiseStrength) / this.transform.localScale.y;

        boxCollider.center = center;
      }
    }

    /**
     * Returns the wave height offset (above the flat water level) at a particular XZ location.
     */
    public float CalculateHeightOffset(float x, float z, bool noise)
    {
      // All vertices parameterized by (x + z = constant) will have the same wave height.
      // This orients all of the waves diagonally at a 45 deg angle.
      float offset = Mathf.Sin(Time.time * this.speed + x + z);

      // Apply Perlin noise to make the waves look more realistic than a pure sinewave.
      if (noise) {
        offset += Mathf.PerlinNoise(x + this.noiseWalk, 0) * this.noiseStrength;
      }

      return offset;
    }
  }
}
