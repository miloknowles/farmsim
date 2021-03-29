using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

// https://docs.unity3d.com/ScriptReference/Mesh.CombineMeshes.html
// Copy meshes from children into the parent's Mesh.
// CombineInstance stores the list of meshes.  These are combined
// and assigned to the attached Mesh.
[RequireComponent(typeof(MeshFilter))]
[RequireComponent(typeof(MeshRenderer))]
public class MeshCombiner : MonoBehaviour {
  // If specified (nonnegative), only children with this layer index will be included.
  public int restrictToLayer = -1;

  void Start()
  {
    MeshFilter[] meshFiltersAll = GetComponentsInChildren<MeshFilter>();

    // Get all relevant meshes (non-empty).
    List<MeshFilter> meshFiltersRelevant = new List<MeshFilter>();

    for (int i = 0; i < meshFiltersAll.Length; ++i) {
      int layer = meshFiltersAll[i].gameObject.layer;
      bool valid = meshFiltersAll[i].sharedMesh != null;

      if (valid && (this.restrictToLayer < 0 || (this.restrictToLayer == layer))) {
        meshFiltersRelevant.Add(meshFiltersAll[i]);
      }
    }

    // Debug.Log("Combining " + meshFiltersRelevant.Count.ToString() + " sub-meshes");

    CombineInstance[] combine = new CombineInstance[meshFiltersRelevant.Count];

    Matrix4x4 thisLocalToWorld = this.gameObject.transform.localToWorldMatrix;

    // Add sub-meshes to the combined mesh.
    for (int i = 0; i < combine.Length; ++i) {
      if (!meshFiltersRelevant[i].sharedMesh.isReadable) {
        Debug.Log("WARNING: Encountered static object. Combining mesh won't work.");
        Debug.Log(meshFiltersRelevant[i].gameObject);
      }
      combine[i].mesh = meshFiltersRelevant[i].sharedMesh;
      combine[i].transform = thisLocalToWorld.inverse * meshFiltersRelevant[i].transform.localToWorldMatrix;
      meshFiltersRelevant[i].gameObject.SetActive(false);
    }

    this.transform.GetComponent<MeshFilter>().mesh = new Mesh();
    this.transform.GetComponent<MeshFilter>().mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
    this.transform.GetComponent<MeshFilter>().mesh.CombineMeshes(combine, true, true, false);
    this.transform.gameObject.SetActive(true);
  }
}
