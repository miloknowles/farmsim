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
  void Start()
  {
    MeshFilter[] meshFiltersAll = GetComponentsInChildren<MeshFilter>();

    // Get all relevant meshes (non-empty and of the target material).
    List<MeshFilter> meshFiltersRelevant = new List<MeshFilter>();

    for (int i = 0; i < meshFiltersAll.Length; ++i) {
      if (meshFiltersAll[i].sharedMesh != null) {
        meshFiltersRelevant.Add(meshFiltersAll[i]);
      }
    }

    CombineInstance[] combine = new CombineInstance[meshFiltersRelevant.Count];

    // Add sub-meshes to the combined mesh.
    for (int i = 0; i < combine.Length; ++i) {
      if (!meshFiltersRelevant[i].sharedMesh.isReadable) {
        Debug.Log(meshFiltersRelevant[i].gameObject);
      }
      combine[i].mesh = meshFiltersRelevant[i].sharedMesh;
      combine[i].transform = meshFiltersRelevant[i].transform.localToWorldMatrix;
      meshFiltersRelevant[i].gameObject.SetActive(false);
    }

    // int i = 0;
    // while (i < meshFilters.Length) {
    //   if (meshFilters[i].sharedMesh != null) {
    //     combine[i].mesh = meshFilters[i].sharedMesh;
    //     combine[i].transform = meshFilters[i].transform.localToWorldMatrix;
    //     meshFilters[i].gameObject.SetActive(false);
    //     i++;
    //   }
    // }

    this.transform.GetComponent<MeshFilter>().mesh = new Mesh();
    this.transform.GetComponent<MeshFilter>().mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
    this.transform.GetComponent<MeshFilter>().mesh.CombineMeshes(combine);
    this.transform.gameObject.SetActive(true);
  }
}
