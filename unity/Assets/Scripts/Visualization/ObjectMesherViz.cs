using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using LCM;
using vehicle;
using Simulator;

// Avoid LCM's comically deep namespace.
using LcmHandle = LCM.LCM.LCM;
using LcmStream = LCM.LCM.LCMDataInputStream;
using LcmSubscriber = LCM.LCM.LCMSubscriber;


public class ObjectMesherViz : MonoBehaviour, LcmSubscriber
{
  private Mesh mesh;
  private LcmHandle lcmHandle;
  private string meshChannel = "object_mesher/mesh";
  private Vector3[] newVertices;
  private int[] newTriangles;
  private bool meshWasUpdated = false;

  // Start is called before the first frame update
  void Start()
  {
    this.mesh = this.GetComponent<MeshFilter>().mesh;

    this.lcmHandle = LcmHandle.Singleton;
    this.lcmHandle.Subscribe(this.meshChannel, this);
    Debug.Log($"[ ObjectMesherViz ] Listening on: {this.meshChannel}");
  }

  void Update()
  {
    // Don't do unnecessary mesh rebuilding.
    if (!this.meshWasUpdated) {
      return;
    }
    this.meshWasUpdated = false;
    this.mesh.Clear();
    this.mesh.vertices = this.newVertices;
    this.mesh.triangles = this.newTriangles;
  }

  // https://docs.unity3d.com/ScriptReference/Mesh.html
  public void MessageReceived(LcmHandle lcm, string channel, LcmStream dins)
  {
    if (channel == this.meshChannel) {
      mesh_stamped_t msg = new mesh_stamped_t(dins);

      Array.Resize(ref this.newVertices, msg.mesh.vertices.Length);
      Array.Resize(ref this.newTriangles, 3*msg.mesh.triangles.Length);

      for (int i = 0; i < msg.mesh.vertices.Length; ++i) {
        LCMUtils.unpack_vector3_t(msg.mesh.vertices[i], ref this.newVertices[i]);
        TransformUtils.ToLeftHandedTranslation(this.newVertices[i], ref this.newVertices[i]);
      }

      for (int i = 0; i < msg.mesh.triangles.Length; ++i) {
        this.newTriangles[3*i] = msg.mesh.triangles[i].vertex_indices[0];
        this.newTriangles[3*i + 1] = msg.mesh.triangles[i].vertex_indices[1];
        this.newTriangles[3*i + 2] = msg.mesh.triangles[i].vertex_indices[2];
      }
      this.meshWasUpdated = true;

    } else {
      Debug.Log("Received mesh on unrecognized channel: " + channel);
    }
  }
}
