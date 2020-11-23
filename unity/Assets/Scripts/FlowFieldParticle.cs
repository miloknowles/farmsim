using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class FlowFieldParticle : MonoBehaviour {
    // Speed at which the particle moves forward.
    public float _moveSpeed = 10.0f;

    void Update()
    {
        // Apply forward movement.
        this.transform.position += transform.forward * _moveSpeed * Time.deltaTime;
    }

    // Rotate the particle towards a vector.
    public void ApplyRotation(Vector3 rotation, float rotateSpeed)
    {
        Quaternion targetRotation = Quaternion.LookRotation(rotation.normalized);
        this.transform.rotation = Quaternion.RotateTowards(transform.rotation, targetRotation, rotateSpeed * Time.deltaTime);
    }
}
