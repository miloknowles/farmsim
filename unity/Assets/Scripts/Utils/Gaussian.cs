using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Simulator {

public class Gaussian {
  // Generate a sample from a Gaussian distribution using the Box-Muller transform.
  // https://en.wikipedia.org/wiki/Box%E2%80%93Muller_transform
  // http://keyonvafa.com/box-muller-transform/
  public static float Sample1D(float mu, float sigma)
  {
    float U1 = Random.Range(0.0f, 1.0f);
    float U2 = Random.Range(0.0f, 1.0f);
    float X = Mathf.Sqrt(-2.0f * Mathf.Log(U1)) * Mathf.Sin(2.0f * Mathf.PI * U2);

    return mu + sigma*X;
  }

  // Sample a vector of three i.i.d Gaussian variables with mean and variance given mu and sigma.
  // Slightly more efficient than using Sample1D three times because this gets (2) random values
  // out of the first Box-Muller transform.
  // NOTE(milo): Could easily extend to Sample4D with the extra X4 below.
  public static Vector3 Sample3D(Vector3 mu, Vector3 sigma)
  {
    float U1 = Random.Range(0.0f, 1.0f);
    float U2 = Random.Range(0.0f, 1.0f);
    float X1 = Mathf.Sqrt(-2.0f * Mathf.Log(U1)) * Mathf.Cos(2.0f * Mathf.PI * U2);
    float X2 = Mathf.Sqrt(-2.0f * Mathf.Log(U1)) * Mathf.Sin(2.0f * Mathf.PI * U2);

    float U3 = Random.Range(0.0f, 1.0f);
    float U4 = Random.Range(0.0f, 1.0f);
    float X3 = Mathf.Sqrt(-2.0f * Mathf.Log(U3)) * Mathf.Cos(2.0f * Mathf.PI * U4);
    // float X4 = Mathf.Sqrt(-2.0f * Mathf.Log(U1)) * Mathf.Sin(2.0f * Mathf.PI * U2);

    Vector3 X = new Vector3(X1, X2, X3);
    return mu + Vector3.Scale(X, sigma); // "Scale" is componentwise multiplication.
  }
}

}
