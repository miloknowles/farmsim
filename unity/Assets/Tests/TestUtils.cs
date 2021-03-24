using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

using Simulator;

namespace Tests {

public class TestUtils {
  [Test]
  public void TestStandardGaussian()
  {
    TestBoxMuller(0.0f, 1.0f);
  }

  [Test]
  public void TestGaussian2()
  {
    TestBoxMuller(3.14159f, 7.3f);
  }

  [Test]
  public void TestGaussian3()
  {
    TestBoxMuller(-0.1f, 0.05f);
  }

  public void TestBoxMuller(float mu, float sigma)
  {
    List<float> samples = new List<float>();

    for (int i = 0; i < 10000; ++i) {
      float s = Utils.Gaussian(mu, sigma);
      samples.Add(s);
    }

    float mean = Utils.Average(samples);
    Assert.IsTrue(Mathf.Abs(mean - mu) < 0.1f);
    Debug.Log("Empirical mean: " + mean.ToString());

    // Convert to a standard normal.
    List<float> sqerr = new List<float>();
    foreach (float s in samples) {
      sqerr.Add((s - mu) * (s - mu) / (sigma * sigma));
    }

    float var = Utils.Average(sqerr);
    Assert.IsTrue(Mathf.Abs(var - 1.0f) < 0.1f);
    Debug.Log("Empirical variance (should be 1): " + var.ToString());
  }
}

}
