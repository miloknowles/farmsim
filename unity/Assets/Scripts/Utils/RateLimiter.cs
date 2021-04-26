using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Simulator {

// Useful for limiting the rate of a loop to a desired hz. This will take into
// account the execution time of the loop itself, and continuously adjust the
// wait time to achieve the nominal hz.
public class RateLimiter {
  private float nominalInterval;  // How much time between loop executions.
  private float ticTime = 0.0f;      // Last loop start time.
  public float avgLoopTime = 0.0f;  // Moving avg of the loop time.
  private float alpha = 0.6f;  // Controls the moving avg smoothing (high = more smooth).

  public RateLimiter(float nominalHz)
  {
    this.nominalInterval = 1.0f / nominalHz;
  }

  // Mark the start of a loop iteration.
  public void Tic()
  {
    this.ticTime = (float)Timestamp.UnitySeconds();
  }

  // Mark the end of a loop iteration.
  public void Toc()
  {
    float thisLoopTime = (float)Timestamp.UnitySeconds() - this.ticTime;

    if (this.avgLoopTime <= 0.0f) {
      this.avgLoopTime = thisLoopTime;  // Handle first measurement.
    } else {
      this.avgLoopTime = this.alpha*this.avgLoopTime + (1.0f - this.alpha)*(thisLoopTime);
    }
  }

  // Return the amount of time to wait before another loop execution.
  public float WaitTime()
  {
    // Wait at least a tiny amount of time.
    return Mathf.Max(0.01f, this.nominalInterval - this.avgLoopTime);
  }
}

}
