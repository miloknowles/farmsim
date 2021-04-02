using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

using Simulator;

namespace Tests {


public class TestTransformUtil
{
  public void AssertQuaternionAlmostEqual(Quaternion a, Quaternion b)
  {
    Assert.IsTrue(Mathf.Abs(a.w - b.w) < 0.0001f);
    Assert.IsTrue(Mathf.Abs(a.x - b.x) < 0.0001f);
    Assert.IsTrue(Mathf.Abs(a.y - b.y) < 0.0001f);
    Assert.IsTrue(Mathf.Abs(a.z - b.z) < 0.0001f);
  }

  [Test]
  public void TestToRightHandedTranslation()
  {
    Vector3 t_lh_0 = new Vector3(1, 2, 3);
    Vector3 t_rh_0 = Vector3.zero;
    TransformUtils.ToRightHandedTranslation(t_lh_0, ref t_rh_0);
    Assert.AreEqual(t_rh_0, new Vector3(1, -2, 3));

    Vector3 t_lh_1 = new Vector3(1, -2, 3);
    Vector3 t_rh_1 = Vector3.zero;
    TransformUtils.ToRightHandedTranslation(t_lh_1, ref t_rh_1);
    Assert.AreEqual(t_rh_1, new Vector3(1, 2, 3));
  }

  [Test]
  public void TestToRightHandedQuaternion()
  {
    // https://www.andre-gaschler.com/rotationconverter/

    // IDENTITY (LOOK FORWARD): The identity orientation (x-right, z-forward) should be the same in both worlds.
    Quaternion q_identity_lh = new Quaternion(0f, 0f, 0f, 1f); // xyzw
    Quaternion q_identity_rh = Quaternion.identity;
    TransformUtils.ToRightHandedQuaternion(q_identity_lh, ref q_identity_rh);
    Assert.AreEqual(q_identity_rh, new Quaternion(0f, 0f, 0f, 1f));

    // LOOKING UP
    Quaternion q_lookup_lh = new Quaternion(0.7071068f, 0f, 0f, 0.7071068f);
    Quaternion q_lookup_rh = Quaternion.identity;
    TransformUtils.ToRightHandedQuaternion(q_lookup_lh, ref q_lookup_rh);
    Quaternion true_q_lookup_rh = new Quaternion(-0.7071068f, 0f, 0f, 0.7071068f);
    Assert.AreEqual(true_q_lookup_rh, q_lookup_rh);

    // LOOKING DOWN
    Quaternion q_lookdown_lh = new Quaternion(-0.7071068f, 0f, 0f, 0.7071068f);
    Quaternion q_lookdown_rh = Quaternion.identity;
    TransformUtils.ToRightHandedQuaternion(q_lookdown_lh, ref q_lookdown_rh);
    Quaternion true_q_lookdown_rh = new Quaternion(0.7071068f, 0f, 0f, 0.7071068f);
    Assert.AreEqual(true_q_lookdown_rh, q_lookdown_rh);

    // LOOKING LEFT
    Quaternion q_lookleft_lh = new Quaternion(0f, 0.7071068f, 0f, 0.7071068f);
    Quaternion q_lookleft_rh = Quaternion.identity;
    TransformUtils.ToRightHandedQuaternion(q_lookleft_lh, ref q_lookleft_rh);
    Quaternion true_q_lookleft_rh = new Quaternion(0f, 0.7071068f, 0f, 0.7071068f);
    Assert.AreEqual(true_q_lookleft_rh, q_lookleft_rh);

    // LOOKING RIGHT
    Quaternion q_lookr_lh = new Quaternion(0f, -0.7071068f, 0f, 0.7071068f);
    Quaternion q_lookr_rh = Quaternion.identity;
    TransformUtils.ToRightHandedQuaternion(q_lookr_lh, ref q_lookr_rh);
    Quaternion true_q_lookr_rh = new Quaternion(0f, -0.7071068f, 0f, 0.7071068f);
    Assert.AreEqual(true_q_lookr_rh, q_lookr_rh);

    // LOOKING BACK
    Quaternion q_lookb_lh = new Quaternion(0f, 1f, 0f, 0f);
    Quaternion q_lookb_rh = Quaternion.identity;
    TransformUtils.ToRightHandedQuaternion(q_lookb_lh, ref q_lookb_rh);
    Quaternion true_q_lookb_rh = new Quaternion(0f, 1f, 0f, 0f);
    AssertQuaternionAlmostEqual(true_q_lookb_rh, q_lookb_rh);
  }

  [Test]
  public void TestToRightHandedAngularVelocity()
  {
    Vector3 rh = Vector3.zero;
    TransformUtils.ToRightHandedAngularVelocity(new Vector3(1, 2, 3), ref rh);
    Assert.AreEqual(new Vector3(-1, 2, -3), rh);
  }
}

}
