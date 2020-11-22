using UnityEngine;
using System.Collections;

/**
 * A script that can be attached to cameras to simulate underwater visibility.
 */
public class UnderwaterEffect : MonoBehaviour {
  public float waterHeight = 0;
  public bool alwaysUnderwater = false;
  private bool isUnderwater;
  private Color normalColor = new Color(0.5f, 0.5f, 0.5f, 0.5f);
  private Color underwaterColor = new Color(0.22f, 0.65f, 0.77f, 0.5f);

  void Start()
  {
    if (alwaysUnderwater) {
      SetUnderwater();
    }
  }

  void Update()
  {
    if (alwaysUnderwater) {
      return;
    }
    if ((transform.position.y < waterHeight) != isUnderwater) {
      isUnderwater = transform.position.y < waterHeight;
      if (isUnderwater) {
        SetUnderwater();
      } else {
        SetNormal();
      }
    }
  }

  void SetNormal()
  {
    RenderSettings.fogColor = normalColor;
    RenderSettings.fogDensity = 0.01f;
  }

  void SetUnderwater()
  {
    RenderSettings.fogColor = underwaterColor;
    RenderSettings.fogDensity = 0.015f;
  }
}
