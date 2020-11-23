using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public enum CornerCode
{
  BOTTOMLEFT = 0,
  TOPLEFT = 1,
  TOPRIGHT = 2,
  BOTTOMRIGHT = 3
}


public class FarmController : MonoBehaviour {
  public int maxRow = 1;
  public char maxBuoy = 'D';

  public float minDepth = -1;  // Max y-value.
  public float maxDepth = -19; // Min y-value.
  public float nominalDepth = -10;
  public float submergeDepth =-19;

  private int selectedRow = 0;
  private char selectedBuoy = 'A';

  public bool winchInProgress = false;

  // Start is called before the first frame update
  void Start()
  {
    Highlight(true);
  }

  // Update is called once per frame
  void Update()
  {
    // Toggle depth between nominal and submerged.
		if (Input.GetKeyUp("right shift")) {
      float currentY = GetWinchesAtAddress(this.selectedRow, this.selectedBuoy)[0].transform.position.y;
      float nextY = (currentY > this.submergeDepth) ? this.submergeDepth : this.nominalDepth;
      bool success = SetDepth(this.selectedRow, this.selectedBuoy, nextY, true);
		}

    // Switch the buoy that is currently selected with the <> keys.
    if (Input.GetKeyUp(",")) {
      Highlight(false);
      this.selectedBuoy = PrevLetterWrap(this.selectedBuoy);
      if (this.selectedBuoy == this.maxBuoy) {
        this.selectedRow = (this.selectedRow - 1) % (this.maxRow + 1);
      }
      Highlight(true);
    } else if (Input.GetKeyUp(".")) {
      Highlight(false);
      this.selectedBuoy = NextLetterWrap(this.selectedBuoy);
      if (this.selectedBuoy == 'A') {
        this.selectedRow = (this.selectedRow + 1) % (this.maxRow + 1);
      }
      Highlight(true);
    }
  }

  /**
   * Toggles the depth of a particular buoy between a submerged/nominal position.
   */
  public void ToggleDepth(int row, char buoy)
  {
    float currentY = GetWinchesAtAddress(row, buoy)[0].transform.position.y;
    float nextY = (currentY > this.submergeDepth) ? this.submergeDepth : this.nominalDepth;
    SetDepth(row, buoy, nextY, true);
  }

  void Highlight(bool on)
  {
    List<GameObject> selectedWinches = GetWinchesAtAddress(this.selectedRow, this.selectedBuoy);
    foreach (GameObject w in selectedWinches) {
      MeshRenderer mr = w.GetComponent<MeshRenderer>();
      if (mr != null) {
        mr.material.color = on ? Color.red : Color.gray;
      }
    }
    Debug.Log("Highlighted address:");
    Debug.Log($"{this.selectedBuoy}{this.selectedRow}");
  }

  public IEnumerator AnimateMotion(List<GameObject> objects, Vector3 start, Vector3 end, float sec)
  {
    this.winchInProgress = true;

    float startTime = Time.time;
    float elap = (Time.time - startTime);
    while (elap < sec) {
      elap = (Time.time - startTime);
      float t = Mathf.Clamp(elap / sec, 0, 1);
      foreach (GameObject obj in objects) {
        // Linear interpolation between the two endpoints.
        Vector3 interpolated = (1 - t)*start + t*end;

        if (obj.GetComponent<FollowWaveHeight>() != null) {
          obj.GetComponent<FollowWaveHeight>().nominalPosition = interpolated;
        } else {
          obj.transform.position = interpolated;
        }
      }
      yield return t;
    }

    this.winchInProgress = false;
  }

  bool SetDepth(int row, char buoy, float y, bool animate = false)
  {
    // Ignore request to adjust the depth of buoys that don't exist.
    if (!AddressValid(row, buoy)) {
      return false;
    }

    // Get all overlapping winches at this location (hack, sorry) and move them.
    List<GameObject> winchesToAdjust = GetWinchesAtAddress(row, buoy);

    Vector3 startPosition = winchesToAdjust[0].transform.position;
    Vector3 endPosition = startPosition;
    endPosition.y = Mathf.Clamp(y, this.maxDepth, this.minDepth);

    if (animate) {
      IEnumerator coroutine = AnimateMotion(winchesToAdjust, startPosition, endPosition, 5.0f);
      StartCoroutine(coroutine);

    // Set positions instantaneously.
    } else {
      foreach (GameObject w in winchesToAdjust) {
        // If the winch is following wave heights, we need to override its nominal position.
        if (w.GetComponent<FollowWaveHeight>() != null) {
          w.GetComponent<FollowWaveHeight>().nominalPosition = endPosition;
        } else {
          w.transform.position = endPosition;
        }
      }
    }

    return true;
  }

  bool AddressValid(int row, char buoy)
  {
    return ((row >= 0 && row <= this.maxRow) && (buoy >= 'A' && buoy <= this.maxBuoy));
  }

  //============================================================================
  char NextLetter(char c) { return ++c; }
  char PrevLetter(char c) { return --c; }
  char NextLetterWrap(char c) { return (c == this.maxBuoy) ? 'A' : NextLetter(c); }
  char PrevLetterWrap(char c) { return (c == 'A') ? this.maxBuoy : PrevLetter(c); }

  GameObject GetWinchGridCellCorner(int row, char buoy, CornerCode corner)
  {
    Transform cell = this.gameObject.transform.Find($"MGrid_{buoy}{row}");
    int cornerInt = (int)corner;
    GameObject winch = cell.Find($"Winch_{cornerInt}").gameObject;
    return winch;
  }

  public List<GameObject> GetWinchesAtAddress(int row, char buoy)
  {
    List<GameObject> winches = new List<GameObject>();

    // Is there a grid cell to the bottom-left?
    if (row > 0 && buoy > 'A') {
      winches.Add(GetWinchGridCellCorner(row-1, PrevLetter(buoy), CornerCode.TOPRIGHT));
    }
    // Is there a grid cell to the top-left?
    if (row < this.maxRow && buoy > 'A') {
      winches.Add(GetWinchGridCellCorner(row, PrevLetter(buoy), CornerCode.BOTTOMRIGHT));
    }
    // Is there a grid cell to the top-right?
    if (row < this.maxRow && buoy < this.maxBuoy) {
      winches.Add(GetWinchGridCellCorner(row, buoy, CornerCode.BOTTOMLEFT));
    }
    // Is there a grid cell to the bottom-right?
    if (row > 0 && buoy < this.maxBuoy) {
      winches.Add(GetWinchGridCellCorner(row-1, buoy, CornerCode.TOPLEFT));
    }

    return winches;
  }

  public Transform GetWinchLocation(int row, char buoy)
  {
    return GetWinchesAtAddress(row, buoy)[0].transform;
  }
}
