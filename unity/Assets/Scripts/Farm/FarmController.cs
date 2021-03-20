using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;


public enum CornerCode
{
  BOTTOMLEFT = 0,
  TOPLEFT = 1,
  TOPRIGHT = 2,
  BOTTOMRIGHT = 3
}


// 26^2 = 676 locations should be sufficient.
public struct BuoyCode {
  private char[] _code;

  // Construct with a string.
  public BuoyCode(string s) {
    Assert.IsTrue(s.Length == 1 || s.Length == 2);
    this._code = s.ToCharArray();
    Validate(this._code);
  }

  // Construct with a char array.
  public BuoyCode(char[] c) {
    Assert.IsTrue(c.Length == 1 || c.Length == 2);
    this._code = c;
    Validate(this._code);
  }

  // Construct with a single character.
  public BuoyCode(char c) {
    this._code = new char[1]{c};
    Validate(this._code);
  }

  // Make sure all letters are in the alphabet.
  private void Validate(char[] array) {
    foreach (char c in array) {
      Assert.IsTrue(c >= 'A' && c <= 'Z');
    }
  }

  // Inteprets an alphabetic code as a Base-26 number.
  public int Integer() {
    int num = 0;
    int powerOf26 = 1;
    for (int ord = 0; ord < this._code.Length; ++ord) {
      int indexOfBit = this._code.Length - ord - 1;
      num += (int)(this._code[indexOfBit] - 'A') * powerOf26;
      powerOf26 *= 26;
    }
    return num;
  }
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
    ConfigureAprilTags();
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

  public bool SetDepth(int row, char buoy, float y, bool animate = false)
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

  private bool AddressValid(int row, char buoy)
  {
    return ((row >= 0 && row <= this.maxRow) && (buoy >= 'A' && buoy <= this.maxBuoy));
  }

  //============================================================================
  char NextLetter(char c) { return ++c; }
  char PrevLetter(char c) { return --c; }
  char NextLetterWrap(char c) { return (c == this.maxBuoy) ? 'A' : NextLetter(c); }
  char PrevLetterWrap(char c) { return (c == 'A') ? this.maxBuoy : PrevLetter(c); }

  private GameObject GetWinchGridCellCorner(int row, char buoy, CornerCode corner)
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

  private int CharToInt(char c) { return (int)(c - 'A'); }

  private void SetChildrenActive(GameObject o, bool active)
  {
    for (int i = 0; i < o.transform.childCount; ++i) {
      o.transform.GetChild(i).gameObject.SetActive(active);
    }
  }

  private void ConfigureAprilTags()
  {
    for (int row = 0; row <= this.maxRow; ++row) {
      for (char buoy = 'A'; buoy <= this.maxBuoy; buoy = NextLetter(buoy)) {
        List<GameObject> winches = GetWinchesAtAddress(row, buoy);

        // AprilTag ID is in row-major order.
        int aprilIndex = row*(CharToInt(this.maxBuoy)+1) + CharToInt(buoy) + SimulationController.RESERVED_APRILTAGS;
        // Debug.Log($"Assigning AprilTag id={aprilIndex}");

        // NOTE(milo): Unity doesn't find these resources with .png at the end!
        string texturePath = $"AprilTags/tagStandard41h12/tag41_12_{aprilIndex:00000}";
        Texture2D tex = Resources.Load(texturePath) as Texture2D;
        GameObject ATCube = winches[0].transform.Find("ATCube_Winch").gameObject;
        ATCube.GetComponent<MeshRenderer>().material.SetTexture("_MainTex", tex);

        // Make only one of the winches visible.
        for (int i = 1; i < winches.Count; ++i) {
          SetChildrenActive(winches[i], false);
        }
      }
    }
  }
}
