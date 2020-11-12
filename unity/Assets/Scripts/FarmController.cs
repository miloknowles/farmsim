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
  public int maxRow = 2;
  public char maxBuoy = 'D';

  // Start is called before the first frame update
  void Start()
  {

  }

  // Update is called once per frame
  void Update()
  {

  }

  bool AdjustDepth(int row, char buoy, float deltaY)
  {
    // Ignore request to adjust the depth of buoys that don't exist.
    if (!AddressValid(row, buoy)) {
      return false;
    }

    // Get all overlapping winches at this location (hack, sorry) and move them.
    List<GameObject> winchesToAdjust = GetWinchesAtAddress(row, buoy);
    foreach (GameObject w in winchesToAdjust) {
      Vector3 position = w.transform.position;
      position.y += deltaY;
      w.transform.position = position;
    }

    return true;
  }

  bool SetDepth(int row, char buoy, float y)
  {
    // Ignore request to adjust the depth of buoys that don't exist.
    if (!AddressValid(row, buoy)) {
      return false;
    }

    // Get all overlapping winches at this location (hack, sorry) and move them.
    List<GameObject> winchesToAdjust = GetWinchesAtAddress(row, buoy);
    foreach (GameObject w in winchesToAdjust) {
      Vector3 position = w.transform.position;
      position.y += y;
      w.transform.position = position;
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

  GameObject GetWinchGridCellCorner(int row, char buoy, CornerCode corner)
  {
    Transform cell = this.gameObject.transform.Find($"MGrid_{buoy}{row}");

    int cornerInt = (int)corner;
    GameObject winch = cell.Find($"Winch_{cornerInt}").gameObject;
    return winch;
  }

  List<GameObject> GetWinchesAtAddress(int row, char buoy)
  {
    List<GameObject> winches = new List<GameObject>();

    // Is there a grid cell to the bottom-left?
    if (row > 0 && buoy > 'A') {
      winches.Add(GetWinchGridCellCorner(row-1, PrevLetter(buoy), CornerCode.TOPRIGHT));
    }
    // Is there a grid cell to the top-left?
    if (row < this.maxRow && buoy > 'A') {
      winches.Add(GetWinchGridCellCorner(row+1, PrevLetter(buoy), CornerCode.BOTTOMRIGHT));
    }
    // Is there a grid cell to the top-right?
    if (row < this.maxRow && buoy < this.maxBuoy) {
      winches.Add(GetWinchGridCellCorner(row+1, NextLetter(buoy), CornerCode.BOTTOMLEFT));
    }
    // Is there a grid cell to the bottom-right?
    if (row > 0 && buoy < this.maxBuoy) {
      winches.Add(GetWinchGridCellCorner(row-1, NextLetter(buoy), CornerCode.TOPLEFT));
    }

    return winches;
  }
}
