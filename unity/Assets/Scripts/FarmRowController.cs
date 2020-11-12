using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;


public class BuoyObjects {
  public BuoyObjects(GameObject _buoy,
                     GameObject _buoyDropLine,
                     GameObject _winch,
                     GameObject _mLineBefore,
                    GameObject _mLineAfter)
  {
    buoy = _buoy;
    buoyDropLine = _buoyDropLine;
    winch = _winch;
    mLineBefore = _mLineBefore;
    mLineAfter = _mLineAfter;
  }

  public GameObject buoy { get; }
  public GameObject buoyDropLine { get; }
  public GameObject winch { get; }
  public GameObject mLineBefore { get; }
  public GameObject mLineAfter { get; }
}


public class FarmRowController : MonoBehaviour {
  // public int farmRowIndex;
  // public char minBuoyLetter = 'A';
  // public char maxBuoyLetter = 'D';

  // private Dictionary<string, BuoyObjects> addressLookup;

  // void Start()
  // {
  //   addressLookup = new Dictionary<string, BuoyObjects>();

  //   for (char ltr = minBuoyLetter; ltr <= maxBuoyLetter; ltr++) {
  //     string buoyAddress = $"{ltr}{this.farmRowIndex}";

  //     GameObject buoy = GetBuoyByAddress(buoyAddress);
  //     GameObject winch = GetWinchByAddress(buoyAddress);
  //     GameObject

  //     // GameObject winch = this.gameObject.transform.Find(
  //     //     $"AdjustableEquipment/Winch_{buoyAddress}").gameObject;
  //     // GameObject buoy = GetBuoyByAddress(buoyAddress);
  //     // GameObject dropLine = this.gameObject.transform.Find(
  //     //     $"AdjustableEquipment/DropperBuoy_{buoyAddress}/AdjustableLine_{buoyAddress}").gameObject;
  //     // GameObject lBefore = this.gameObject.transform.Find(
  //     //     $"AdjustableEquipment/mLine/Segment_{PrevLetter(ltr)}x{ltr}").gameObject;
  //     // GameObject lAfter = this.gameObject.transform.Find(
  //     //     $"AdjustableEquipment/mLine/Segment_{ltr}x{NextLetter(ltr)}").gameObject;

  //     this.addressLookup.Add(buoyAddress, new BuoyObjects(buoy, dropLine, winch, lBefore, lAfter));
  //   }

  //   // // Collect objects for the moored buoys.
  //   // GameObject buoy = this.gameObject.transform.Find(
  //   //         $"MooredBuoy_{buoyAddress}/Buoy_{buoyAddress}").gameObject;
  //   // GameObject winch = this.gameObject.transform.Find(
  //   //     $"AdjustableEquipment/Winch_{buoyAddress}").gameObject;

  //   // // If this is a moored buoy, need to find the buoy gameObject a different way.
  //   //   if (ltr == minBuoyLetter || ltr == maxBuoyLetter) {
  //   //     buoy = this.gameObject.transform.Find(
  //   //         $"MooredBuoy_{buoyAddress}/Buoy_{buoyAddress}").gameObject;
  //   //   }

  //   PrintBuoysFound();
  // }

  // void PrintBuoysFound()
  // {
  //   Debug.Log(this.addressLookup.Count);
  //   foreach (KeyValuePair<string, BuoyObjects> kvp in this.addressLookup) {
  //     Debug.Log($"Key = {kvp.Key}");
  //   }
  // }

  // GameObject GetBuoyByAddress(string a)
  // {
  //   if (a[0] == this.minBuoyLetter || a[0] == this.maxBuoyLetter) {
  //     return this.gameObject.transform.Find(
  //         $"MooredBuoy_{buoyAddress}/Buoy_{buoyAddress}").gameObject;
  //   } else {
  //     return this.gameObject.transform.Find(
  //         $"AdjustableEquipment/DropperBuoy_{buoyAddress}/Buoy_{buoyAddress}").gameObject;
  //   }
  // }

  // GameObject GetWinchByAddress(string a)
  // {
  //   return this.gameObject.transform.Find(
  //       $"AdjustableEquipment/Winch_{buoyAddress}").gameObject;
  // }

  // GameObject GetDropLineByAddress(string a)
  // {
  //   if (a[0] == this.minBuoyLetter || a[0] == this.maxBuoyLetter) {
  //     return null;
  //   } else {
  //     return this.gameObject.transform.Find(
  //         $"AdjustableEquipment/DropperBuoy_{buoyAddress}/AdjustableLine_{buoyAddress}").gameObject;
  //   }
  // }

  // GameObject GetMLineBeforeByAddress(string a)
  // {
  //   // No line segment before the first buoy.
  //   if (a[0] == this.minBuoyLetter) {
  //     return null;
  //   }
  //   return this.gameObject.transform.Find(
  //       $"AdjustableEquipment/mLine/Segment_{PrevLetter(ltr)}x{ltr}").gameObject;
  // }

  // GameObject GetMLineAfterByAddress(string a)
  // {
  //   // No line segment after the last buoy.
  //   if (a[0] == this.maxBuoyLetter) {
  //     return null;
  //   }
  //   return this.gameObject.transform.Find(
  //       $"AdjustableEquipment/mLine/Segment_{ltr}x{NextLetter(ltr)}").gameObject;
  // }

  // char NextLetter(char c) { return ++c; }
  // char PrevLetter(char c) { return --c; }

  // // Update is called once per frame
  // void Update()
  // {

  // }
}
