using System.Collections;


namespace ROSBridgeLib {
  // Helper functions for serializing data into strings.
  public static class FormatUtils {

    /**
     * Format a 1D array into string that looks like [ 0, 1, ... , 2 ].
     *
     * For example, call this function with:
     *    string output = FormatUtils.ArrayToString<double>(input);
     */
    public static string ArrayToString<T>(ref T[] array) {
      return "[ " + string.Join(", ", array) + " ]";
    }
  }
}
