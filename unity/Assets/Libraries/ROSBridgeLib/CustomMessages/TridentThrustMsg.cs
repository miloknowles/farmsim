using System.Collections;
using System.Text;
using SimpleJSON;
using UnityEngine;


namespace ROSBridgeLib {
	namespace CustomMessages {
		public class TridentThrustMsg : ROSBridgeMsg {
			// Force commands for each Thrust.
      private float _F_lt, _F_rt, _F_ct;

			public TridentThrustMsg(JSONNode msg) {
				this._F_lt = float.Parse(msg["F_lt"]);
				this._F_rt = float.Parse(msg["F_rt"]);
				this._F_ct = float.Parse(msg["F_ct"]);
			}

			public TridentThrustMsg(float F_lt, float F_rt, float F_ct) {
				this._F_lt = F_lt;
				this._F_rt = F_rt;
				this._F_ct = F_ct;
			}

			public static string getMessageType() {
				return "auv/TridentThrust";
			}

			public float GetFlt() { return this._F_lt; }
			public float GetFrt() { return this._F_rt; }
			public float GetFct() { return this._F_ct; }

			public override string ToString() {
				return "auv/TridentThrust [F_lt=" + this._F_lt +
						", F_rt=" + this._F_rt +
						", F_ct=" + this._F_ct + "]";
			}

			public override string ToYAMLString() {
				return "{\"F_lt\": " + this._F_lt +
						", \"F_rt\": " + this._F_rt +
						", \"F_ct\": " + this._F_ct +
						" }";
			}
		}
	}
}


