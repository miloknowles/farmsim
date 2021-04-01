/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

using System;
using System.Collections.Generic;
using System.IO;
using LCM.LCM;
 
namespace vehicle
{
    public sealed class vector3_t : LCM.LCM.LCMEncodable
    {
        public double x;
        public double y;
        public double z;
 
        public vector3_t()
        {
        }
 
        public static readonly ulong LCM_FINGERPRINT;
        public static readonly ulong LCM_FINGERPRINT_BASE = 0x573f2fdd2f76508fL;
 
        static vector3_t()
        {
            LCM_FINGERPRINT = _hashRecursive(new List<String>());
        }
 
        public static ulong _hashRecursive(List<String> classes)
        {
            if (classes.Contains("vehicle.vector3_t"))
                return 0L;
 
            classes.Add("vehicle.vector3_t");
            ulong hash = LCM_FINGERPRINT_BASE
                ;
            classes.RemoveAt(classes.Count - 1);
            return (hash<<1) + ((hash>>63)&1);
        }
 
        public void Encode(LCMDataOutputStream outs)
        {
            outs.Write((long) LCM_FINGERPRINT);
            _encodeRecursive(outs);
        }
 
        public void _encodeRecursive(LCMDataOutputStream outs)
        {
            outs.Write(this.x); 
 
            outs.Write(this.y); 
 
            outs.Write(this.z); 
 
        }
 
        public vector3_t(byte[] data) : this(new LCMDataInputStream(data))
        {
        }
 
        public vector3_t(LCMDataInputStream ins)
        {
            if ((ulong) ins.ReadInt64() != LCM_FINGERPRINT)
                throw new System.IO.IOException("LCM Decode error: bad fingerprint");
 
            _decodeRecursive(ins);
        }
 
        public static vehicle.vector3_t _decodeRecursiveFactory(LCMDataInputStream ins)
        {
            vehicle.vector3_t o = new vehicle.vector3_t();
            o._decodeRecursive(ins);
            return o;
        }
 
        public void _decodeRecursive(LCMDataInputStream ins)
        {
            this.x = ins.ReadDouble();
 
            this.y = ins.ReadDouble();
 
            this.z = ins.ReadDouble();
 
        }
 
        public vehicle.vector3_t Copy()
        {
            vehicle.vector3_t outobj = new vehicle.vector3_t();
            outobj.x = this.x;
 
            outobj.y = this.y;
 
            outobj.z = this.z;
 
            return outobj;
        }
    }
}

