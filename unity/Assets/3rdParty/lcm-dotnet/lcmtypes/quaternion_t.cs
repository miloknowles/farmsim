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
    public sealed class quaternion_t : LCM.LCM.LCMEncodable
    {
        public double w;
        public double x;
        public double y;
        public double z;
 
        public quaternion_t()
        {
        }
 
        public static readonly ulong LCM_FINGERPRINT;
        public static readonly ulong LCM_FINGERPRINT_BASE = 0x9b2deea5fc88050fL;
 
        static quaternion_t()
        {
            LCM_FINGERPRINT = _hashRecursive(new List<String>());
        }
 
        public static ulong _hashRecursive(List<String> classes)
        {
            if (classes.Contains("vehicle.quaternion_t"))
                return 0L;
 
            classes.Add("vehicle.quaternion_t");
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
            outs.Write(this.w); 
 
            outs.Write(this.x); 
 
            outs.Write(this.y); 
 
            outs.Write(this.z); 
 
        }
 
        public quaternion_t(byte[] data) : this(new LCMDataInputStream(data))
        {
        }
 
        public quaternion_t(LCMDataInputStream ins)
        {
            if ((ulong) ins.ReadInt64() != LCM_FINGERPRINT)
                throw new System.IO.IOException("LCM Decode error: bad fingerprint");
 
            _decodeRecursive(ins);
        }
 
        public static vehicle.quaternion_t _decodeRecursiveFactory(LCMDataInputStream ins)
        {
            vehicle.quaternion_t o = new vehicle.quaternion_t();
            o._decodeRecursive(ins);
            return o;
        }
 
        public void _decodeRecursive(LCMDataInputStream ins)
        {
            this.w = ins.ReadDouble();
 
            this.x = ins.ReadDouble();
 
            this.y = ins.ReadDouble();
 
            this.z = ins.ReadDouble();
 
        }
 
        public vehicle.quaternion_t Copy()
        {
            vehicle.quaternion_t outobj = new vehicle.quaternion_t();
            outobj.w = this.w;
 
            outobj.x = this.x;
 
            outobj.y = this.y;
 
            outobj.z = this.z;
 
            return outobj;
        }
    }
}

