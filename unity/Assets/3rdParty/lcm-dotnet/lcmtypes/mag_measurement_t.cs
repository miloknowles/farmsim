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
    public sealed class mag_measurement_t : LCM.LCM.LCMEncodable
    {
        public vehicle.header_t header;
        public vehicle.vector3_t bM;
 
        public mag_measurement_t()
        {
        }
 
        public static readonly ulong LCM_FINGERPRINT;
        public static readonly ulong LCM_FINGERPRINT_BASE = 0x88ce1ef00f3317c3L;
 
        static mag_measurement_t()
        {
            LCM_FINGERPRINT = _hashRecursive(new List<String>());
        }
 
        public static ulong _hashRecursive(List<String> classes)
        {
            if (classes.Contains("vehicle.mag_measurement_t"))
                return 0L;
 
            classes.Add("vehicle.mag_measurement_t");
            ulong hash = LCM_FINGERPRINT_BASE
                 + vehicle.header_t._hashRecursive(classes)
                 + vehicle.vector3_t._hashRecursive(classes)
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
            this.header._encodeRecursive(outs); 
 
            this.bM._encodeRecursive(outs); 
 
        }
 
        public mag_measurement_t(byte[] data) : this(new LCMDataInputStream(data))
        {
        }
 
        public mag_measurement_t(LCMDataInputStream ins)
        {
            if ((ulong) ins.ReadInt64() != LCM_FINGERPRINT)
                throw new System.IO.IOException("LCM Decode error: bad fingerprint");
 
            _decodeRecursive(ins);
        }
 
        public static vehicle.mag_measurement_t _decodeRecursiveFactory(LCMDataInputStream ins)
        {
            vehicle.mag_measurement_t o = new vehicle.mag_measurement_t();
            o._decodeRecursive(ins);
            return o;
        }
 
        public void _decodeRecursive(LCMDataInputStream ins)
        {
            this.header = vehicle.header_t._decodeRecursiveFactory(ins);
 
            this.bM = vehicle.vector3_t._decodeRecursiveFactory(ins);
 
        }
 
        public vehicle.mag_measurement_t Copy()
        {
            vehicle.mag_measurement_t outobj = new vehicle.mag_measurement_t();
            outobj.header = this.header.Copy();
 
            outobj.bM = this.bM.Copy();
 
            return outobj;
        }
    }
}

