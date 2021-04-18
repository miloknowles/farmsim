using System;
using System.IO;
using System.IO.MemoryMappedFiles;
using System.Threading;

using UnityEngine;
using vehicle;

namespace Simulator {


public class SharedImagePublisher {
  // Acquires (locks) a mutex with the specified name. If the mutex does not exist yet (e.g on
  // first publish), it is created.
  public static bool GetMutexByName(string mutex_name, out Mutex mutex)
  {
    bool did_exist = Mutex.TryOpenExisting(mutex_name, out mutex);

    if (!did_exist) {
      bool mutex_created;
      mutex = new Mutex(true, mutex_name, out mutex_created);
    }

    return !did_exist;
  }

  public static void PublishStereoImage(string channel_name, ref stereo_image_t msg, int timeout_ms)
  {
    string mutex_name = channel_name + "/mutex";
    long size_bytes = 1000;

    using (MemoryMappedFile mmf = MemoryMappedFile.CreateOrOpen(channel_name, size_bytes))
    {
      Mutex mutex;
      bool was_created = GetMutexByName(mutex_name, out mutex);

      if (!was_created) {
        mutex.WaitOne(timeout_ms);  // Wait for the mutex to become free.
      }

      using (MemoryMappedViewStream stream = mmf.CreateViewStream())
      {
        BinaryWriter writer = new BinaryWriter(stream);
        writer.Write(1);
      }

      mutex.ReleaseMutex();
    }
  }
};


}
