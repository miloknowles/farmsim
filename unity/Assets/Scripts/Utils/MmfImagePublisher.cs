using System;
using System.IO;
using System.IO.MemoryMappedFiles;
using System.Threading;
using System.Runtime.InteropServices;
using System.Threading.Tasks;
using System.Data;

using UnityEngine;
using vehicle;

namespace Simulator {


public class MmfImagePublisher {
  private string channel_name = "channel_not_set";
  private int img_height;
  private int img_width;
  private int img_channels;
  private long mmf_size_bytes;
  private string mmf_filename;

  // private int mutex_timeout_ms = 1;
  // private Mutex publish_mutex;
  private MemoryMappedFile mmf;
  private MemoryMappedViewStream stream;
  private BinaryWriter writer;
  private LCM.LCM.LCM lcmHandle;
  private mmf_stereo_image_t msg;
  private int start_byte = 0;
  private byte[] lbuf, rbuf;

  public MmfImagePublisher(string channel_name, int img_height, int img_width, int img_channels)
  {
    this.channel_name = channel_name;
    this.img_height = img_height;
    this.img_width = img_width;
    this.img_channels = img_channels;

    // Allocate enough space in shmem to add an uncompressed image pair.
    // Through JPG encoding, we can fit 10-30 images in that buffer.
    this.mmf_size_bytes = 2 * img_height * img_width * img_channels;

    // GetMutexByName(this.channel_name + "/mutex", out this.publish_mutex);
    this.mmf_filename = GetMmfFilenameSafe(
        SimulationParams.SYSTEM_SHM_FOLDER,
        ConvertChannelToFilename(channel_name));

    // Create the memory-mapped file.
    this.mmf = MemoryMappedFile.CreateFromFile(
        this.mmf_filename,
        FileMode.Open,
        this.mmf_filename,
        this.mmf_size_bytes);

    Debug.Log("Will write images to memory-mapped file: " + this.mmf_filename);

    this.stream = this.mmf.CreateViewStream();
    this.writer = new BinaryWriter(this.stream);
    this.lcmHandle = LCM.LCM.LCM.Singleton;

    // Preallocate LCM message.
    this.msg = new mmf_stereo_image_t();
    this.msg.img_left = new mmf_image_t();
    this.msg.img_right = new mmf_image_t();
    this.msg.img_left.mm_filename = this.mmf_filename;
    this.msg.img_right.mm_filename = this.mmf_filename;
    this.msg.img_left.width = this.img_width;
    this.msg.img_right.width = this.img_width;
    this.msg.img_left.height = this.img_height;
    this.msg.img_right.height = this.img_height;
    this.msg.img_left.format = "bgr8";
    this.msg.img_right.format = "bgr8";
    this.msg.img_left.encoding = "jpg";
    this.msg.img_right.encoding = "jpg";
  }

  // Makes sure that the system shm_folder exists, and that the shm_filename within it also exists.
  private string GetMmfFilenameSafe(string shm_folder, string shm_filename)
  {
    if (!Directory.Exists(shm_folder)) {
      Debug.Log("WARNING: shm_folder does not exist, creating: " + shm_folder);
      Directory.CreateDirectory(shm_folder);
    }

    string shm_path = Path.Combine(shm_folder, shm_filename);

    if (!File.Exists(shm_path)) {
      Debug.Log("WARNING: shm_filename does not exist, creating: " + shm_path);
      File.Create(shm_path);
    }

    return shm_path;
  }

  // Replaces slashes in the channel name with __, and adds a .shm extension.
  private string ConvertChannelToFilename(string channel_name)
  {
    return channel_name.Replace("/", "__") + ".shm";
  }

  // Acquires (locks) a mutex with the specified name. If the mutex does not exist yet (e.g on
  // first publish), it is created.
  private bool GetMutexByName(string mutex_name, out Mutex mutex)
  {
    bool did_exist = Mutex.TryOpenExisting(mutex_name, out mutex);

    if (!did_exist) {
      bool mutex_created;
      mutex = new Mutex(true, mutex_name, out mutex_created);
    }

    return !did_exist;
  }

  // Writes a byte array to the memory-mapped file. Returns the byte offset to the data in the MMF.
  private int WriteToMmf(ref byte[] data)
  {
    int end_byte = this.start_byte + data.Length;
    if (end_byte >= this.mmf_size_bytes) {
      this.start_byte = 0;
    }

    this.writer.Seek(this.start_byte, System.IO.SeekOrigin.Begin);
    this.writer.Write(data, 0, data.Length);

    return this.start_byte;
  }

  // Publish a stereo image to shared memory. First writes the image data to a memory-mapped file,
  // and then publishes metadata about how to access that raw data.
  public void PublishStereo(ref header_t header_msg, ref Texture2D img_left, ref Texture2D img_right)
  {
    this.msg.header = header_msg;

    // // Do the encoding outside of the mutex to hold it for less time.
    this.lbuf = ImageConversion.EncodeToJPG(img_left, 90);
    this.rbuf = ImageConversion.EncodeToJPG(img_right, 90);

    // this.publish_mutex.WaitOne(this.mutex_timeout_ms);

    // Left image
    this.msg.img_left.offset = this.WriteToMmf(ref this.lbuf);
    this.msg.img_left.size = this.lbuf.Length;

    this.start_byte += this.lbuf.Length;

    // Right image
    this.msg.img_right.offset = this.WriteToMmf(ref this.rbuf);
    this.msg.img_right.size = this.rbuf.Length;

    this.start_byte += this.rbuf.Length;

    // this.publish_mutex.ReleaseMutex();
    this.lcmHandle.Publish(this.channel_name, this.msg);
  }
};


}
