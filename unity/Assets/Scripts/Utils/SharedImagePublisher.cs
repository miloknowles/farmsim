using System;
using System.IO;
using System.IO.MemoryMappedFiles;
using System.Threading;

using UnityEngine;
using vehicle;

namespace Simulator {


public class SharedImagePublisher {
  private string channel_name = "channel_not_set";
  private int img_height;
  private int img_width;
  private int img_channels;
  private long max_size_bytes;

  private int mutex_timeout_ms = 1;
  private Mutex publish_mutex;
  private MemoryMappedFile mmf;
  private MemoryMappedViewStream stream;
  private BinaryWriter writer;
  private LCM.LCM.LCM lcmHandle;
  private mmf_stereo_image_t msg;
  private int start_byte = 0;
  private byte[] databuf_left, databuf_right;

  public SharedImagePublisher(string channel_name, int img_height, int img_width, int img_channels)
  {
    this.channel_name = channel_name;
    this.img_height = img_height;
    this.img_width = img_width;
    this.img_channels = img_channels;

    // Allocate enough space in shmem to add an uncompressed image pair. Through JPG encoding, we can
    // fit 10-30x more images in that buffer.
    this.max_size_bytes = 2 * img_height * img_width * img_channels;

    GetMutexByName(this.channel_name + "/mutex", out this.publish_mutex);

    // Create the memory-mapped file.
    this.mmf = MemoryMappedFile.CreateOrOpen(this.channel_name + "/mmf", this.max_size_bytes);
    this.stream = this.mmf.CreateViewStream();
    this.writer = new BinaryWriter(this.stream);

    this.lcmHandle = LCM.LCM.LCM.Singleton;

    // Preallocate LCM message.
    this.msg = new mmf_stereo_image_t();
    this.msg.img_left = new mmf_image_t();
    this.msg.img_right = new mmf_image_t();
    this.msg.img_left.mmf_name = this.channel_name + "/mmf";
    this.msg.img_right.mmf_name = this.channel_name + "/mmf";
    this.msg.img_left.width = this.img_width;
    this.msg.img_right.width = this.img_width;
    this.msg.img_left.height = this.img_height;
    this.msg.img_right.height = this.img_height;
    this.msg.img_left.format = "bgr8";
    this.msg.img_right.format = "bgr8";
    this.msg.img_left.encoding = "jpg";
    this.msg.img_right.encoding = "jpg";
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

  // If the next write will surpass the buffer length, go back to the start.
  private int NextByteOffset(int current_offset, int size)
  {
    int end_byte = current_offset + size;
    if (end_byte >= this.max_size_bytes) {
      return 0;
    } else {
      return current_offset;
    }
  }

  public void PublishStereo(ref header_t header_msg, ref Texture2D img_left, ref Texture2D img_right)
  {
    this.msg.header = header_msg;

    // Do the encoding outside of the mutex to hold it for less time.
    this.databuf_left = ImageConversion.EncodeToJPG(img_left, 90);
    this.databuf_right = ImageConversion.EncodeToJPG(img_right, 90);

    this.publish_mutex.WaitOne(this.mutex_timeout_ms);

    // Left image
    this.start_byte = NextByteOffset(this.start_byte, this.databuf_left.Length);
    this.writer.Seek(this.start_byte, System.IO.SeekOrigin.Begin);
    this.writer.Write(this.databuf_left, 0, this.databuf_left.Length);
    this.msg.img_left.byte_offset = this.start_byte;
    this.msg.img_left.size = this.databuf_left.Length;
    this.start_byte += this.databuf_left.Length;

    // Right image
    this.start_byte = NextByteOffset(this.start_byte, this.databuf_right.Length);
    this.writer.Seek(this.start_byte, System.IO.SeekOrigin.Begin);
    this.writer.Write(this.databuf_right, 0, this.databuf_right.Length);
    this.msg.img_right.byte_offset = this.start_byte;
    this.msg.img_right.size = this.databuf_right.Length;
    this.start_byte += this.databuf_right.Length;

    this.publish_mutex.ReleaseMutex();

    this.lcmHandle.Publish(this.channel_name, this.msg);
  }
};


}
