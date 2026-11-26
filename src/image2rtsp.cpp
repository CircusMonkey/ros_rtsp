// NOTE: Do not reorder these since it does not work in all orders.
// 1. Standard C/C++ libraries first (defines core types)
#include <stdio.h>
#include <string>
#include <chrono>

// 2. Framework and plugin headers
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// 3. GStreamer and GLib-based RTSP server headers
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>

// 4. ROS Core and Message headers
#include "sensor_msgs/Image.h"
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

// 5. Local project headers last
#include <image2rtsp.h>

using namespace std;
using namespace image2rtsp;

void Image2RTSPNodelet::onInit() {
  string pipeline, mountpoint, bitrate, caps, pipeline_tail, keyframe_interval;

  NODELET_DEBUG("Initializing image2rtsp nodelet...");

  if (getenv((char *)"GST_DEBUG") == NULL) {
    // set GST_DEBUG to warning if unset
    putenv((char *)"GST_DEBUG=*:1");
  }

  ros::NodeHandle &nh = getPrivateNodeHandle();

  // Get the parameters from the rosparam server
  XmlRpc::XmlRpcValue streams;
  nh.getParam("streams", streams);
  ROS_ASSERT(streams.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  ROS_INFO("Number of RTSP streams: %d", streams.size());
  nh.getParam("port", this->port);

  video_mainloop_start();
  rtsp_server = rtsp_server_create(port);

  // Go through and parse each stream
  for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = streams.begin();
       it != streams.end(); ++it) {
    XmlRpc::XmlRpcValue stream = streams[it->first];
    ROS_INFO_STREAM("Found stream: " << (std::string)(it->first) << " ==> "
                                     << stream);

    // Set up pipeline tail
    keyframe_interval =
        std::to_string(static_cast<int>(stream["keyframe_interval"]));
    pipeline_tail = " key-int-max=" + keyframe_interval +
                    " ! video/x-h264, profile=baseline ! "
                    "rtph264pay name=pay0 "
                    "pt=96 )"; // Gets completed based on rosparams below

    // Convert to string for ease of use
    mountpoint = static_cast<std::string>(stream["mountpoint"]);
    bitrate = std::to_string(static_cast<int>(stream["bitrate"]));

    // uvc camera?
    if (stream["type"] == "cam") {
      pipeline = "( " + static_cast<std::string>(stream["source"]) +
                 " ! x264enc tune=zerolatency bitrate=" + bitrate +
                 pipeline_tail;

      rtsp_server_add_url(mountpoint.c_str(), pipeline.c_str(), NULL);
    }
    // ROS Image topic?
    else if (stream["type"] == "topic") {
      /* Keep record of number of clients connected to each topic.
       * so we know to stop subscribing when no-one is connected. */
      num_of_clients[mountpoint] = 0;
      appsrc[mountpoint] = NULL;
      caps = static_cast<std::string>(stream["caps"]);

      // Setup the full pipeline
      /*pipeline = "( appsrc name=imagesrc do-timestamp=true min-latency=0 "
                 "max-latency=0 max-bytes=1000 is-live=true ! videoconvert ! "
                 "videoscale ! " +
                 caps + " ! x264enc tune=zerolatency bitrate=" + bitrate +
                 pipeline_tail;*/

      pipeline = "( appsrc name=imagesrc do-timestamp=true "
                 "is-live=true min-latency=0 ! "
                 "queue leaky=downstream "
                 "max-size-buffers=1 "
                 "max-size-bytes=0 "
                 "max-size-time=0 ! "
                 "videoconvert ! videoscale ! "
                 "videorate drop-only=true ! " +
                 caps +
                 " ! "
                 "x264enc tune=zerolatency "
                 "speed-preset=ultrafast "
                 "bitrate=" +
                 bitrate + pipeline_tail;

      // Add the pipeline to the rtsp server
      rtsp_server_add_url(mountpoint.c_str(), pipeline.c_str(),
                          (GstElement **)&(appsrc[mountpoint]));
      ROS_INFO_STREAM("[Image2RTSPNodelet::OnInit] Pipeline: \n" << pipeline);
    } else {
      ROS_ERROR("Undefined stream type. Check your stream_setup.yaml file.");
    }
    NODELET_INFO("Stream available at rtsp://%s:%s%s",
                 gst_rtsp_server_get_address(rtsp_server), port.c_str(),
                 mountpoint.c_str());
  }
}

/* Modified from
 * https://github.com/ProjectArtemis/gst_video_server/blob/master/src/server_nodelet.cpp
 */
GstCaps *Image2RTSPNodelet::gst_caps_new_from_image(
    const sensor_msgs::Image::ConstPtr &msg) {
  // http://gstreamer.freedesktop.org/data/doc/gstreamer/head/pwg/html/section-types-definitions.html
  static const ros::M_string known_formats = {{
      {sensor_msgs::image_encodings::RGB8, "RGB"},
      {sensor_msgs::image_encodings::RGB16, "RGB16"},
      {sensor_msgs::image_encodings::RGBA8, "RGBA"},
      {sensor_msgs::image_encodings::RGBA16, "RGBA16"},
      {sensor_msgs::image_encodings::BGR8, "BGR"},
      {sensor_msgs::image_encodings::BGR16, "BGR16"},
      {sensor_msgs::image_encodings::BGRA8, "BGRA"},
      {sensor_msgs::image_encodings::BGRA16, "BGRA16"},
      {sensor_msgs::image_encodings::MONO8, "GRAY8"},
      {sensor_msgs::image_encodings::MONO16, "GRAY16_LE"},
  }};

  if (msg->is_bigendian) {
    ROS_ERROR("GST: big endian image format is not supported");
    return nullptr;
  }

  auto format = known_formats.find(msg->encoding);
  if (format == known_formats.end()) {
    ROS_ERROR("GST: image format '%s' unknown", msg->encoding.c_str());
    return nullptr;
  }

  return gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING,
                             format->second.c_str(), "width", G_TYPE_INT,
                             msg->width, "height", G_TYPE_INT, msg->height,
                             "framerate", GST_TYPE_FRACTION, 24, 1, nullptr);
}

/*void Image2RTSPNodelet::imageCallback(const sensor_msgs::Image::ConstPtr &msg,
                                      const std::string &topic) {
  GstBuffer *buf;

  GstCaps *caps;
  char *gst_type, *gst_format = (char *)"";
  // g_print("Image encoding: %s\n", msg->encoding.c_str());
  if (appsrc[topic] != NULL) {
    // Set caps from message
    caps = gst_caps_new_from_image(msg);
    // gst_app_src_set_caps(appsrc[topic], caps);
    if (caps != nullptr) {
      gst_app_src_set_caps(appsrc[topic], caps);
      gst_caps_unref(caps);
    }

    buf = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
    gst_buffer_fill(buf, 0, msg->data.data(), msg->data.size());
    GST_BUFFER_FLAG_SET(buf, GST_BUFFER_FLAG_LIVE);

    guint64 queued_bytes = 0;

    g_object_get(G_OBJECT(appsrc[topic]), "current-level-bytes", &queued_bytes,
                 nullptr);

    const guint64 frame_bytes = static_cast<guint64>(msg->step) * msg->height;

    ROS_INFO_THROTTLE(
        1.0, "Appsrc queue: %" G_GUINT64_FORMAT " bytes (%.2f frames)",
        queued_bytes,
        frame_bytes > 0 ? static_cast<double>(queued_bytes) / frame_bytes
                        : 0.0);

    gst_app_src_push_buffer(appsrc[topic], buf);
  }
}*/

void Image2RTSPNodelet::imageCallback(const sensor_msgs::Image::ConstPtr &msg,
                                      const std::string &topic) {
  using Clock = std::chrono::steady_clock;

  const auto t0 = Clock::now();

  auto elapsed_ms = [](Clock::time_point start, Clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
  };

  GstAppSrc *source = appsrc[topic];

  if (source == nullptr) {
    return;
  }

  // ------------------------------------------------
  // 1. Create and set GStreamer caps
  // ------------------------------------------------
  auto t_caps_start = Clock::now();

  GstCaps *caps = gst_caps_new_from_image(msg);

  if (caps != nullptr) {
    gst_app_src_set_caps(GST_APP_SRC(source), caps);
    gst_caps_unref(caps);
  }

  auto t_caps_end = Clock::now();

  // ------------------------------------------------
  // 2. Allocate GstBuffer
  // ------------------------------------------------
  auto t_alloc_start = Clock::now();

  GstBuffer *buf = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);

  auto t_alloc_end = Clock::now();

  if (buf == nullptr) {
    ROS_ERROR_THROTTLE(1.0, "Failed to allocate GstBuffer");
    return;
  }

  // ------------------------------------------------
  // 3. Copy image data into GstBuffer
  // ------------------------------------------------
  auto t_copy_start = Clock::now();

  gsize copied_bytes =
      gst_buffer_fill(buf, 0, msg->data.data(), msg->data.size());

  auto t_copy_end = Clock::now();

  GST_BUFFER_FLAG_SET(buf, GST_BUFFER_FLAG_LIVE);

  // ------------------------------------------------
  // 4. Check appsrc queue
  // ------------------------------------------------
  auto t_queue_start = Clock::now();

  guint64 queued_bytes = 0;

  g_object_get(G_OBJECT(source), "current-level-bytes", &queued_bytes, nullptr);

  auto t_queue_end = Clock::now();

  // ------------------------------------------------
  // 5. Push buffer into appsrc
  // ------------------------------------------------
  auto t_push_start = Clock::now();

  GstFlowReturn flow = gst_app_src_push_buffer(GST_APP_SRC(source), buf);

  auto t_push_end = Clock::now();

  // ------------------------------------------------
  // 6. Total callback runtime
  // ------------------------------------------------
  auto t_end = Clock::now();

  ROS_INFO_THROTTLE(
      1.0,
      "[RTSP TIMING] "
      "Total: %.3f ms | "
      "Caps: %.3f ms | "
      "Allocation: %.3f ms | "
      "Image copy: %.3f ms | "
      "Queue query: %.3f ms | "
      "Appsrc push: %.3f ms | "
      "Queue: %.2f frames | "
      "Copied: %lu bytes | "
      "Flow: %s",

      elapsed_ms(t0, t_end), elapsed_ms(t_caps_start, t_caps_end),
      elapsed_ms(t_alloc_start, t_alloc_end),
      elapsed_ms(t_copy_start, t_copy_end),
      elapsed_ms(t_queue_start, t_queue_end),
      elapsed_ms(t_push_start, t_push_end),

      msg->step * msg->height > 0
          ? static_cast<double>(queued_bytes) / (msg->step * msg->height)
          : 0.0,

      static_cast<unsigned long>(copied_bytes), gst_flow_get_name(flow));
}

void Image2RTSPNodelet::url_connected(string url) {
  std::string mountpoint, source, type;

  NODELET_INFO("Client connected: %s", url.c_str());
  ros::NodeHandle &nh = getPrivateNodeHandle();

  // Get the parameters from the rosparam server
  XmlRpc::XmlRpcValue streams;
  nh.getParam("streams", streams);
  ROS_ASSERT(streams.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  // Go through and parse each stream
  for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = streams.begin();
       it != streams.end(); ++it) {
    XmlRpc::XmlRpcValue stream = streams[it->first];
    type = static_cast<std::string>(stream["type"]);
    mountpoint = static_cast<std::string>(stream["mountpoint"]);
    source = static_cast<std::string>(stream["source"]);

    // Check which stream the client has connected to
    if (type == "topic" && url == mountpoint) {

      if (num_of_clients[url] == 0) {
        // Subscribe to the ROS topic
        ROS_WARN("Subscribing to %s", source.c_str());
        subs[url] = nh.subscribe<sensor_msgs::Image>(
            source, 1,
            boost::bind(&Image2RTSPNodelet::imageCallback, this,
                        boost::placeholders::_1, url));
      }
      num_of_clients[url]++;
    }
  }
}

void Image2RTSPNodelet::url_disconnected(string url) {
  std::string mountpoint;

  NODELET_INFO("Client disconnected: %s", url.c_str());
  ros::NodeHandle &nh = getPrivateNodeHandle();

  // Get the parameters from the rosparam server
  XmlRpc::XmlRpcValue streams;
  nh.getParam("streams", streams);
  ROS_ASSERT(streams.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  // Go through and parse each stream
  for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = streams.begin();
       it != streams.end(); ++it) {
    XmlRpc::XmlRpcValue stream = streams[it->first];
    mountpoint = static_cast<std::string>(stream["mountpoint"]);

    // Check which stream the client has disconnected from
    if (url == mountpoint) {
      if (num_of_clients[url] > 0)
        num_of_clients[url]--;
      if (num_of_clients[url] == 0) {
        // No-one else is connected. Stop the subscription.
        subs[url].shutdown();
        appsrc[url] = NULL;
      }
    }
  }
}

void Image2RTSPNodelet::print_info(char *s) { NODELET_INFO("%s", s); }

void Image2RTSPNodelet::print_error(char *s) { NODELET_ERROR("%s", s); }

PLUGINLIB_EXPORT_CLASS(image2rtsp::Image2RTSPNodelet, nodelet::Nodelet)
