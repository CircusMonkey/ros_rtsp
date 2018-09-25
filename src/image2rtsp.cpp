#include <string>
#include <stdio.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <image2rtsp.h>


using namespace std;
using namespace image2rtsp;


void Image2RTSPNodelet::onInit() {
	string pipeline_cam = "( appsrc name=imagesrc do-timestamp=true max-latency=50 max-bytes=1000 is-live=true ! videoconvert ! videoscale ! video/x-raw,width=800,height=640 ! videoconvert ! videorate ! video/x-raw,framerate=10/1 ! queue ! x264enc tune=zerolatency bitrate=1000 key-int-max=90 ! video/x-h264, profile=baseline ! rtph264pay name=pay0 pt=96 )";

	string pipeline_appsrc = "( appsrc name=imagesrc do-timestamp=true max-latency=50 max-bytes=1000 is-live=true ! videoconvert ! videoscale ! video/x-raw,width=800,height=640 ! videoconvert ! videorate ! video/x-raw,framerate=10/1 ! queue ! x264enc tune=zerolatency bitrate=1000 key-int-max=90 ! video/x-h264, profile=baseline ! rtph264pay name=pay0 pt=96 )";

	NODELET_DEBUG("Initializing image2rtsp nodelet...");

	if (getenv((char*)"GST_DEBUG") == NULL) {
		// set GST_DEBUG to warning if unset
		putenv((char*)"GST_DEBUG=*:2");
	}

	num = 0;
	appsrc = NULL;
	ros::NodeHandle& nh = getPrivateNodeHandle();

	video_mainloop_start();
	rtsp_server = rtsp_server_create();

	XmlRpc::XmlRpcValue streams;
	nh.getParam("streams", streams);
	ROS_ASSERT(streams.getType() == XmlRpc::XmlRpcValue::TypeStruct);

	ROS_INFO("Number of streams: %d", streams.size());

	for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = streams.begin(); it != streams.end(); ++it) 
	{
    	ROS_INFO_STREAM("Found stream: " << (std::string)(it->first) << " ==> " << streams[it->first]);

		if (streams[it->first]["type"]=="cam")
		{
			ROS_INFO("ADDING CAMERA");
			rtsp_server_add_url(static_cast<std::string>(streams[it->first]["mountpoint"]).c_str(), pipeline_cam.c_str(), NULL);
		} 
		else if (streams[it->first]["type"]=="topic")
		{
			ROS_INFO("ADDING topic");
			rtsp_server_add_url(static_cast<std::string>(streams[it->first]["mountpoint"]).c_str(), pipeline_appsrc.c_str(), (GstElement **)&appsrc);
		}

	}


	
}

/* Borrowed from https://github.com/ProjectArtemis/gst_video_server/blob/master/src/server_nodelet.cpp */
GstCaps* Image2RTSPNodelet::gst_caps_new_from_image(const sensor_msgs::Image::ConstPtr &msg)
{
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

	return gst_caps_new_simple("video/x-raw",
			"format", G_TYPE_STRING, format->second.c_str(),
			"width", G_TYPE_INT, msg->width,
			"height", G_TYPE_INT, msg->height,
			"framerate", GST_TYPE_FRACTION, 10, 1,	// 0/1 = dynamic
			nullptr);
}

void Image2RTSPNodelet::imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
	GstBuffer *buf;
	void *imgdata;
	GstMapInfo map;
	static GstClockTime timestamp=0;

	GstCaps *caps;
	char *gst_type, *gst_format=(char *)"";

	g_print("Image encoding: %s\n", msg->encoding.c_str());

	if (appsrc != NULL) {
		// Set caps from message
		caps = gst_caps_new_from_image(msg);
		gst_app_src_set_caps(appsrc, caps);

		buf = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
		gst_buffer_fill(buf, 0, msg->data.data(), msg->data.size());
		GST_BUFFER_FLAG_SET(buf, GST_BUFFER_FLAG_LIVE);

		// gst_buffer_map(buf, &map, GST_MAP_READ);
		// imgdata = map.data;

		// GST_BUFFER_PTS(buf) = timestamp;
		// GST_BUFFER_DURATION(buf) = gst_util_uint64_scale_int(1, GST_SECOND, 15);
		// timestamp += GST_BUFFER_DURATION(buf);

		// memcpy(imgdata, &msg->data[0], msg->step*msg->height);

		// gst_buffer_unmap(buf, &map);
		gst_app_src_push_buffer(appsrc, buf);
	}


	// /* Create the sample from the ros msg */
	// GstFlowReturn ret;
	// // unfortunately we may not move image data because it is immutable. copying.
	// auto buffer = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
	// g_assert(buffer);

	// /* NOTE(vooon):
	//  * I found that best is to use `do-timestamp=true` parameter
	//  * and forgot about stamping stamp problem (PTS).
	//  */
	// //auto ts = gst_time_from_stamp(msg->header.stamp);
	// //NODELET_INFO("TS: %lld, %lld", ts, gst_util_uint64_scale_int(1, GST_SECOND, 40));

	// gst_buffer_fill(buffer, 0, msg->data.data(), msg->data.size());
	// GST_BUFFER_FLAG_SET(buffer, GST_BUFFER_FLAG_LIVE);
	// //GST_BUFFER_PTS(buffer) = ts;

	// auto caps = gst_caps_new_from_image(msg);
    // g_print("Got caps from msg\n");
	// if (caps == nullptr) {
	// 	gst_object_unref(GST_OBJECT(buffer));
	// 	return;
	// }

	// auto sample = gst_sample_new(buffer, caps, nullptr, nullptr);
    // g_print("Assigned a sample\n");
	// gst_buffer_unref(buffer);
	// gst_caps_unref(caps);

	// /* Get the pipeline to push to */
	// ret = gst_app_src_push_sample (GST_APP_SRC (appsrc), sample);
	// gst_object_unref (sample);
}


void Image2RTSPNodelet::url_connected(string url) {
	string topic;

	NODELET_INFO("Client connected: %s", url.c_str());

	if (url == "/front") {
		if (num == 0) {
			ros::NodeHandle& nh = getPrivateNodeHandle();
			nh.getParam("topic_1", topic);
			sub = nh.subscribe(topic, 10, &Image2RTSPNodelet::imageCallback, this);
		}
		num++;
	}
}

void Image2RTSPNodelet::url_disconnected(string url) {
	NODELET_INFO("Client disconnected: %s", url.c_str());

	if (url == "/front") {
		if (num > 0) num--;
		if (num == 0) {
			sub.shutdown();
			appsrc = NULL;
		}
	}
}

void Image2RTSPNodelet::print_info(char *s) {
	NODELET_INFO(s);
}

void Image2RTSPNodelet::print_error(char *s) {
	NODELET_ERROR(s);
}

PLUGINLIB_EXPORT_CLASS(image2rtsp::Image2RTSPNodelet, nodelet::Nodelet)
