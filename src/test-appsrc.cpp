/* GStreamer
 * Copyright (C) 2008 Wim Taymans <wim.taymans at gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 * Boston, MA 02110-1301, USA.
 */

#include <string>
#include <thread>
#include "test-appsrc.h"


GstElement *appsrc;
GstBuffer *buffer;



static gboolean
timeout(GstRTSPServer *server)
{
    GstRTSPSessionPool *pool;

    pool = gst_rtsp_server_get_session_pool(server);
    gst_rtsp_session_pool_cleanup(pool);
    g_object_unref(pool);

    return TRUE;
}

/* called when we need to give data to appsrc */
static void
need_data (GstElement * appsrc, guint unused)
{
    g_print("NEED DATA. Source is dry.");

}


/* called when a new media pipeline is constructed. We can query the
 * pipeline and configure our appsrc */
static void
media_constructed(GstRTSPMediaFactory *factory, GstRTSPMedia *media,
                gpointer user_data)
{
    GstElement *element;
    g_print ("YAY MEDIA BEING CONSTRUCTED!!!\n");

    /* Setup for multicasting */
    guint i, n_streams;

    n_streams = gst_rtsp_media_n_streams(media);

    for (i = 0; i < n_streams; i++)
    {
        GstRTSPAddressPool *pool;
        GstRTSPStream *stream;
        gchar *min, *max;

        stream = gst_rtsp_media_get_stream(media, i);

        /* make a new address pool */
        pool = gst_rtsp_address_pool_new();

        min = g_strdup_printf("224.3.0.%d", (2 * i) + 1);
        max = g_strdup_printf("224.3.0.%d", (2 * i) + 2);
        gst_rtsp_address_pool_add_range(pool, min, max,
                                        5000 + (10 * i), 5010 + (10 * i), 1);
        g_free(min);
        g_free(max);

        gst_rtsp_stream_set_address_pool(stream, pool);
        g_object_unref(pool);
    }


    /* get the element used for providing the streams of the media */
    element = gst_rtsp_media_get_element (media);

    /* get our appsrc, we named it 'mysrc' with the name property */
    appsrc = gst_bin_get_by_name_recurse_up (GST_BIN (element), "mysrc");

    /* this instructs appsrc that we will be dealing with timed buffer */
    gst_util_set_object_arg (G_OBJECT (appsrc), "format", "time");

    g_signal_connect (appsrc, "need-data", (GCallback) need_data, NULL);


}


/* Borrowed from https://github.com/ProjectArtemis/gst_video_server/blob/master/src/server_nodelet.cpp */
namespace enc = sensor_msgs::image_encodings;

GstCaps* gst_caps_new_from_image(const sensor_msgs::Image::ConstPtr &msg)
{
	// http://gstreamer.freedesktop.org/data/doc/gstreamer/head/pwg/html/section-types-definitions.html
	static const ros::M_string known_formats = {{
		{enc::RGB8, "RGB"},
		{enc::RGB16, "RGB16"},
		{enc::RGBA8, "RGBA"},
		{enc::RGBA16, "RGBA16"},
		{enc::BGR8, "BGR"},
		{enc::BGR16, "BGR16"},
		{enc::BGRA8, "BGRA"},
		{enc::BGRA16, "BGRA16"},
		{enc::MONO8, "GRAY8"},
		{enc::MONO16, "GRAY16_LE"},
	}};

	if (msg->is_bigendian) {
		ROS_ERROR("GST: big endian image format is not supported");
		return nullptr;
	}

	auto format = known_formats.find(msg->encoding);
    g_print("Format found = %s\n", format);
	if (format == known_formats.end()) {
		ROS_ERROR("GST: image format '%s' unknown", msg->encoding.c_str());
		return nullptr;
	}

	return gst_caps_new_simple("video/x-raw",
			"format", G_TYPE_STRING, format->second.c_str(),
			"width", G_TYPE_INT, msg->width,
			"height", G_TYPE_INT, msg->height,
			"framerate", GST_TYPE_FRACTION, 0, 1,	// 0/1 = dynamic
			nullptr);
}

/* Borrowed from https://github.com/ProjectArtemis/gst_video_server/blob/master/src/server_nodelet.cpp */
GstSample* gst_sample_new_from_image(const sensor_msgs::Image::ConstPtr &msg)
{
    g_print("Image details: %s\n", msg->encoding.c_str());


	// unfortunately we may not move image data because it is immutable. copying.
	auto buffer = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
	g_assert(buffer);

	/* NOTE(vooon):
	 * I found that best is to use `do-timestamp=true` parameter
	 * and forgot about stamping stamp problem (PTS).
	 */
	//auto ts = gst_time_from_stamp(msg->header.stamp);
	//NODELET_INFO("TS: %lld, %lld", ts, gst_util_uint64_scale_int(1, GST_SECOND, 40));

	gst_buffer_fill(buffer, 0, msg->data.data(), msg->data.size());
	GST_BUFFER_FLAG_SET(buffer, GST_BUFFER_FLAG_LIVE);
	//GST_BUFFER_PTS(buffer) = ts;

	auto caps = gst_caps_new_from_image(msg);
    g_print("Got caps from msg\n");
	if (caps == nullptr) {
		gst_object_unref(GST_OBJECT(buffer));
		return nullptr;
	}

	auto sample = gst_sample_new(buffer, caps, nullptr, nullptr);
    g_print("Assigned a sample\n");
	gst_buffer_unref(buffer);
	gst_caps_unref(caps);

	return sample;
}



void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    GstSample *sample;
    GstFlowReturn ret;

    g_print("Got new ROS image callback\n");
    /* Create the sample from the ros msg */
    sample = gst_sample_new_from_image(msg);

    /* Get the pipeline to push to */
    ret = gst_app_src_push_sample (GST_APP_SRC (appsrc), sample);
    gst_object_unref (sample);

}



int main(int argc, char *argv[])
{
    GMainLoop *loop;
    GstRTSPServer *server;
    GstRTSPMountPoints *mounts;
    GstRTSPMediaFactory *factory;

    gst_init(&argc, &argv);

    loop = g_main_loop_new(NULL, FALSE);

    /* create a server instance */
    server = gst_rtsp_server_new();

    /* get the mount points for this server, every server has a default object
    * that be used to map uri mount points to media factories */
    mounts = gst_rtsp_server_get_mount_points(server);

    /* make a media factory for a test stream. The default media factory can use
    * gst-launch syntax to create pipelines.
    * any launch line works as long as it contains elements named pay%d. Each
    * element with pay%d names will be a stream */
    factory = gst_rtsp_media_factory_new();
    gst_rtsp_media_factory_set_launch(factory, "( "
        "appsrc name=mysrc ! videoconvert ! videoscale ! video/x-raw,framerate=15/1,width=1280,height=720 ! videoconvert ! "
        "x264enc tune=zerolatency bitrate=1000 key-int-max=90 ! "
        "rtph264pay name=pay0 pt=96"
        " )");

    gst_rtsp_media_factory_set_shared(factory, TRUE);

    /* notify when our media is ready, This is called whenever someone asks for
    * the media and a new pipeline with our appsrc is created */
    g_signal_connect(factory, "media-constructed", (GCallback)media_constructed,
                     NULL);

    /* attach the test factory to the /test url */
    gst_rtsp_mount_points_add_factory(mounts, "/test", factory);

    /* don't need the ref to the mounts anymore */
    g_object_unref(mounts);

    /* attach the server to the default maincontext */
    if (gst_rtsp_server_attach(server, NULL) == 0)
    {
        g_print("failed to attach the server\n");
        return -1;
    }
    g_timeout_add_seconds(2, (GSourceFunc)timeout, server);

    /* start serving */
    g_print("stream ready at rtsp://127.0.0.1:8554/test\n");


    /* Initialise the ros subscriber node */
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/image", 1,  imageCallback);
    
    /* Spin up ROS, but use the non-blocking spinner so
    * our g_main_loop can still listen for RTSP connections */
    ros::AsyncSpinner spinner(1);
    spinner.start();

    g_main_loop_run(loop);

    spinner.stop();

    return 0;

}
