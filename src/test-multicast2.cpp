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

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/rtsp-server/rtsp-server.h>


static gboolean
timeout(GstRTSPServer *server)
{
    GstRTSPSessionPool *pool;

    pool = gst_rtsp_server_get_session_pool(server);
    gst_rtsp_session_pool_cleanup(pool);
    g_object_unref(pool);

    return TRUE;
}

static void
media_constructed(GstRTSPMediaFactory *factory, GstRTSPMedia *media)
{
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
}



/* called when there is a new buffer ready for
 * processing */
static void 
wrapper_imageCallback(void* pt2Object, const sensor_msgs::ImageConstPtr& msg)
{
    myImages* mySelf = (myImages*) pt2Object; // explicitly cast to a pointer to class myImages
    mySelf->imageCallback(msg );          // call member
}


static GstFlowReturn
imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    GstSample *sample;
    GstFlowReturn ret;

    GstCaps *caps;
	char *gst_type, *gst_format = (char *)"";

    GstBuffer *buf;
	void *imgdata;
	GstMapInfo map;
	static GstClockTime timestamp=0;

    appsrc_front = gst_bin_get_by_name(GST_BIN(data->sink), "appsrc_front");

	if (appsrc_front != NULL) 
    {
		// Set caps from message
		get_format(msg->encoding.c_str(), &gst_type, &gst_format);
		caps = gst_caps_new_simple (gst_type,
				"format", G_TYPE_STRING, gst_format,
				"width", G_TYPE_INT, msg->width,
				"height", G_TYPE_INT, msg->height,
				NULL);
		gst_app_src_set_caps(appsrc_front, caps);

		buf = gst_buffer_new_and_alloc(msg->step*msg->height);

		gst_buffer_map(buf, &map, GST_MAP_READ);
		imgdata = map.data;

		GST_BUFFER_PTS(buf) = timestamp;
		GST_BUFFER_DURATION(buf) = gst_util_uint64_scale_int(1, GST_SECOND, 15);
		timestamp += GST_BUFFER_DURATION(buf);


		memcpy(imgdata, &msg->data[0], msg->step*msg->height);

		gst_buffer_unmap(buf, &map);
		gst_app_src_push_buffer(appsrc_rgb, buf);
	}



    /* get the sample from appsink */
    sample = msg; //Get from ROS

    /* get source an push new sample */
    source = gst_bin_get_by_name(GST_BIN(source), "appsrc_front");
    ret = gst_app_src_push_sample(GST_APP_SRC(source), sample);
    gst_object_unref(source);

    return ret;
}



int main(int argc, char **argv)
{

    std::thread rosthr(ros_main)

    GMainLoop *loop;
    GstRTSPServer *server;
    GstRTSPMountPoints *mounts;
    GstRTSPMediaFactory *factory_front, *factory_back;

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
    factory_front = gst_rtsp_media_factory_new();
    gst_rtsp_media_factory_set_launch(factory_front, "( "
        "appsrc name=appsrc_front ! videoconvert ! videoscale ! video/x-raw,framerate=15/1,width=1280,height=720 ! "
        "x264enc tune=zerolatency bitrate=1000 key-int-max=90 ! "
        "rtph264pay name=pay0 pt=96 "
        ")");

    gst_rtsp_media_factory_set_shared(factory_front, TRUE);

    g_signal_connect(factory_front, "media-constructed", (GCallback)media_constructed, NULL);

    /* attach the test factory to the /test url */
    gst_rtsp_mount_points_add_factory(mounts, "/front", factory_front);

    /* don't need the ref to the mapper anymore */
    g_object_unref(mounts);

    /* attach the server to the default maincontext */
    if (gst_rtsp_server_attach(server, NULL) == 0)
    {
        g_print("failed to attach the server\n");
        return -1;
    }

    g_timeout_add_seconds(2, (GSourceFunc)timeout, server);

    /* start serving */
    g_print("stream ready at rtsp://127.0.0.1:8554/front\n");

    // Use the ros::spin main loop instead.
    // g_main_loop_run(loop);

    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
    ros::spin();

    return 0;
}
