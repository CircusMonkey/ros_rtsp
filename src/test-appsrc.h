#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/app/gstappsrc.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

static gboolean
timeout(GstRTSPServer *server);

static void
media_constructed(GstRTSPMediaFactory *factory, GstRTSPMedia *media,
                gpointer user_data);

GstCaps* 
gst_caps_new_from_image(const sensor_msgs::Image::ConstPtr &msg);

GstSample* 
gst_sample_new_from_image(const sensor_msgs::Image::ConstPtr &msg);


int 
ros_main(int argc, char *argv[]);
