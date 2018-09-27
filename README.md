# ros_rtsp
ROS package to subscribe to an ROS Image topic (and as many other video sources as you want) and serve it up as a RTSP video feed with different mount points.

This is still very much a work in progress. Developing on a mix of Ubuntu 16.04 and 18.04.


## Dependencies
- ROS

- gstreamer development libs:
```
sudo apt-get install libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev libgstreamer-plugins-bad1.0-dev libgstrtspserver-1.0-dev 
```

- gstreamer: For linux (https://gstreamer.freedesktop.org/documentation/installing/on-linux.html)
```
apt-get install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools
```
For other platforms: https://gstreamer.freedesktop.org/documentation/installing/index.html

## Stream Setup
Change the config/stream_setup.yaml to suit your required streams.

```
# Set up your streams to rtsp here.
streams: # Cannot rename - must leave this as is.

  # Example v4l2 camera stream
  stream-x:                 # Can name this whatever you choose
    type: cam               # cam - Will not look in ROS for a image. The video src is set in the 'source' parameter.
    source: "v4l2src device=/dev/video0 ! videoconvert ! videoscale ! video/x-raw,framerate=15/1,width=1280,height=720"  # Should work with most valid gstreamer piplines (ending with raw video) 
    mountpoint: /front      # Choose the mountpoint for the rtsp stream. This will be able to be accessed from rtsp://<server_ip>/front
    bitrate: 800            # bitrate for the h264 encoding.

  # Example ROS Image topic stream
  stream-42:                # Can name this whatever you choose
    type: topic             # topic - Image is sourced from a sensor_msgs::Image topic
    source: /usb_cam0/image_raw  # The ROS topic to subscribe to
    mountpoint: /back      # Choose the mountpoint for the rtsp stream. This will be able to be accessed from rtsp://<server_ip>/back
    bitrate: 500            # bitrate for the h264 encoding.
```

## Checking the streams
The best way to check a stream is working is to use gst-launch-1.0 as follows:

```
gst-launch-1.0 -v rtspsrc location=rtsp://<server_ip>:8554/<your_mount_point> drop-on-latency=true use-pipeline-clock=true do-retransmission=false latency=0 protocols=GST_RTSP_LOWER_TRANS_UDP ! rtph264depay ! h264parse ! avdec_h264 ! xvimagesink sync=true
```