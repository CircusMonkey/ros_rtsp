# ros_rtsp
ROS package to subscribe to an ROS Image topic (and as many other video sources as you want) and serve it up as a RTSP video feed with different mount points.
Should provide a real-time video feed (or as close as possible).

This is still very much a work in progress. Developing on Ubuntu 16.04 and 18.04 with ROS kinetic and melodic.


## Dependencies
- ROS

- gstreamer libs:
```bash
sudo apt-get install libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev libgstreamer-plugins-bad1.0-dev libgstrtspserver-1.0-dev gstreamer1.0-plugins-ugly gstreamer1.0-plugins-bad
```

## Build into your catkin workspace
Navigate to your catkin workspace `src` folder. I.e. `cd ~/catkin_ws/src`.
Clone this package to the repository:
```bash
git clone https://github.com/CircusMonkey/ros_rtsp.git
```

Navigate back to the catkin workspace root and make the package:
```bash
cd ..
catkin_make pkg:=ros_rtsp
```

## Stream Setup
Change the `config/stream_setup.yaml` to suit your required streams.

```yaml
# Set up your streams for rtsp here.
streams:

  # Example v4l2 camera stream
  stream-1: # Can name this whatever you choose
    type: cam  # cam - Will not look in ROS for a image. The video src is set in the 'source' parameter.
    source: "v4l2src device=/dev/video0 ! videoconvert ! videoscale ! video/x-raw,framerate=15/1,width=1280,height=720"  # You can enter any valid gstreamer source and caps here as long as it ends in raw video
    mountpoint: /front      # Choose the mountpoint for the rtsp stream. This will be able to be accessed from rtsp://<server_ip>/front
    bitrate: 800            # bitrate for the h264 encoding.

  # Example ROS Image topic stream
  stream2:
    type: topic  # topic - Image is sourced from a sensor_msgs::Image topic
    source: /usb_cam0/image_raw  # The ROS topic to subscribe to
    mountpoint: /back      # Choose the mountpoint for the rtsp stream. This will be able to be accessed from rtsp://<server_ip>/back
    caps: video/x-raw,framerate=10/1,width=640,height=480  # Set the caps to be applied after getting the ROS Image and before the x265 encoder.
    bitrate: 500
```
Add as many streams as you require.

## Checking the streams
Launch the streams from the ROS launch file:
```bash
roslaunch ros_rtsp rtsp_streams.launch
```

In the following examples, replace the `rtsp://127.0.0.1:8554/front` with your servers IP address and mount point `rtsp://YOUR_IP:8554/MOUNT_POINT`.

### gstreamer
Use `gst-launch-1.0`. You will need to install gstreamer for your client system. See https://gstreamer.freedesktop.org/documentation/installing/index.html
```bash
gst-launch-1.0 -v rtspsrc location=rtsp://127.0.0.1:8554/front drop-on-latency=true use-pipeline-clock=true do-retransmission=false latency=0 protocols=GST_RTSP_LOWER_TRANS_UDP ! rtph264depay ! h264parse ! avdec_h264 ! autovideosink sync=true
```

### mpv
```bash
mpv --no-cache --untimed --no-demuxer-thread --vd-lavc-threads=1 rtsp://127.0.0.1:8554/front
```

### VLC
VLC adds way too much latency. Please don't use it for this purpose. If you want to try, this is the command that was the least slow (Let me know if you find a better command):
```bash
cvlc --no-audio --mux none --demux none --deinterlace 0 --no-autoscale --avcodec-hw=any --no-auto-preparse --sout-rtp-proto=udp --network-caching=300 --realrtsp-caching=0 --sout-udp-caching=0 --clock-jitter=0 --rtp-max-misorder=0 rtsp://127.0.0.1:8554/front :udp-timeout=0
```
If you wish to use the VLC mobile app to stream on Android or iOS, navigate to the network or streams menu and type in your server URL and mountpoint e.g. `rtsp://127.0.0.1:8554/front`


## Debugging
- If too much latency is encounted with multiple streams running, the server computer may be maxing out its processor trying to encode all the streams. Try reducing the resolution of the source caps.
- The ROS Image topic stream may be buggy with framerates too fast for the Image publisher and the buffer writing. Stick with 10/1 fps unless you want to debug? :)
- If too many frames are being dropped, it is likely due to network bandwidth. Try dropping the bitrate.
- If the ROS topic isn't available, you will get a `can't prepare media` error after a delay.
