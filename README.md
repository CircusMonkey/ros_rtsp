# ros_rtsp
This is still very much a work in progress. Developing on a mix of Ubuntu 16.04 and 18.04.
ROS package to subscribe to an ROS Image topic and serve it up as a RTSP video feed.

## Dependencies
Ensure you have gstreamer development libs installed for your system.
```
sudo apt-get install libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev libgstreamer-plugins-bad1.0-dev libgstrtspserver-1.0-dev 
```

## Setup
Change the config/stream_setup.yaml to suit your required streams. There are 2 types to choose from: [cam, topic]. Cam should work with nearly any video source plugin available with gstreamer.
