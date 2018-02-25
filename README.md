# movo_hmi

A sample HMI for the MOVO robot.

Uses [ROSBridge](https://wiki.ros.org/rosbridge_suite) and
[ROSLibJS](https://github.com/RobotWebTools/roslibjs) to connect to various
HTML HMI displays.

## Requirements

You have to install rosbridge_suite before using this.

```
sudo apt-get install ros-<rosdistro>-rosbridge-server
```

Then clone this repo to your catkin workspace `src` folder.

```
cd ~/movo_ws/src
git clone https://github.com/aaronsnoswell/movo_hmi
```

## Usage

To run the ROSBridge server and gaze position publisher:

```
roslaunch movo_hmi movo_hmi
```

By default this will open a ROSBridge websocket server at `localhost:9090`,
which is what the html pages are set up to expect.

You can then open `www/*.html` in a browser (no need for a web server, just
double click the files) to view the demos. Available demos;

 * [`eyes.html`](www/eyes.html): Displays a set of blinking eyes.
   The gaze direction can be controlled from ROS by publishing a
   `geometry_msgs/Point` to "/gaze_direction" (the z value is ignored). If the
   [`gaze_publisher.py`](scripts/gaze_publisher.py) script is
   running, you should see MOVO rolling his eyes at you continuously.

 * [`star_rating.html`](www/star_rating.html): Gives the user 5
   star-rating buttons to provide feedback on the robot's behaviour. Publishes
   `sts_msgs/Float32` to "/star_rating". You can check it is working by
   running `rostopic echo /star_rating`. Intelligent robot learning and
   responses are left as an excerise for the reader.
