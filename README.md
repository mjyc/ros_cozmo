# (WIP) ROS Cozmo

This package provides ROS interfaces for [Cozmo SDK](http://cozmosdk.anki.com/docs/). ROS Cozmo is tested on Ubuntu 16.04 with ROS Kinetic and Cozmo [SDK 1.3.2 / App 2.4.0](http://cozmosdk.anki.com/docs/sdk-versions.html). This package is inspired by the [cozmo_driver](https://github.com/OTL/cozmo_driver) package.

## Getting started

1. [Install ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu).
1. [Install Cozmo SDK](http://cozmosdk.anki.com/docs/).
1. Clone this repository to your catkin workspace and build it.
1. Install `rospkg catkin_pkg`:

    ```
    sudo apt-get install python3-yaml
    sudo pip3 install rospkg catkin_pkg
    ```

    * Thanks [OTL](https://github.com/OTL) for finding [this workaround](https://github.com/OTL/cozmo_driver#super-hack-to-run-rospy-from-python3)!
    * Note that [ROS Kinectic does not require python3](http://www.ros.org/reps/rep-0003.html#kinetic-kame-may-2016-may-2021).

1. [Start Cozmo SDK](http://cozmosdk.anki.com/docs/getstarted.html).
1. Run `roscore` in a terminal and run `rosrun ros_cozmo run_cozmo_node.py` in another terminal window.
1. Try starting an action with axclient, e.g., `rosrun actionlib axclient.py /say_text ros_cozmo/SayTextAction`.
