# PR2 Programming by Demonstration (PbD)

This repository contains the work of [Maya Cakmak](http://www.mayacakmak.com/) and the [Human-Centered Robotics Lab](https://sites.google.com/site/humancenteredrobotics/) at the University of Washington. Please see those sites for citing publications.



## System Requirements
Currently the PbD system has the following requirements:

- Ubuntu 12.04
- ROS Groovy
- rosbuild

We are currently (this text written summer 2014) working on ROS Hydro and Catkin releases of the system.



## Installing

### ROS Software
- There are a ton of required ROS packages. Remember that currently, you'll need to be running Ubuntu 12.04 and ROS Groovy.

- As a start, install `ros-groovy-desktop-full` and `ros-groovy-pr2-full`. Once we have an easy comprehensive list, we'll post it here.

- As a note, [here is a list](https://gist.github.com/mbforbes/a1580f5434e35c597108) of packages you may need; if it's not running.

### PbD Code
- Assuming you've set up a rosbuild workspace in the location `~/rosbuild_ws`, and you've correctly `source`'d the correct ROS Groovy setup files, you do the following:

```bash
$ cd ~/rosbuild_ws  # or your rosbuild workspace
$ git clone git@github.com:PR2/pr2_pbd.git  # or your fork
$ cd pr2_pbd
$ rosmake  # generate message classes
```

Note that this needs to be done on **both** the PR2 _and_ your desktop machine.



## Running

### Prerequisites
Here we assume that, in addition to having ROS Groovy and your rosbuild workspace setup correctly, by default you have the following environment variables **on your desktop**:

```bash
$ export ROS_HOSTNAME=localhost
$ export ROS_MASTER_URI=http://localhost:11311
$ export ROBOT=sim
$ export MY_IP=$(/sbin/ifconfig eth0 | awk '/inet/ { print $2 } ' | sed -e s/addr://)
```

and that you have some way of pointing your desktop to look at the PR2 (`c1`) as the ROS master, such as this alias:

```bash
alias realrobot='unset ROBOT; unset ROS_HOSTNAME; export ROS_MASTER_URI=http://c1:11311; export ROS_IP=$MY_IP'
```

### On the PR2

#### Commands on PR2 (`c1`)
```bash
# Terminal 1: PbD backend
$ ssh <uname>@c1  # ssh into the PR2
$ robot stop; robot claim; robot start  # gets the robot
# Before continuing, complete Terminal 1 on desktop to ensure PR2 ready.
$ roslaunch pr2_pbd_interaction pbd_demo_robot.launch  # run PbD backend
```
#### Commands on desktop
```bash
# Terminal 1: PR2 dashboard
$ realrobot  # see 'Prerequisites' section above
$ rosrun rqt_pr2_dashboard rqt_pr2_dashboard  # dashboard to monitor PR2
# Make sure both runstops are OK (far right icon), and motors OK (red gear icon)

# Terminal 2: PbD frontend
$ realrobot  # see 'Prerequisites' section above
$ roslaunch pr2_pbd_interaction pbd_demo_desktop.launch
```

### On desktop only (simulation)

#### Commands on desktop

```bash
# Terminal 1: Robot simulation with Gazebo
$ roslaunch pr2_pbd_interaction simulated_robot.launch

# Terminal 2: PbD backend. Note sim:=true for simulated Kinect.
$ roslaunch pr2_pbd_interaction pbd_demo_robot.launch sim:=true

# Terminal 3: PbD frontend
$ roslaunch pr2_pbd_interaction pbd_demo_desktop.launch
```

## Questions, feedback, improvements
Please use the Github issues page for this project.
