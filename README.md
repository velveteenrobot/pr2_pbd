# PR2 Programming by Demonstration
[![Coverage Status](https://img.shields.io/coveralls/mbforbes/pr2_pbd.svg)](https://coveralls.io/r/mbforbes/pr2_pbd?branch=style)

This repository contains the work of [Maya Cakmak](http://www.mayacakmak.com/) and the [Human-Centered Robotics Lab](https://sites.google.com/site/humancenteredrobotics/) at the University of Washington. Please see those sites for citing publications. We abbreviate Programming by Demonstration with PbD.



## System Requirements
Currently the PbD system has the following requirements:

- Ubuntu 12.04
- ROS Groovy
- rosbuild
- Python 2.7

We are currently (this text written summer 2014) working on ROS Hydro and Catkin releases of the system.



## Installing

### ROS Software
- [Install ROS Groovy](http://wiki.ros.org/groovy/Installation/Ubuntu) (ros-groovy-desktop-full) on your Ubuntu 12.04 (precise) machine.

- Use apt-get to install the following ROS packages:
	- `ros-groovy-pr2-desktop`
	- `ros-groovy-interactive-manipulation`
	- `ros-groovy-simulator-gazebo`
	- `ros-groovy-pr2-simulator`
	- `ros-groovy-openni-launch`

- As a note, [here is a list](https://gist.github.com/mbforbes/a1580f5434e35c597108) of all packages you may need on your desktop; if it's not running, check against that list.

### PbD Code
Assuming you've set up a rosbuild workspace in the location `~/rosbuild_ws`, and you've correctly `source`'d the correct ROS Groovy setup files, you do the following:

```bash
$ cd ~/rosbuild_ws  # or your rosbuild workspace
$ git clone git@github.com:PR2/pr2_pbd.git  # or your fork
$ cd pr2_pbd
$ rosmake  # generate message classes
```

Note that this needs to be done on **both** the PR2 _and_ your desktop machine.

### Python packages
If you don't have Python, install Python 2.7
```bash
$ sudo apt-get install python2.7
```

If you don't have pip, install pip
```bash
$ sudo apt-get install python-pip
```

The required pip packages are listed in `requirements.txt`. From the root of the repository, run:
```bash
$ pip install -r requirements.txt --use-mirrors
```



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
$ alias realrobot='unset ROBOT; unset ROS_HOSTNAME; export ROS_MASTER_URI=http://c1:11311; export ROS_IP=$MY_IP'
```

### On the PR2

#### Commands on PR2 (`c1`)
```bash
# Terminal: PbD backend
$ ssh <uname>@c1
$ robot claim; robot stop; robot start  # gets the robot
# Before continuing, complete Terminal 1 on desktop to ensure the PR2 is ready.
$ roslaunch pr2_pbd_interaction pbd_backend.launch
```
#### Commands on desktop
```bash
# Terminal 1: PR2 dashboard
$ realrobot  # see 'Prerequisites' section above
$ rosrun rqt_pr2_dashboard rqt_pr2_dashboard  # dashboard to monitor PR2
# Make sure both runstops are OK (far right icon), and motors OK (red gear icon)

# Terminal 2: PbD frontend
$ realrobot  # see 'Prerequisites' section above
$ roslaunch pr2_pbd_interaction pbd_frontend.launch
```

### On desktop only (simulation)

#### Commands on desktop

```bash
# Terminal: Robot simulation with Gazebo, PbD backend, PbD frontend
$ roslaunch pr2_pbd_interaction pbd_simulation_stack.launch
```



## Tests
Run the tests with `rostest pr2_pbd_interaction test_endtoend.test`. View code coverage by opening `~/.ros/htmlcov/index.html` with a web broswer. With an acout setup at [Coveralls](https://coveralls.io), edit the `.coveralls.yml` with your repo_token, and track coverage there by running `coveralls --data_file ~/.ros/.coverage`.

To do all of these steps automatically (opening with Google Chrome, assuming Coveralls and `.coveralls.yml` setup), we have provided a script:
```bash
$ roscd pr2_pbd_interaction; ./scripts/test_and_coverage.sh
```



## Questions, feedback, improvements
Please use the Github issues page for this project.
