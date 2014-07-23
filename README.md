# PR2 Programming by Demonstration
[![Build Status](https://travis-ci.org/PR2/pr2_pbd.svg?branch=groovy-devel)](https://travis-ci.org/PR2/pr2_pbd)
[![Coverage Status](https://coveralls.io/repos/PR2/pr2_pbd/badge.png?branch=groovy-devel)](https://coveralls.io/r/PR2/pr2_pbd?branch=groovy-devel)

This repository contains the work of [Maya Cakmak](http://www.mayacakmak.com/) and the [Human-Centered Robotics Lab](https://sites.google.com/site/humancenteredrobotics/) at the University of Washington. Please see those sites for citing publications. We abbreviate Programming by Demonstration with PbD.



## System Requirements
Currently the PbD system has the following requirements:

- Ubuntu 12.04
- ROS Groovy
- rosbuild
- Python 2.7

We are currently (this text written summer 2014) working on ROS Hydro and Catkin releases of the system.



## Installing
Note: Unless otherwise stated, all commands should be run from the root of this repository.

If you go through these instructions and your installation is not working, check out the `install:` section of our `.travis.yml` build script. It contains all commands needed to get up-and-running from a barebones Ubuntu 12.04 installation.

### Ubuntu setup
An up-to-date Ubuntu 12.04 machine should do fine, but just to make sure that you've got everything you need, run the following commands:
```bash
# Update software list and install new versions.
$ sudo apt-get update
$ sudo apt-get upgrade

# Install python requirements
$ sudo apt-get install python2.7 python-pip
```

### ROS software
[Install ROS Groovy](http://wiki.ros.org/groovy/Installation/Ubuntu) (pick `ros-groovy-desktop-full`) on your Ubuntu 12.04 (precise) machine. Make sure you follow all steps; we have omitted here everything contained in that guide.

Additional required debian packages are listed in `packages.txt` and can be installed with apt-get:

```bash
$ yes | sudo apt-get -y install $(< packages.txt)
```

### ROS setup

#### Both PR2 (`c1`) and desktop
First, you need to setup a `rosbuild` workspace.

```bash
$ mkdir -p ~/rosbuild_ws  # Create your rosbuild workspace
$ cd ~/rosbuild_ws && rosws init . /opt/ros/groovy  # Enter it and initialize
```

#### PR2 (`c1`) specific
You need to set environment variables. In addition to running these commands, you should put them in your `~/.bashrc` file so terminals automatically run them.

```bash
$ source /opt/ros/groovy/setup.sh
$ source ~/rosbuild_ws/setup.bash
$ export ROS_ENV_LOADER=/etc/ros/groovy/env.sh
$ export ROS_PACKAGE_PATH=~/rosbuild_ws:$ROS_PACKAGE_PATH
```

#### Desktop specific
You need to set environment variables. In addition to running these commands, you should put them in your `~/.bashrc` file so terminals automatically run them.

```bash
$ source ~/rosbuild_ws/setup.bash
$ export ROS_PACKAGE_PATH=~/rosbuild_ws/:$ROS_PACKAGE_PATH
$ export ROS_HOSTNAME=localhost
$ export ROS_MASTER_URI=http://localhost:11311
$ export ROBOT=sim
$ export MY_IP=$(/sbin/ifconfig eth0 | awk '/inet/ { print $2 } ' | sed -e s/addr://)
```

Finally, for running on the PR2, you need some way of pointing your desktop to look at the PR2 (`c1`) as the ROS master. Put this alias in your `~/.bashrc` or `~/.bash_aliases`:

```bash
$ alias realrobot='unset ROBOT; unset ROS_HOSTNAME; export ROS_MASTER_URI=http://c1:11311; export ROS_IP=$MY_IP'
```

### Python setup
The required python packages are listed in `requirements.txt` and can be installed with pip:

```bash
$ sudo pip install -r requirements.txt
```
Note that if you use python with virtualenv, you want it to still find the apt-get installed python packages, as not all are available or work property with pip. Do this by starting virtualenv with: `virtualenv --system-site-packages ENV` ([more info](http://virtualenv.readthedocs.org/en/latest/virtualenv.html#the-system-site-packages-option)).

### PbD code
Do the following to checkout and make the PBD code:

```bash
$ cd ~/rosbuild_ws  # or your rosbuild workspace
$ git clone git@github.com:PR2/pr2_pbd.git  # or your fork
$ cd pr2_pbd && rosmake  # generate message classes
```

Note that this needs to be done on **both** the PR2 _and_ your desktop machine.



## Running

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
$ realrobot  # points ROS to PR2
$ rosrun rqt_pr2_dashboard rqt_pr2_dashboard  # dashboard to monitor PR2
# Make sure both runstops are OK (far right icon), and motors OK (gear icon)

# Terminal 2: PbD frontend
$ realrobot  # points ROS to PR2
$ roslaunch pr2_pbd_interaction pbd_frontend.launch  # rviz, rqt, speech
```

### On desktop only (simulation)

#### Commands on desktop

```bash
# Terminal: Robot simulation with Gazebo, PbD backend, PbD frontend
$ roslaunch pr2_pbd_interaction pbd_simulation_stack.launch
```



## Tests
### Desktop
```bash
$ rostest pr2_pbd_interaction test_endtoend.test
```

### PR2
First, launch everything from the [Running On the PR2 section](#on-the-pr2) (above), _except_ for Terminal 2 on the desktop (`pbd_frontend`). Instead, run the following in a new terminal on `c1`:
```bash
$ roscd pr2_pbd_interaction
$ python test/test_endtoend_realrobot.py
```

### Code coverage
After running the tests, you can view code coverage by opening `~/.ros/htmlcov/index.html` with a web browser. Note that you can also view code coverage for normal execution by passing `coverage:=true` when launching `pr2_pbd_backend`.

With an acout setup at [Coveralls](https://coveralls.io), edit the `.coveralls.yml` with your repo_token, and track coverage there by running `coveralls --data_file ~/.ros/.coverage`.



## Contributing
Before creating a pull request, please do the following things:

0. Lint your Python to pep8 standards by running `pep8 file1 file2 ...`.

0. Run the tests on your desktop (see above).

To lint all python files in common directories, run the tests on the desktop, open up code coverage with Google Chrome, and send the results to Coveralls (assuming Coveralls account and `.coveralls.yml` correctly setup), we have provided a script:
```bash
$ roscd pr2_pbd_interaction; ./scripts/test_and_coverage.sh
```



## Questions, feedback, improvements
Please use the Github issues page for this project.
