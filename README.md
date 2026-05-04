# MIRTE ROS2 Workspace for RO47007 with Pixi

## status

This repository is still work in progress and has only been tested under Linux and macOS (M processors), but no guarantees.

## how to use this repository

This repository serves as your ROS2 workspace for your robot development with Pixi allowing you to seamlessly set up ROS2 on different machines and OS. The key advantage of [Pixi](https://pixi.prefix.dev/latest/) is a cross-system setup of ROS and using a manifest file to specify the exact configuration of your ROS2 setup for reproducibility. To this end, Pixi uses a `pixi.toml` manifest file that specifies what packages, dependencies, and commands should be installed. Fork this repository and use it as your main ROS development workspace. Every additional ROS package you build should be located in a separate repository. Within this ROS2 Pixi environment, you can add this repository in the file `repos.repos` that automatically clones your specified packages for your workspace. The advantage here is that you keep your overall ROS2 system separate from individual packages you develop and to have a single ROS2 WS configuration that can be run and maintained by your whole team. Note: we recommend you to use merge request for your ROS2 WS repository to ensure that your team has a single configuration to work with and to test new configurations before merging them into your main development.

## building

If you don't have Pixi, install it: [Pixi: Installation](https://pixi.prefix.dev/latest/installation/).

Then, in a terminal (this shows the Linux instructions, macOS would be similar, Windows: you're on your own):

```shell
# clone this repository somewhere. Here I do it in $HOME, but that is not
# required. The workspace could be anywhere
git clone https://gitlab.tudelft.nl/cor/ro47007/ro47007_mirte_ws.git $HOME/ro47007_mirte_ws
cd $HOME/ro47007_mirte_ws

# let pixi do its thing (this could take a while)
pixi install
```

if there were no errors, try fetching the packages (or `pixi run fetch` from within `ro47007_mirte_ws`):

```shell
pixi run vcs import --input $HOME/ro47007_mirte_ws/repos.repos $HOME/ro47007_mirte_ws/src
```

No errors? Ignore some unneeded packages:

```shell
touch src/mirte-ros-packages/mirte_{bringup,telemetrix_cpp,teleop,test,zenoh_setup}/COLCON_IGNORE
```

No errors? Try building the workspace (I prefer using `pixi shell` for this, but `pixi run build` would likely also work):

```shell
pixi shell
colcon build
```

note: depending on the machine this runs on, it can take quite some time.

If you want to clean the build artifacts (build, install, log directories) from within the pixi shell, you can use:
```shell
pixi run ws-clean
```

To clean first and then build, use:
```shell
pixi run clean-build
```

## running

If the build was successful, try starting the main launch file (in the same `pixi shell` session):

```shell
source install/setup.bash
ros2 launch mirte_gazebo gazebo_mirte_master_empty.launch.xml
```

this should start up only Gazebo (Classic) with the MIRTE Master in an empty world.

To teleop the robot, try starting the teleop node (from another `pixi shell` session):
```shell
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/mirte_base_controller/cmd_vel_unstamped
```

## working with the physical robot

Working with a physical MIRTE Master robot and your pixi env is straightforward via ROS2. First, connect to the robot either via the Wifi AP or an ethernet cable. SSH into the robot and set the ROS domain id (0-101) and restart ROS:
```shell
export ROS_DOMAIN_ID=1
sudo service mirte-ros restart
```

In your `pixi shell`, set the same id and turn off the localhost setting:
```shell
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=0
ros2 daemon stop
ros2 daemon start
```

You should now be able to see the topics of the robot:
```shell
ros2 topic list
```

note: every new pixi session requires you to set the domain id.


## troubleshooting

* **problems with Python when building**: Make sure that you fully deactivate Anaconda or other virtual Python environment managers when trying to set up this repository. The symbolic links may interfere with Pixi and cause problems when compiling your workspace.

* **Shells other than Bash** If you are using shells other than Bash, e.g., ZSH, make sure to source the correct ROS2 setup file in the folder `install`, e.g., `setup.zsh`.

* **Binaries blocked on macOS** The new security features of macOS do not allow you to run Pixi installed libraries immediately, e.g., CMake. After receiving the error message, you can allow the execution by going to `System Settings` -> `Privacy & Security` and clicking allow under the Security section for the corresponding binary. Note: you might need to repeat this process multiple times for various binaries.

* **Build problems** If you encounter build problems, we recommend you to look at the corresponding error output. When fixing problems, make sure to run a clean build to remove cached and previously built files. 

* **Pixi environment problems**: If you encounter Pixi environment problems, you can rebuild your Pixi environment by removing the environment using `pixi clean` and reinstalling it. 

* **Missing packages in Pixi**: This Pixi environment configuration does not provide all ROS2 packages you would find in a ROS2 Desktop Full Install. You can install additional packages using Pixi's install routine, e.g. to install turtle sim use `pixi add ros-humble-turtlesim`. This will install the package and automatically add it to your Pixi manifest file `pixi.toml`. Note: not all packages might be available via Pixi for your system. See the [Pixi documentation](https://pixi.prefix.dev/latest/tutorials/ros2/) for more information.

* **Inconsistent results across your group?** Make sure to use the same configuration of your workspace. Look at `git diff` to see changes. Moreover, we recommend using Pixi's lock file `pixi.lock`. This file pins the exact versions of dependencies installed in your Pixi environment. You can share this lock file within your group to detect inconsistencies and install the same dependency versions across your group.

