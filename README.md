# hric-system

Main code repository of the Human-Robot Interaction and Collaboration (HRIC) system.

![libfranka](https://frankaemika.github.io/docs/_images/libfranka-architecture.png "libfranka schematic overview.")

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Dependencies

The system is running a Ubuntu (`18.04.3 LTS`) distribtuion which is patched with a [real-time kernel][rt-kernel] (`5.4.3rt`). The GDE doesn't really matter, but a lightweight GDE might improve performance, e.g. [LDXE][lubuntu]. The project is compiled using `C++17 GCC`, managed with CMake (`3.10.2`) and built with:

* [ROS (1.14.7)][ros] - used as framework for robot operation
* [libfranka][libfranka] - C++ interface for Franka Control Interface
* [franka_ros][franka_ros] - metapackage that integrates libfranka into ROS and ROS control.

All necessary documentation for the Franka Emika Panda robot can be found [here](https://frankaemika.github.io/docs/).

### Configuration

Once a clean install of Linux is installed, there are scripts available for both the setup of real-time kernel and system configuration. The [`sysconf.bash`][sysconf-sh] script installs essential packages, `ros-melodic`, `libfranka`, `franka_ros` etc. It creates a `catkin` workspace, configures the network for robot connection and modifies system parameters (deletes packages / disables daemons) to optimize the system performance.

A [modified version][erdal-ros] of `franka_ros` is installed, enabling simulation of the robot by integrating the FRANKA EMIKA Panda robot into Gazebo.

### Installation

Installation of the system/workspace/package.

### Test

An [advanced network performance analysis][comm-test] can be performed using the `communication_test` example from `libfranka`. It requires several steps in order to function properly:

- Launch system with RT Linux kernel
- Source `setup.bash` file
- Set CPU frequency control to performance mode
- Set robot mode
- Disable all unecessary background daemons/programs

The desired result of the test should be greater or equal to a success rate of `0.95`.

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [releases on this repository](about:blank). Furthermore, this [changelog](CHANGELOG.md) documents the most relevant changes.

## License

No license has been decided yet.

[ros]: http://wiki.ros.org/melodic/
[libfranka]: https://frankaemika.github.io/docs/libfranka.html
[franka_ros]: https://github.com/frankaemika/franka_ros

[rt-kernel]: https://index.ros.org/doc/ros2/Tutorials/Building-Realtime-rt_preempt-kernel-for-ROS-2/
[lubuntu]: https://lubuntu.me/
[rtkernel-sh]: google.com
[sysconf-sh]: google.com
[erdal-ros]: https://erdalpekel.de/?p=55
[comm-test]: https://frankaemika.github.io/docs/troubleshooting.html#advanced-network-performance-analysis
