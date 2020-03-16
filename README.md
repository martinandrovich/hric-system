# HRIC System
Human-Robot Interaction and Collaboration System.

* [Overview](#overview)
* [Getting Started](#getting-started)
	+ [Dependencies](#dependencies)
	+ [Configuration](#configuration)
	+ [Installation](#installation)
	+ [Test](#test)
	+ [API & Troubleshooting](#api--troubleshooting)
* [Versioning](#versioning)
* [License](#license)
* [Acknowledgments](#acknowledgments)

## Overview

![libfranka](https://frankaemika.github.io/docs/_images/libfranka-architecture.png "libfranka schematic overview.")

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Dependencies

The system is running a Ubuntu (`18.04.3 LTS`) distribtuion which is patched with a [real-time kernel][rt-kernel] (`5.4.10rt`). The GDE doesn't really matter, but a lightweight GDE might improve performance, e.g. [LDXE][lubuntu]. The project is compiled using `C++17 GCC`, managed with CMake (`3.10.2`) and built with:

* [ROS (1.14.7)][ros] - used as framework for robot operation
* [libfranka][libfranka] - C++ interface for Franka Control Interface
* [franka_ros][franka_ros] - metapackage that integrates libfranka into ROS and ROS control.
* [Gazebo][gazebo] - used as robot simulation environment
* [OpenSim][opensim] - used as simulation environment

All necessary documentation for the Franka Emika Panda robot can be found [here](https://frankaemika.github.io/docs/). A [modified version][erdal-ros] of `franka_ros` is installed, enabling simulation of the robot by integrating the FRANKA EMIKA Panda robot into Gazebo.

### Configuration

Once a clean install of Linux is installed, there are [scripts][sh-dir] available for system configuration and installation of the real-time kernel. A [network configuration][franka-net-conf] is necesasry in order to communicate with the robot.

Installation of the system/workspace/package; the following configuration steps are necessary:

1. RT kernel + performance configuration
2. Installing `ROS`
3. Installing `libfranka`
4. Cloning repository (catkin workspace)

Scripts for these steps will be available later.

> **NOTICE:**
> The scripts modify several system parameters; it is recommend to examine the script and comment out any unnecessary parts.

The [`sysconf.bash`][sysconf-sh] script installs essential packages, necessary libraries (`ros-melodic`, `libfranka`, `franka_ros`) etc. It creates a `catkin` workspace, configures the network for robot connection and modifies system parameters (deletes packages / disables daemons) to optimize the system performance.

> **NOTICE:**
> The RT-kernel does NOT support NVIDIA driver binaries; the default nouveau drivers might pose some issues.

The [`rtkernel.bash`][rt-kernel-sh] script downloads, configures, compiles, and installs the realtime kernel; the script includes a [guide][rt-kernel-guide] on how to configure the kernel using a graphical interface.

### Test

An [advanced network performance analysis][comm-test] can be performed using the `communication_test` example from `libfranka/build/examples/`. The provided scripts should configure the system such that when running on a RT kernel, the test  should be greater or equal to a success rate of `0.95`.

### API & Troubleshooting

All necessary documentation is available in the [wiki] of this repository.

## Versioning

We use [SemVer][semver] for versioning. For the versions available, see the [releases on this repository][releases]. Furthermore, this [changelog] documents the most relevant changes.

## License

No license has been decided yet.

## Acknowledgments

- [Erdal Perkel][erdal-git] - integration of Franka Emika Panda into Gazebo

[semver]: http://semver.org/
[releases]: about:blank
[changelog]: CHANGELOG.md
[wiki]: about:blank

[ros]: http://wiki.ros.org/melodic/
[libfranka]: https://frankaemika.github.io/docs/libfranka.html
[franka_ros]: https://github.com/frankaemika/franka_ros
[gazebo]: http://gazebosim.org/
[opensim]: http://simtk.org/projects/opensim
[lubuntu]: https://lubuntu.me/
[franka-net-conf]: https://frankaemika.github.io/docs/getting_started.html#setting-up-the-network
[sh-dir]: /scripts/
[sysconf-sh]: /scripts/sysconf.bash
[rt-kernel]: https://index.ros.org/doc/ros2/Tutorials/Building-Realtime-rt_preempt-kernel-for-ROS-2/
[rt-kernel-sh]: /scripts/rtkernel.bash
[rt-kernel-guide]: https://hungpham2511.github.io/setup/install-rtlinux/
[erdal-ros]: https://erdalpekel.de/?p=55
[erdal-git]: https://github.com/erdalpekel
[comm-test]: https://frankaemika.github.io/docs/troubleshooting.html#advanced-network-performance-analysis
