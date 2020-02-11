# Real-time system configuration

## Installation

The system is running a Ubuntu (`18.04.3 LTS`) distribtuion which is patched with a [real-time kernel][rt-kernel] (`5.4.3rt`). The GDE doesn't really matter, but a lightweight GDE might improve performance, e.g. [LDXE][lubuntu].

## Configuration

Once a clean install of Linux is installed, there are scripts available for both the setup of real-time kernel and system configuration. The [`sysconf.bash`][sysconf-sh] script installs essential packages, `ros-melodic`, `libfranka`, `franka_ros` etc. It creates a `catkin` workspace, configures the network for robot connection and modifies system parameters (deletes packages / disables daemons) to optimize the system performance.

A [modified version][erdal-ros] of `franka_ros` is installed, enabling simulation of the robot by integrating the FRANKA EMIKA Panda robot into Gazebo.

## libfranka checklist

An [advanced network performance analysis][comm-test] can be performed using the `communication_test` example from `libfranka`. It requires several steps in order to function properly:

- Launch system with RT Linux kernel
- Source `setup.bash` file
- Set CPU frequency control to performance mode
- Set robot mode
- Disable all unecessary background daemons/programs

The desired result of the test should be greater or equal to a success rate of `0.95`.

[rt-kernel]: https://index.ros.org/doc/ros2/Tutorials/Building-Realtime-rt_preempt-kernel-for-ROS-2/
[lubuntu]: https://lubuntu.me/
[rtkernel-sh]: google.com
[sysconf-sh]: google.com
[erdal-ros]: https://erdalpekel.de/?p=55
[comm-test]: https://frankaemika.github.io/docs/troubleshooting.html#advanced-network-performance-analysis
