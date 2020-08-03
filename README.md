# HRIC System
> This project is part of a thesis ([source][thesis-pdf]) presented for the degree of **BSc in Engineering (Robot Systems)** at the **University of Southern Denmark (SDU)**.

For research in safety of Human-Robot Interaction and Collaboration (HRIC), an integrated simulation environment is setup, facilitating the means of real time robot interfacing and collection of sensory data, furthermore providing a generic framework for development and deployment of an arbitrary robot controller in both a simulated and real environment.

![workcell-setup][img-workcell-setup]

Using the workcell in SDU Industry 4.0 lab, equipped with a Franka Emika Panda robot and various perceptual sensory equipment (e.g., motion capture system, EMG sensors etc.), the Robot Operating System (ROS) middleware is used as the underlying framework of the software architecture, in which the development of the generic controller framework is be accommodated by ROS Control and Franka ROS, using Gazebo as the simulation environment.

To analyze human-robot interaction, the robot model is integrated into a biomechanical simulator (OpenSim), automating the transformation of a Unified Robot Description Format (URDF) model to OpenSim Model Format (OSIM).

## Overview

### Features

This reposity contains the `catkin` workspace used for the HRIC system, consisting of various packages.

- [`franka_description`][pkg-franka-desc] - modified `URDF` robot descriptions with estimated dynamics parameters.
- [`franka_gazebo`][pkg-franka-gazebo] - integration of Panda into `Gazebo` along with emulation of joint space robot dynamics.
- [`franka_sim_controllers`][pkg-franka-sim] - various controllers to be used in the simulated environment.
- [`franka_irl_controllers`][pkg-franka-irl] - various controllers to be used in the real environment.
- [`urdf_to_osim`][pkg-urdf-to-osim] - automated transformation of URDF model to OSIM model.
- [`mocap_sampler`][pkg-mocap] - sampling of the OptiTrack MOCAP system using NatNet SDK.

### Dependencies

This project heavily relies on the Robot Operating System (ROS) framework accompanied by `libfranka` and `franka_ros` for real time robot interfacing. The system is running a Ubuntu (`18.04.3 LTS`) distribtuion which is patched with a [real-time kernel][rt-kernel] (`5.4.10rt`). The GDE doesn't really matter, but a lightweight GDE might improve performance, e.g. [LDXE][lubuntu]. The project is compiled using `C++17 GCC`, managed with `catkin` build system, CMake (`2.8.3`) and `rosdep` for dependecny managements, built with:

- [`ROS`][ros] - robotics middleware.
- [`libfranka`][libfranka] - C++ interface for Franka Control Interface.
- [`franka_ros`][franka-ros] - metapackage that integrates libfranka into `ROS` and `ros_control`.
- [`Gazebo`][gazebo] - robot simulation environment.
- [`OpenSim 4.1`][opensim] - biomechanical simulation environment.
- [`KDL`][kdl] - rigid-body dynamics library.
- [`Eigen3`][eigen3] - high-level `C++` library for linear algebra etc.
- [`NatNet`][natnet] - SDK for interfacting OptiTrack Motive.

All necessary documentation for the Franka Emika Panda robot and the FCI (`libfranka`, `franka_ros` etc.) can be found [here][franka-docs]. The OpenSim controller are compileable on Linux, but are only useful when compiled on Windows (yeah, that sucks) and used with the OpenSim GUI.

---

## Getting started

It is recommended to install the system on a dedicated workstation computer, directly connected to the Panda robot(s) and any measurements devices; a network switch is permissable but not advisable.

### Installation

The installation is comprised of the following steps:

1. Installing Ubuntu (`18.04.3 LTS`)
2. Installing `ROS`
3. Pathcing real time kernel and configuring performance parameters
4. Installing `libfranka` and `franka_ros`
5. Cloning repository (installing `catkin` workspace)
6. Installing dependecies using scripts and `rosdep`

After a clean install of Ubuntu, there are [bash scripts][script-dir] available for system configuration and installation of the real time kernel. A [network configuration][franka-net-conf] is necesasry in order to communicate with the robot.

> **NOTICE:**
> The scripts modify several system parameters; it is recommend to examine the script and comment out any unnecessary parts.

The [`setup-essential.bash`][script-ess] script installs any essential packages. The [`setup-ros.bash`][script-ros] installs `ROS` as well installs the dependency manger `rosdep` and and initializes it. These scripts must be run first, otherwise dependecy issues might arise.

> **NOTICE:**
> The RT kernel does NOT support NVIDIA driver binaries; the default nouveau drivers might pose some issues.

The [`setup-rt.bash`][script-rt] script downloads, configures, compiles, and installs the real time kernel; the script includes a [guide][rt-kernel-guide] on how to configure the kernel using a graphical interface.

Once the system is configured, this repository can be cloned (typically `~/Desktop/hric-system`). From within the source directory of the catkin workspace (e.g., `~/Desktop/hric-system/ws`), the following command will automatically install any missing catkin dependecies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

The project can then be built by issuing the command (optionally specifying a custom libfranka build):

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/Desktop/libfranka/build
```

Furthermore, the [`setup-opensim.bash`][script-opensim] script downloads and compiles the OpenSim library; the installed location must then be specified in the CMake files of where the OpenSim library is used (e.g., `urdf_to_osim` package).

### Usage

In order for `roslaunch` and `rosrun` to be able to locate the contents of the workspace whenever a new terminal window, it must be sourced by issuing the `source devel/setup.bash` command from the workspace directory. This command can also be added to the `~/.bashrc` file.

An [advanced network performance analysis][franka-comm-test] can be performed using the `communication_test` example from `libfranka/build/examples/`. The provided scripts should configure the system such that when running on a RT kernel, the test  should be greater or equal to a success rate of `0.95`.

The simulated environment with a trajectory controler can be launched using `roslaunch franka_sim_controllers joint_trajectory_control`, exposing a `ROS` interface for commanding a `trajectory_msgs/JointTrajectory` type setpoint at the `/panda_joint_trajectory_controller/command` topic.

The real robot can be interfaced using any of the `franka_irl_controller` controller or using a controller from the (not provided) `franka_example_controllers` package (part of `franka_ros`).

## Acknowledgements

- [Cheng Fang][ref-cheng] - project supervisor for bachelor thesis.
- [Daniel Tofte][ref-daniel] - comrade of the bachelor thesis, noble Corona warrior.
- [Erdal Perkel][ref-erdal] - inspiration for integration of Franka Emika Panda into Gazebo.

<!-- LINKS -->

[thesis-pdf]: /docs/bachelor-thesis-marta16.pdf
[img-workcell-setup]: /assets/media/img/workcell-setup.jpg

[semver]: http://semver.org/
[releases]: about:blank
[changelog]: CHANGELOG.md
[wiki]: about:blank

[pkg-franka-desc]: /ws/src/franka_description
[pkg-franka-gazebo]: /ws/src/franka_gazebo
[pkg-franka-sim]: /ws/src/franka_sim_controllers
[pkg-franka-irl]: /ws/src/franka_irl_controllers
[pkg-urdf-to-osim]: /ws/src/urdf_to_osim
[pkg-mocap]: /ws/src/mocap_sampler

[franka-docs]: https://frankaemika.github.io
[franka-ros]: https://github.com/frankaemika/franka_ros
[franka-net-conf]: https://frankaemika.github.io/docs/getting_started.html#setting-up-the-network
[franka-comm-test]: https://frankaemika.github.io/docs/troubleshooting.html#advanced-network-performance-analysis
[libfranka]: https://frankaemika.github.io/docs/libfranka.html
[ros]: http://wiki.ros.org/melodic/
[gazebo]: http://gazebosim.org/
[opensim]: http://simtk.org/projects/opensim
[kdl]: https://www.orocos.org/kdl
[eigen3]: eigen.tuxfamily.org/index.php
[natnet]: https://optitrack.com/products/natnet-sdk/
[lubuntu]: https://lubuntu.me/
[script-dir]: /assets/scripts/
[script-ess]: /assets/scripts/setup-essential.bash
[script-ros]: /assets/scripts/setup-ros.bash
[script-rt]: /assets/scripts/setup-rt.bash
[script-opensim]: /assets/scripts/setup-opensim.bash
[rt-kernel]: https://index.ros.org/doc/ros2/Tutorials/Building-Realtime-rt_preempt-kernel-for-ROS-2/
[rt-kernel-guide]: https://hungpham2511.github.io/setup/install-rtlinux/

[ref-cheng]: https://scholar.google.com/citations?user=B3S4PyQAAAAJ
[ref-daniel]: https://github.com/dscho15
[ref-erdal]: https://github.com/erdalpekel
