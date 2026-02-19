# `za6_control_sandbox` package

[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)

**ROS controller tests for the ZA6**

## Contents

- [Installation](#1)
- [Simulating the System in ROS 2](#2)
- [Controller Demos](#3)
- [Controller Configuration](#4)

<a id='1'></a>

## Installation

Create ROS 2 workspace and clone this package into a `src` directory.

Import and install the necessary dependencies:

```bash
sudo apt update
rosdep update
cd src
vcs import < za6_control_sandbox/za6_control_sandbox.repos
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

Build the packages:

```bash
cd <COLCON_WORKSPACE>
colcon build
```

<a id='2'></a>

## Simulating the controllers in ROS 2

Bring up the ros2 simulation as follows:

```bash
ros2 launch za6_control_sandbox bringup.launch.py controller:=pose_controller
```

Use the `controller` parameter to select the loaded controller.

```
'controller':
        Which controller should be started?. Valid choices are: ['pose_controller', 'as_nullspace_controller']
        (default: 'as_nullspace_controller')
```

> [!NOTE]
> All args can be seen by passing the `--show-args` flag

<a id='3'></a>

## Controller Demos

Launch the desired controller demo with the same type of `controller` used in
the bringup.

```bash
ros2 launch za6_control_sandbox control_demo.launch.py controller:=pose_controller
```

<a id='4'></a>

## Controller Configuration

The `axially_symmetric_controller`s use tool axis rotation to optimize the robot
configuration. The objective used to determine the optimal configuration can be
modified with the `rr_objective` parameter in the [controller
configuration](config/za6_controllers.yaml).
