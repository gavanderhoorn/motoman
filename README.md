# Motoman

[![license - Apache-2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![license - BSD-3-Clause](https://img.shields.io/:license-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

[![support level: consortium / vendor](https://img.shields.io/badge/support%20level-consortium%20/%20vendor-brightgreen.svg)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)


## Contents

Direct ROS 2 port of all non-experimental robot support packages.

These packages are compatible with ROS 2 Foxy and newer.


## Changes wrt ROS 1 versions

Launch files are not yet provided.

The various `joint_names_...yaml` files are also no longer included, as those are not needed (they were only used by `motoman_driver`, which is no longer used).


## Building

`git clone` the repository into a ROS 2 Colcon workspace.

Use `rosdep install -i --from-paths /path/to/colcon_ws/src/motoman` to install dependencies. 

Build the workspace.

Activate the workspace by `source`-ing the correct version of `(local_)setup.(ba)sh`.
