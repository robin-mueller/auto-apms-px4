<p align="center">
  <img width="30%" src="https://github.com/PX4/PX4-graphics/blob/master/PX4_Logo_Black_RGB.png?raw=true">
  <img width="50%" src="https://robin-mueller.github.io/auto-apms-guide/logo/logo.png">
</p>
<div align="center">

<a href="https://robin-mueller.github.io/auto-apms-guide/concept/px4-integration">![Docs](https://img.shields.io/website?url=https%3A%2F%2Frobin-mueller.github.io%2Fauto-apms-guide&label=🎓Documentation)</a>
<a href="https://doi.org/10.5281/zenodo.14790307">![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.14790307.svg)</a>

</div>

# Using ROS 2 and AutoAPMS with PX4 Autopilot

This repository holds ROS 2 packages enabling behavior-based ROS 2 applications for the popular [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot) project. This implementation adopts the [AutoAPMS](https://github.com/robin-mueller/auto-apms) framework and introduces plenty of plugins making it easier than ever to communicate with PX4 from e.g. a companion computer running ROS 2.

## PX4 Compatibility

This repo has been tested with **PX4 1.15**. The following links point to the respective revisions of the PX4 dependency repositories:

- [px4_msgs](https://github.com/PX4/px4_msgs/tree/release/1.15)
- [px4-ros2-interface-lib](https://github.com/Auterion/px4-ros2-interface-lib/commit/22c378d92524e134610a04aa770d35f0431390b4)
