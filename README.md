<p align="center">
  <img width="30%" src="https://github.com/PX4/PX4-graphics/blob/master/PX4_Logo_Black_RGB.png?raw=true">
  <img width="50%" src="https://autoapms.github.io/auto-apms-guide/logo/autoapms_logo.svg">
</p>
<div align="center">

<a href="https://autoapms.github.io/auto-apms-guide/concept/px4-integration">![Docs](https://img.shields.io/website?url=https%3A%2F%2Fautoapms.github.io%2Fauto-apms-guide&label=🎓Documentation)</a>
<a href="https://doi.org/10.5281/zenodo.14790307">![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.14790307.svg)</a>

</div>

# Using ROS 2 and AutoAPMS with PX4 Autopilot

This repository holds ROS 2 packages enabling behavior-based ROS 2 applications for the popular [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot) project. This implementation adopts the [AutoAPMS](https://github.com/AutoAPMS/auto-apms) framework and introduces plenty of plugins making it easier than ever to communicate with PX4 from e.g. a companion computer running ROS 2.

> [!NOTE]
> This project follows the same branch naming convention as the rest of the PX4 ecosystem (e.g. "release/XX"). To ensure compatibility, make sure to checkout the same branch for [px4_msgs](https://github.com/PX4/px4_msgs) and [px4-ros2-interface-lib](https://github.com/Auterion/px4-ros2-interface-lib)
>
> The dependency versions are pinned for CI inside [dependencies.repos](./dependencies.repos) and should be used for local development as well.


