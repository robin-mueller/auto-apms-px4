<p align="center">
  <img width="30%" src="https://github.com/PX4/PX4-graphics/blob/master/PX4_Logo_Black_RGB.png?raw=true">
  <img width="50%" src="https://robin-mueller.github.io/auto-apms-guide/logo/logo.png">
</p>
<div align="center">

<a href="https://robin-mueller.github.io/auto-apms-guide/usage/concepts/px4-bridge">![Docs](https://img.shields.io/website?url=https%3A%2F%2Frobin-mueller.github.io%2Fauto-apms-guide&label=Documentation)</a>
<a href="https://doi.org/10.5281/zenodo.14790307">![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.14790307.svg)</a>

</div>

# Using ROS 2 and AutoAPMS with PX4 Autopilot

This repository holds ROS 2 packages enabling behavior-based ROS 2 applications for the popular [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot) project. This implementation adopts the [AutoAPMS](https://github.com/robin-mueller/auto-apms) framework and introduces plenty of plugins making it easier than ever to communicate with PX4 from e.g. a companion computer running ROS 2.

## PX4 Modes exposed as ROS 2 Actions

The package `auto_apms_px4` implements ROS 2 actions acting as peers for common PX4 flight modes:

Class Name | Description
:---: | ---
`auto_apms_px4::ArmDisarmSkill` | Arm respectively disarm the UAS
`auto_apms_px4::EnableHoldSkill` | Enable the standard mode for holding the current position
`auto_apms_px4::GoToLocalSkill` | Fly to the given local position
`auto_apms_px4::GoToGlobalSkill` | Fly to the given global position
`auto_apms_px4::LandSkill` | Land immediately at the current position
`auto_apms_px4::TakeoffSkill` | Takeoff to a given height
`auto_apms_px4::RTLSkill` | Return to the launch position
`auto_apms_px4::MissionSkill` | Execute the PX4 mission that was uploaded to the UAS

If you want to use those actions, either create you're own launch script and load the ROS 2 nodes using [ROS 2 Composition](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Composition.html#) or run

```bash
ros2 launch auto_apms_px4 skills_launch.py
```
