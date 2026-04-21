# auto_apms_px4

To successfully build this package, you must manually install the following dependencies (they are not available via rosdep):

| Name | Description |
| :---| :--- |
| [px4_ros2_cpp](https://github.com/Auterion/px4-ros2-interface-lib) | Library that allows to model PX4 flight modes as ROS 2 applications. |
| [px4_msgs](https://github.com/PX4/px4_msgs) | ROS 2 message definitions for the PX4 Autopilot project. |

> [!NOTE]
> This project follows the same branch naming convention as the rest of the PX4 ecosystem (e.g. "release/XX"). To ensure compatibility, make sure to checkout the same branch for all the dependencies!
