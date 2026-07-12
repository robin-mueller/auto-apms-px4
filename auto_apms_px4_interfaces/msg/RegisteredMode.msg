# Announces a PX4 flight mode that has been registered with the FMU, mapping its name to the nav_state (mode id) that
# PX4 assigns dynamically during registration. Published (latched and periodically re-announced) by
# auto_apms_px4::ModeRegistrationHandler so that behaviors can resolve a mode name to its nav_state and switch to
# it via the SwitchMode behavior tree node.

string name       # Registered mode name (as passed to px4_ros2::ModeBase::Settings).
uint8 nav_state   # PX4 navigation state (mode id) assigned to the mode after successful registration.
