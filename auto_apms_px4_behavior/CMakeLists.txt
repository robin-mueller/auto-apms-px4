cmake_minimum_required(VERSION 3.12)
project(auto_apms_px4_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(auto_apms_behavior_tree_core REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(auto_apms_px4_interfaces REQUIRED)

#
# ------------ Library --------------
#

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE
    # Adding this for the generated node model header
    auto_apms_behavior_tree_core::auto_apms_behavior_tree_core
)

#
# ------------ Behavior tree nodes --------------
#

add_library(behavior_tree_nodes SHARED
    "src/action/arm.cpp"
    "src/action/disarm.cpp"
    "src/action/enable_hold.cpp"
    "src/action/goto.cpp"
    "src/action/land.cpp"
    "src/action/mission.cpp"
    "src/action/rtl.cpp"
    "src/action/takeoff.cpp"
    "src/sub/get_position.cpp"
)
ament_target_dependencies(behavior_tree_nodes
    auto_apms_behavior_tree_core
    Eigen3
    px4_msgs
    auto_apms_px4_interfaces
)
auto_apms_behavior_tree_register_nodes(behavior_tree_nodes
    "${PROJECT_NAME}::ArmAction"
    "${PROJECT_NAME}::DisarmAction"
    "${PROJECT_NAME}::EnableHoldAction"
    "${PROJECT_NAME}::GoToAction"
    "${PROJECT_NAME}::LandAction"
    "${PROJECT_NAME}::MissionAction"
    "${PROJECT_NAME}::RTLAction"
    "${PROJECT_NAME}::TakeoffAction"
    "${PROJECT_NAME}::GetPosition<GlobalPositionMsg>"
    "${PROJECT_NAME}::GetPosition<LocalPositionMsg>"
    NODE_MANIFEST
    "config/behavior_tree_nodes.yaml.in"
    NODE_MODEL_HEADER_TARGET ${PROJECT_NAME}
)

#
# ------------ Behavior Trees --------------
#

auto_apms_behavior_tree_register_trees(
    "behavior/relative_goto.xml"
    NODE_MANIFEST
    "auto_apms_behavior_tree::behavior_tree_nodes"
    "${PROJECT_NAME}::behavior_tree_nodes"
)

#
# ------------ Packaging --------------
#

# Export library
install(
    TARGETS
    ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    auto_apms_behavior_tree_core # Adding this for the generated node model header
)

# Plugin library
install(
    TARGETS
    behavior_tree_nodes
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_package()
