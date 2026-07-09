cmake_minimum_required(VERSION 3.8)
project(trainit_motion_runtime)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- dependencies ------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(geometric_shapes REQUIRED)   # generic scene loader: mesh -> shape_msgs/Mesh

# --- BehaviorTree.CPP v4 (apt: ros-humble-behaviortree-cpp; FetchContent fallback) ---
find_package(behaviortree_cpp QUIET)
if(NOT behaviortree_cpp_FOUND)
  message(STATUS "[trainit_motion_runtime] behaviortree_cpp not found; fetching from GitHub.")
  include(FetchContent)
  FetchContent_Declare(behaviortree_cpp
    GIT_REPOSITORY https://github.com/BehaviorTree/BehaviorTree.CPP.git
    GIT_TAG 4.6.2
    GIT_SHALLOW TRUE)
  set(BTCPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
  set(BTCPP_EXAMPLES    OFF CACHE BOOL "" FORCE)
  set(BTCPP_UNIT_TESTS  OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(behaviortree_cpp)
  set(BTCPP_FETCHED TRUE)
else()
  set(BTCPP_FETCHED FALSE)
endif()

function(link_btcpp tgt)
  if(BTCPP_FETCHED)
    target_link_libraries(${tgt} behaviortree_cpp)
  elseif(TARGET behaviortree_cpp::behaviortree_cpp)
    target_link_libraries(${tgt} behaviortree_cpp::behaviortree_cpp)
  else()
    target_link_libraries(${tgt} behaviortree_cpp)
  endif()
endfunction()

set(TRAINIT_DEPS
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  std_srvs
  control_msgs
  shape_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2_eigen
  tf2_geometry_msgs)

# --- core runtime library ----------------------------------------------------
# Sources are added milestone by milestone; the build stays green at each step.
add_library(trainit_motion_runtime SHARED
  src/motion_types.cpp
  src/manual_motion_strategy_selector.cpp
  src/gripper_controller.cpp
  src/planning_scene_manager.cpp
  src/trajectory_validator.cpp
  src/moveit_motion_runtime.cpp
  src/process_controller.cpp
  src/pilz_sequence_backend.cpp
  src/cartesian_waypoint_backend.cpp
  src/process_path_generators.cpp
  src/process_path_loaders.cpp
  src/task_runner.cpp
  src/task_loader.cpp
  src/bt_profiles.cpp
  src/bt_nodes.cpp
)
target_include_directories(trainit_motion_runtime PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(trainit_motion_runtime ${TRAINIT_DEPS})
target_link_libraries(trainit_motion_runtime yaml-cpp)
link_btcpp(trainit_motion_runtime)

# --- install -----------------------------------------------------------------
install(DIRECTORY include/ DESTINATION include)
install(TARGETS trainit_motion_runtime
  EXPORT export_trainit_motion_runtime
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

# --- generic application runner (no application logic; the task is YAML) ------
add_executable(trainit_run_task src/trainit_run_task_node.cpp)
target_link_libraries(trainit_run_task trainit_motion_runtime)
ament_target_dependencies(trainit_run_task rclcpp)
install(TARGETS trainit_run_task DESTINATION lib/${PROJECT_NAME})

# --- BehaviorTree runner (generic; the application is the tree XML) ----------
# Groot2 live monitor is best-effort: enabled only if libzmq is present.
find_package(PkgConfig QUIET)
set(TRAINIT_GROOT2 OFF)
if(PkgConfig_FOUND)
  pkg_check_modules(ZMQ QUIET libzmq)
  if(ZMQ_FOUND)
    set(TRAINIT_GROOT2 ON)
  endif()
endif()

add_executable(trainit_run_bt src/trainit_run_bt_node.cpp)
target_link_libraries(trainit_run_bt trainit_motion_runtime)
link_btcpp(trainit_run_bt)
ament_target_dependencies(trainit_run_bt rclcpp)
if(TRAINIT_GROOT2)
  target_compile_definitions(trainit_run_bt PRIVATE BTCPP_GROOT2_SUPPORT)
  if(ZMQ_LIBRARIES)
    target_link_libraries(trainit_run_bt ${ZMQ_LIBRARIES})
  endif()
  message(STATUS "[trainit_motion_runtime] Groot2 live monitor ENABLED (libzmq).")
else()
  message(STATUS "[trainit_motion_runtime] Groot2 live monitor DISABLED (no libzmq).")
endif()
install(TARGETS trainit_run_bt DESTINATION lib/${PROJECT_NAME})

# --- generic scene loader (box|cylinder|sphere|mesh -> planning scene) --------
# Standalone node (no BT, no runtime lib); used by the config-phase
# scene_loader launch and the generated bundle bringup alike.
add_executable(scene_manager_node src/scene_manager_node.cpp)
ament_target_dependencies(scene_manager_node
  rclcpp moveit_ros_planning_interface moveit_msgs shape_msgs geometry_msgs geometric_shapes
  std_msgs std_srvs)
install(TARGETS scene_manager_node DESTINATION lib/${PROJECT_NAME})

# python launch-assembly helper (the "embedded generic moveit config")
ament_python_install_package(${PROJECT_NAME})

# generic planner pipelines + profiles
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_trainit_motion_runtime HAS_LIBRARY_TARGET)
ament_export_dependencies(${TRAINIT_DEPS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip house-style linters (consistent with the legacy package); functional
  # gtests below are the meaningful tests.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  foreach(test_name
      test_planner_mode_parser
      test_strategy_selector
      test_path_generators
      test_process_path_loader)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} trainit_motion_runtime)
  endforeach()
endif()

ament_package()
