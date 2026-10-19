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
find_package(action_msgs REQUIRED)   # ServoTrack arbitration (GoalStatusArray)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)      # ADR-0022: an EEF axis goal (FollowJointTrajectory)
find_package(trainit_interfaces 0.2.0 REQUIRED)   # ADR-0022: EndEffectorState (TMR 2.0.0); ADR-0026: GraspCandidate (2.5.0)
find_package(diagnostic_msgs REQUIRED)   # ADR-0026: a grasp candidate's metadata
find_package(shape_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)   # capability_names.h: the runners' MoveIt start-up wait
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)   # perception contract consumed by DetectObject
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
  action_msgs
  geometry_msgs
  std_msgs
  std_srvs
  control_msgs
  trajectory_msgs
  trainit_interfaces
  diagnostic_msgs
  shape_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  vision_msgs)

# --- core runtime library ----------------------------------------------------
# Sources are added milestone by milestone; the build stays green at each step.
add_library(trainit_motion_runtime SHARED
  src/motion_types.cpp
  src/motion_failure.cpp   # vNext M2.7: whose failure a failed motion is (pure; seam scan kPlain)
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
  # the canonical end-effector contract (ADR-0022): the parser and the two pure tables are
  # MoveIt-free, the registry is the runner's standard EEF clients (test_m2_5_seam_scan: kPlain)
  src/end_effector_contract.cpp
  src/actuator_outcome.cpp
  src/verify_grasp.cpp
  src/end_effector_registry.cpp
  # vNext M3.1 (ADR-0024, ADR-0025): the motion provider table and the path library (both pure,
  # MoveIt-free; test_m2_5_seam_scan: kPlain)
  src/motion_providers.cpp
  src/path_library.cpp
  # vNext M3.6 (ADR-0026): the grasp-candidate contract's pure half (MoveIt-free; test_m2_5_seam_scan: kPlain)
  src/grasp_candidates.cpp
)
target_include_directories(trainit_motion_runtime PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(trainit_motion_runtime ${TRAINIT_DEPS})
target_link_libraries(trainit_motion_runtime yaml-cpp)
link_btcpp(trainit_motion_runtime)
# The planner mapping (ADR-0021): a PlannerMode added to the enum but not to a switch
# over it is a build error in these two files, not a warning. vNext M2.7: likewise a MotionPhase
# added without its case in the motion-failure classification (src/motion_failure.cpp).
set_source_files_properties(src/motion_types.cpp src/manual_motion_strategy_selector.cpp
  src/motion_failure.cpp PROPERTIES COMPILE_OPTIONS "-Werror=switch")
# The MotionRuntime seam (vNext M2.5, ADR-0015 amendment): the runtime's consumers reach MoveIt
# only through MotionRuntime (moveGroup() was deprecated in 1.9.0 and removed in M2.6). A
# deprecated declaration used in one of them is a build error, not a warning
# (test_m2_5_seam_scan counts what the compiler cannot see).
set_source_files_properties(src/bt_nodes.cpp src/task_runner.cpp src/trainit_run_bt_node.cpp
  src/trainit_run_task_node.cpp PROPERTIES COMPILE_OPTIONS "-Werror=deprecated-declarations")

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
target_include_directories(scene_manager_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)   # scene_service_names.hpp
ament_target_dependencies(scene_manager_node
  rclcpp moveit_ros_planning_interface moveit_msgs shape_msgs geometry_msgs geometric_shapes
  std_msgs std_srvs tf2_ros tf2_eigen)
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
  set(ament_cmake_lint_cmake_FOUND TRUE)   # FetchContent_* is CamelCase by CMake convention
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  foreach(test_name
      test_planner_mode_parser
      test_strategy_selector
      test_path_generators
      test_process_path_loader
      test_scene_service_names
      test_detection_candidates)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} trainit_motion_runtime)
  endforeach()

  # vNext M2.1: what every planner mode resolves to is frozen (test/data/), so the
  # planner registry changes who KNOWS the vocabulary, never what a move resolves to.
  ament_add_gtest(test_planner_resolution_identity test/test_planner_resolution_identity.cpp)
  target_link_libraries(test_planner_resolution_identity trainit_motion_runtime)
  target_compile_definitions(test_planner_resolution_identity PRIVATE
    TRAINIT_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")

  # The launch helper's pipeline set and keyword API (the contract every bundle calls).
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_trainit_moveit_config test/test_trainit_moveit_config.py TIMEOUT 60)

  # The planner registry (ADR-0021): the C++ mapping checked against
  # config/planner_profiles.yaml entry by entry, plus committed mutations of the registry
  # that must each fail with the message naming what is missing. Reads the SOURCE files.
  ament_add_gtest(test_planner_registry test/test_planner_registry.cpp)
  target_link_libraries(test_planner_registry trainit_motion_runtime)
  link_btcpp(test_planner_registry)
  target_compile_definitions(test_planner_registry PRIVATE
    TRAINIT_PLANNER_REGISTRY="${CMAKE_CURRENT_SOURCE_DIR}/config/planner_profiles.yaml"
    TRAINIT_PIPELINES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config/pipelines")
  ament_add_pytest_test(test_planner_registry_helper test/test_planner_registry_helper.py TIMEOUT 60)
  # ament_add_pytest_test() registers NOTHING (a configure-time warning only) when pytest cannot
  # start, and the suite then looks green without the helper's pins. This test makes that loud.
  add_test(NAME pytest_starts COMMAND ${Python3_EXECUTABLE} -m pytest --version)
  # ... and pytest_starts runs at TEST time, while registration happens at CONFIGURE time: a
  # configure without a working pytest leaves both suites unregistered and their old result files
  # still counted (found by the vNext M2.5 review). A suite that did not register fails a test.
  foreach(_trainit_pytest test_trainit_moveit_config test_planner_registry_helper)
    if(NOT TEST ${_trainit_pytest})
      message(WARNING "[trainit_motion_runtime] ${_trainit_pytest} is NOT registered: pytest could "
                      "not start at configure time (reconfigure with PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)")
      add_test(NAME ${_trainit_pytest}_was_not_registered COMMAND ${CMAKE_COMMAND} -E false)
    endif()
  endforeach()

  # The Pilz sequence backend's request (the path circ moves and process paths take), captured
  # by a fake sequence_move_group and checked against the registry's Cartesian entries.
  ament_add_gtest(test_pilz_sequence_request test/test_pilz_sequence_request.cpp TIMEOUT 30)
  target_link_libraries(test_pilz_sequence_request trainit_motion_runtime)
  target_compile_definitions(test_pilz_sequence_request PRIVATE
    TRAINIT_PLANNER_REGISTRY="${CMAKE_CURRENT_SOURCE_DIR}/config/planner_profiles.yaml")

  # vNext M3.1 (ADR-0024): the path library, pure (no ROS graph): every refusal, the derived
  # capabilities, the geometry checks, the dense sources (test/data/m3_1/).
  ament_add_gtest(test_path_library test/test_path_library.cpp TIMEOUT 60)
  target_link_libraries(test_path_library trainit_motion_runtime)
  target_compile_definitions(test_path_library PRIVATE
    TRAINIT_M31_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data/m3_1")

  # BT node registration/load test needs the behaviortree_cpp headers too.
  ament_add_gtest(test_bt_nodes test/test_bt_nodes.cpp)
  target_link_libraries(test_bt_nodes trainit_motion_runtime)
  link_btcpp(test_bt_nodes)

  # DetectObject's write path TICKED against a live detector (node + subscription +
  # TF), so the blackboard contract the rest of the system reads is executed, not
  # assumed. 20 s is the pub/sub discovery allowance, not the expected duration.
  ament_add_gtest(test_detect_object_tick test/test_detect_object_tick.cpp TIMEOUT 20)
  target_link_libraries(test_detect_object_tick trainit_motion_runtime)
  link_btcpp(test_detect_object_tick)

  # SetAttachedCollisionCheck / ResetScene TICKED against a live graph (vNext M0C #5): the
  # relative default reaches the tree's own namespace, and a scene manager that runs but
  # does not answer is a FAILURE. The wait_for_service timeouts make it ~10 s.
  ament_add_gtest(test_scene_nodes_tick test/test_scene_nodes_tick.cpp TIMEOUT 40)
  target_link_libraries(test_scene_nodes_tick trainit_motion_runtime)
  link_btcpp(test_scene_nodes_tick)

  # vNext M2.5: what every motion consumer sends to MoveIt, captured against in-process fakes
  # (test/m2_5/) and compared byte for byte with test/data/m2_5/, recorded on TMR 1.8.0 before
  # the MotionRuntime seam touched a production line. Each binary sets its own private ROS
  # domain IN CODE (and here, as a second guard), localhost only, and refuses to run if a node it
  # did not create is visible: MoveIt's client names are absolute, so isolation is the domain.
  find_package(sensor_msgs REQUIRED)
  function(trainit_m25_test name domain timeout source)
    ament_add_gtest(${name} test/${source}.cpp TIMEOUT ${timeout} SKIP_LINKING_MAIN_LIBRARIES
      ENV ROS_DOMAIN_ID=${domain} ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp)
    target_link_libraries(${name} trainit_motion_runtime)
    link_btcpp(${name})
    ament_target_dependencies(${name} sensor_msgs)
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/m2_5_actual)
    target_compile_definitions(${name} PRIVATE
      TRAINIT_M25_DOMAIN=${domain}
      TRAINIT_M25_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data/m2_5"
      TRAINIT_M25_ACTUAL_DIR="${CMAKE_CURRENT_BINARY_DIR}/m2_5_actual"
      TRAINIT_M25_PATHS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data/m2_5_paths"
      TRAINIT_M25_TASKS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data"
      TRAINIT_PROFILES_FILE="${CMAKE_CURRENT_SOURCE_DIR}/config/bt/trainit_profiles.yaml"
      TRAINIT_MOVEIT_VERSION="${moveit_ros_planning_interface_VERSION}"
      TRAINIT_BTCPP_VERSION="${behaviortree_cpp_VERSION}")
  endfunction()
  trainit_m25_test(test_m2_5_motion_requests 91 600 test_m2_5_motion_requests)
  # the runtime node constructed in /r3, and the whole process remapped to /r3 (the launch's way)
  trainit_m25_test(test_m2_5_namespace 92 300 test_m2_5_namespace)
  trainit_m25_test(test_m2_5_namespace_remap 93 300 test_m2_5_namespace)
  target_compile_definitions(test_m2_5_namespace_remap PRIVATE TRAINIT_M25_REMAP)
  # vNext M2.7 (owner decision D2): the two namespace binaries compare with the AFTER series
  # (test/data/m2_7_ns); the 1.8.0 BEFORE files stay in test/data/m2_5, byte-identical, and
  # test_m2_7_namespace_diff proves the difference is namespace resolution only.
  foreach(_t test_m2_5_namespace test_m2_5_namespace_remap)
    target_compile_definitions(${_t} PRIVATE
      TRAINIT_M27_NS_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data/m2_7_ns")
  endforeach()
  # ... pure (no ROS graph): it reads the committed files of both series; sha256 from the M2.5
  # harness header (test/m2_5/capture.hpp), whose include directories and macros it needs
  ament_add_gtest(test_m2_7_namespace_diff test/test_m2_7_namespace_diff.cpp TIMEOUT 30)
  target_link_libraries(test_m2_7_namespace_diff trainit_motion_runtime)
  target_include_directories(test_m2_7_namespace_diff PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
  target_compile_definitions(test_m2_7_namespace_diff PRIVATE
    TRAINIT_M25_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data/m2_5"
    TRAINIT_M25_ACTUAL_DIR="${CMAKE_CURRENT_BINARY_DIR}/m2_5_actual"
    TRAINIT_M27_NS_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data/m2_7_ns"
    TRAINIT_MOVEIT_VERSION="${moveit_ros_planning_interface_VERSION}"
    TRAINIT_BTCPP_VERSION="${behaviortree_cpp_VERSION}")
  # the class-A reads (StoreCurrentPose, ComputeTcpTarget, CheckRobotState) and ServoTrack
  trainit_m25_test(test_m2_5_state_reads 94 300 test_m2_5_state_reads)
  # the M2.5 review's extra BEFORE captures (frozen on 1.8.0 by test/data/m2_5/README.md)
  trainit_m25_test(test_m2_5_more_captures 96 600 test_m2_5_more_captures)
  # the direct-MoveIt accesses of the runtime consumers, counted in their source (no ROS)
  ament_add_gtest(test_m2_5_seam_scan test/test_m2_5_seam_scan.cpp TIMEOUT 30)
  target_compile_definitions(test_m2_5_seam_scan PRIVATE
    TRAINIT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  # the MotionRuntime header names no MoveIt C++ API and is implementable without MoveIt (no ROS)
  ament_add_gtest(test_m2_5_interface_purity test/test_m2_5_interface_purity.cpp TIMEOUT 30)
  target_link_libraries(test_m2_5_interface_purity trainit_motion_runtime)
  # ... checked on the include closure the compiler recorded (its dependency files)
  add_test(NAME test_m2_5_interface_closure COMMAND ${CMAKE_COMMAND}
    -DPURITY_DEPFILE=${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/test_m2_5_interface_purity.dir/test/test_m2_5_interface_purity.cpp.o.d
    -DCONTROL_DEPFILE=${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/test_m2_5_state_reads.dir/test/test_m2_5_state_reads.cpp.o.d
    -P ${CMAKE_CURRENT_SOURCE_DIR}/test/m2_5/interface_closure.cmake)

  # vNext M2.6 (tests BEFORE, ADR-0022 "Order of work and test first"): TODAY's end-effector
  # behaviour, per family, captured against fakes (test/m2_6/) and compared byte for byte with
  # test/data/m2_6/, recorded on production 1dda503 before the canonical EEF contract touched a
  # production line. Domains, set IN CODE (and here, as a second guard), localhost only,
  # graph-guarded: 97 in-process (one binary at a time: RESOURCE_LOCK), 88 the trainit_run_bt
  # child, 89 the scene_manager_node child.
  function(trainit_m26_test name domain timeout source)
    ament_add_gtest(${name} test/${source}.cpp TIMEOUT ${timeout} SKIP_LINKING_MAIN_LIBRARIES
      ENV ROS_DOMAIN_ID=${domain} ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp)
    target_link_libraries(${name} trainit_motion_runtime)
    link_btcpp(${name})
    ament_target_dependencies(${name} sensor_msgs)
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/m2_6_actual)
    target_compile_definitions(${name} PRIVATE
      TRAINIT_M26_DOMAIN=${domain}
      TRAINIT_M26_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data/m2_6"
      TRAINIT_M26_ACTUAL_DIR="${CMAKE_CURRENT_BINARY_DIR}/m2_6_actual"
      TRAINIT_RUN_BT_EXE="$<TARGET_FILE:trainit_run_bt>"
      TRAINIT_SCENE_MANAGER_EXE="$<TARGET_FILE:scene_manager_node>"
      # the M2.5 harness headers this one includes (test/m2_5/), unchanged
      TRAINIT_M25_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data/m2_5"
      TRAINIT_M25_ACTUAL_DIR="${CMAKE_CURRENT_BINARY_DIR}/m2_5_actual"
      TRAINIT_PROFILES_FILE="${CMAKE_CURRENT_SOURCE_DIR}/config/bt/trainit_profiles.yaml"
      TRAINIT_MOVEIT_VERSION="${moveit_ros_planning_interface_VERSION}"
      TRAINIT_BTCPP_VERSION="${behaviortree_cpp_VERSION}")
    add_dependencies(${name} trainit_run_bt scene_manager_node)
    set_tests_properties(${name} PROPERTIES RESOURCE_LOCK trainit_m26_domain_${domain})
  endfunction()
  # E1: Close/Open/SetGripper through GripperController against scripted GripperCommand servers
  trainit_m26_test(test_m2_6_gripper_command 97 300 test_m2_6_gripper_command)
  # E2: the grasp Bool on gripper_cmd_topic: when it is published, latched/volatile/late joiners,
  # two runners on one global name
  trainit_m26_test(test_m2_6_grasp_signal 97 300 test_m2_6_grasp_signal)
  # E3: CheckRobotState require_attached (the only grasp verification today)
  trainit_m26_test(test_m2_6_check_robot_state 97 300 test_m2_6_check_robot_state)
  # E4: SetReleasePolicy's function-static publisher binds the first topic
  trainit_m26_test(test_m2_6_release_policy 97 120 test_m2_6_release_policy)
  # E5: RunPolicy against a fake policy runtime
  trainit_m26_test(test_m2_6_run_policy 97 300 test_m2_6_run_policy)
  # E6: TaskRunner's GRIPPER step, on the M2.5 rig
  trainit_m26_test(test_m2_6_task_runner 97 600 test_m2_6_task_runner)
  # E7: the real trainit_run_bt as a child process (domain 88)
  trainit_m26_test(test_m2_6_runner_child 88 600 test_m2_6_runner_child)
  # E8: the real scene_manager_node as a child process (domain 89)
  trainit_m26_test(test_m2_6_scene_manager_child 89 600 test_m2_6_scene_manager_child)

  # vNext M2.6 AFTER (ADR-0022, the canonical end-effector contract, TMR 2.0.0). The shared
  # conformance data (test/data/m2_6_contract/) are BYTE COPIES pinned by sha256: the fixture,
  # its expected dump, the token file and the reference normaliser, plus this repository's
  # mutation table (every expected column computed by the reference) and the canonical
  # bin-picking tree TSA will pin its generated bytes to. Harness helpers: test/m2_6/eef_after_*.hpp.
  set(_m26c ${CMAKE_CURRENT_SOURCE_DIR}/test/data/m2_6_contract)
  function(trainit_m26_pure_test name timeout)
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/m2_6_actual)
    ament_add_gtest(${name} test/${name}.cpp TIMEOUT ${timeout})
    target_link_libraries(${name} trainit_motion_runtime)
    link_btcpp(${name})
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
    target_compile_definitions(${name} PRIVATE
      TRAINIT_M26_CONTRACT_DIR="${_m26c}"
      TRAINIT_M26_SCRATCH_DIR="${CMAKE_CURRENT_BINARY_DIR}/m2_6_actual"
      TRAINIT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
      # the M2.5 harness header these include for sha256 (test/m2_5/capture.hpp), unchanged
      TRAINIT_M25_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data/m2_5"
      TRAINIT_M25_ACTUAL_DIR="${CMAKE_CURRENT_BINARY_DIR}/m2_5_actual"
      TRAINIT_MOVEIT_VERSION="${moveit_ros_planning_interface_VERSION}"
      TRAINIT_BTCPP_VERSION="${behaviortree_cpp_VERSION}")
  endfunction()
  # M1: the parser == the reference normaliser (fixture dump, every refusal row), the token parity
  trainit_m26_pure_test(test_m2_6_eef_contract 60)
  # M1, the reference half: the byte copy of normalize_eef.py re-computes the mutation table
  ament_add_pytest_test(test_m2_6_eef_reference test/test_m2_6_eef_reference.py TIMEOUT 60
    ENV TRAINIT_M26_CONTRACT_DIR=${_m26c})
  # M2: the actuator outcome mapping, every row of ADR-0022 D2 (pure)
  trainit_m26_pure_test(test_m2_6_eef_outcome 60)
  # M3: the VerifyGrasp judge, single- and multi-site (pure)
  trainit_m26_pure_test(test_m2_6_eef_verify 60)
  # M7: static scans of the new sources, and the legacy tick bodies textually unchanged
  trainit_m26_pure_test(test_m2_6_eef_static_scan 30)
  # M4: the R1/R2-like fixture ticked in /r1 and /r2 at once, against fakes (domain 97)
  trainit_m26_test(test_m2_6_eef_fixture 97 900 test_m2_6_eef_fixture)
  target_compile_definitions(test_m2_6_eef_fixture PRIVATE TRAINIT_M26_CONTRACT_DIR="${_m26c}"
    TRAINIT_M26_SCRATCH_DIR="${CMAKE_CURRENT_BINARY_DIR}/m2_6_actual")
  # M5: the canonical bin-picking loop, ticked as XML (domain 97)
  trainit_m26_test(test_m2_6_eef_bin_picking 97 900 test_m2_6_eef_bin_picking)
  target_compile_definitions(test_m2_6_eef_bin_picking PRIVATE TRAINIT_M26_CONTRACT_DIR="${_m26c}"
    TRAINIT_M26_SCRATCH_DIR="${CMAKE_CURRENT_BINARY_DIR}/m2_6_actual")
  # vNext M2.7 (owner Q8.2, tests BEFORE): the canonical bin-picking loop with the REAL MoveWaypoint
  # against the M2.5 fake move_group and the M2.6 fake end effector (test/m2_7/): the M2.6 tree on
  # production adf94b0, frozen in test/data/m2_7_mf/ (README.md). Domain 97, the M2.6 in-process lock;
  # its captures are written to their own directory (m2_7_mf_actual). Since MF-P1 (TMR-7) also the
  # M2.7 tree (the AFTER capture, frozen on that commit) and the exact BEFORE -> AFTER diff.
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/m2_7_mf_actual)
  trainit_m26_test(test_m2_7_bin_picking_motion 97 900 test_m2_7_bin_picking_motion)
  target_compile_definitions(test_m2_7_bin_picking_motion PRIVATE TRAINIT_M26_CONTRACT_DIR="${_m26c}"
    TRAINIT_M26_SCRATCH_DIR="${CMAKE_CURRENT_BINARY_DIR}/m2_7_mf_actual"
    TRAINIT_M27_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data/m2_7_mf"
    TRAINIT_M27_ACTUAL_DIR="${CMAKE_CURRENT_BINARY_DIR}/m2_7_mf_actual")
  # vNext M2.7 (owner Q8.2, MF-P1): whose failure a failed motion is. Pure (no ROS): the classification
  # table of motion_failure.hpp, the fault text, every error code of the installed moveit_msgs named,
  # MoveWaypoint's ports (its manifest and the Groot palette)
  ament_add_gtest(test_m2_7_motion_failure test/test_m2_7_motion_failure.cpp TIMEOUT 30)
  target_link_libraries(test_m2_7_motion_failure trainit_motion_runtime)
  link_btcpp(test_m2_7_motion_failure)
  target_compile_definitions(test_m2_7_motion_failure PRIVATE
    TRAINIT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    TRAINIT_MOVEIT_ERROR_CODES_MSG="${moveit_msgs_DIR}/../msg/MoveItErrorCodes.msg")
  # ... and MoveWaypoint's optional `fault` ticked for real on the M2.5 rig, domain 81 (INTEGRATION 3.2):
  # MF-01..MF-31, each with and without fault= (identical MoveIt events); its report is evidence only
  trainit_m25_test(test_m2_7_motion_fault 81 600 test_m2_7_motion_fault)
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/m2_7_mf_actual/motion_fault)
  target_compile_definitions(test_m2_7_motion_fault PRIVATE
    TRAINIT_M27_MF_REPORT_DIR="${CMAKE_CURRENT_BINARY_DIR}/m2_7_mf_actual/motion_fault")
  # M6: the real trainit_run_bt as a child: exit 2, exit 3, the pinned lines (domain 88)
  trainit_m26_test(test_m2_6_eef_runner_child 88 900 test_m2_6_eef_runner_child)
  target_compile_definitions(test_m2_6_eef_runner_child PRIVATE TRAINIT_M26_CONTRACT_DIR="${_m26c}"
    TRAINIT_M26_SCRATCH_DIR="${CMAKE_CURRENT_BINARY_DIR}/m2_6_actual")
  # vNext M2.7 (WP_NS T7, owner section 12), domain 90: the real trainit_run_bt in the root, /r1,
  # /r2 and /r3 one at a time, then in /r1 + /r2 at once; a fake move_group serves every namespace
  trainit_m26_test(test_m2_7_namespace_runner 90 900 test_m2_7_namespace_runner)
  # ... and its T9 cases: a runner whose move_group is elsewhere names what it waits for and exits 2;
  # a remap that sends a robot's MoveIt, scene or joint-state name (a service, a topic, an action's
  # internal entity) out of its namespace is refused at once (ADR-0023 Decision 5, NS-G)
  # vNext M2.7 (WP_NS T8, owner decision D1), domain 90 too (the same lock): the runners' MoveIt
  # start-up wait, the real trainit_run_bt and trainit_run_task as children, MoveIt absent, partial,
  # late or up; the 60 s default and a stopped sim clock (INTEGRATION G-2)
  trainit_m26_test(test_m2_7_moveit_wait 90 600 test_m2_7_moveit_wait)
  target_compile_definitions(test_m2_7_moveit_wait PRIVATE
    TRAINIT_RUN_TASK_EXE="$<TARGET_FILE:trainit_run_task>"
    TRAINIT_M27_TASK_FILE="${CMAKE_CURRENT_SOURCE_DIR}/test/data/m2_5_task.yaml")
  add_dependencies(test_m2_7_moveit_wait trainit_run_task)
  # the pytest registration guard above, for the new suite too
  if(NOT TEST test_m2_6_eef_reference)
    message(WARNING "[trainit_motion_runtime] test_m2_6_eef_reference is NOT registered: pytest could "
                    "not start at configure time (reconfigure with PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)")
    add_test(NAME test_m2_6_eef_reference_was_not_registered COMMAND ${CMAKE_COMMAND} -E false)
  endif()
  # vNext M2.7 (owner decision 11.2, tests BEFORE): the four planning-scene BT nodes, captured
  # against the M2.5 fakes and compared byte for byte with test/data/m2_7/, recorded on production
  # adf94b0 before the BtContext/TaskRunner narrowing touched a production line. Private domain
  # (set IN CODE and here), localhost only, graph-guarded; one binary at a time per domain.
  function(trainit_m27_test name domain timeout source)
    ament_add_gtest(${name} test/${source}.cpp TIMEOUT ${timeout} SKIP_LINKING_MAIN_LIBRARIES
      ENV ROS_DOMAIN_ID=${domain} ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp)
    target_link_libraries(${name} trainit_motion_runtime)
    link_btcpp(${name})
    ament_target_dependencies(${name} sensor_msgs)
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/m2_7_actual)
    target_compile_definitions(${name} PRIVATE
      TRAINIT_M27_DOMAIN=${domain}
      TRAINIT_M27_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data/m2_7"
      TRAINIT_M27_ACTUAL_DIR="${CMAKE_CURRENT_BINARY_DIR}/m2_7_actual"
      # the M2.5 harness headers this one includes (test/m2_5/), unchanged; its own families
      # (when a binary replays one) are compared with test/data/m2_5 and written apart
      TRAINIT_M25_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data/m2_5"
      TRAINIT_M25_ACTUAL_DIR="${CMAKE_CURRENT_BINARY_DIR}/m2_7_actual"
      TRAINIT_M25_TASKS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data"
      TRAINIT_M25_PATHS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data/m2_5_paths"
      TRAINIT_PROFILES_FILE="${CMAKE_CURRENT_SOURCE_DIR}/config/bt/trainit_profiles.yaml"
      TRAINIT_MOVEIT_VERSION="${moveit_ros_planning_interface_VERSION}"
      TRAINIT_BTCPP_VERSION="${behaviortree_cpp_VERSION}")
    set_tests_properties(${name} PROPERTIES RESOURCE_LOCK trainit_m27_domain_${domain})
  endfunction()
  trainit_m27_test(test_m2_7_scene_nodes 82 300 test_m2_7_scene_nodes)
  # the same nodes in a process remapped to /r3 (the namespaced launch's way): which fake set they reach
  trainit_m27_test(test_m2_7_scene_nodes_remap 82 300 test_m2_7_scene_nodes)
  target_compile_definitions(test_m2_7_scene_nodes_remap PRIVATE TRAINIT_M27_REMAP)
  # AFTER (owner decision 11.2): the door's types, the task_runner family through the new
  # TaskRunner constructor, the BT nodes on a fake runtime (same domain, one binary at a time)
  trainit_m27_test(test_m2_7_bt_interface 82 600 test_m2_7_bt_interface)

  # vNext M3.1 (ADR-0024): FollowPath and MoveWaypoint's additive keys ticked for real against the
  # M2.5 fake move_group (domain 83): what each path sends through the door, the refusals that send
  # nothing, and the comparison with the committed AFTER family (test/data/m3_1/captures/).
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/m3_1_actual)
  trainit_m25_test(test_m3_1_follow_path 83 600 test_m3_1_follow_path)
  target_compile_definitions(test_m3_1_follow_path PRIVATE
    TRAINIT_M31_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data/m3_1"
    TRAINIT_M31_ACTUAL_DIR="${CMAKE_CURRENT_BINARY_DIR}/m3_1_actual")
  ament_target_dependencies(test_m3_1_follow_path tf2_ros)

  # vNext M3.4 (ADR-0015 amendment M3.4, owner decisions D3 and D8): every hardened behaviour ticked
  # for real against the M2.5 fake move_group (domain 84), and SIGINT reaching the real trainit_run_bt
  # in the middle of a motion (domain 85, a child process). Both record the SAME scenarios BEFORE
  # (TMR 2.2.0, 9418694: test/data/m3_4/before/) and AFTER (test/data/m3_4/after/).
  # TRAINIT_M34_AFTER: built for the M3.4 production, they compare with the AFTER and assert it.
  trainit_m25_test(test_m3_4_execution 84 900 test_m3_4_execution)
  target_compile_definitions(test_m3_4_execution PRIVATE TRAINIT_M34_AFTER
    TRAINIT_M34_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data/m3_4"
    TRAINIT_M31_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data/m3_1")
  ament_target_dependencies(test_m3_4_execution tf2_ros)
  trainit_m26_test(test_m3_4_runner_stop 85 300 test_m3_4_runner_stop)
  target_compile_definitions(test_m3_4_runner_stop PRIVATE TRAINIT_M34_AFTER
    TRAINIT_M34_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data/m3_4")
  # vNext M3.5 (ADR-0024, M3_DESIGN §9): process-path semantics. FollowPath from= to= (the event-free runs of
  # a path) against the M2.5 fake move_group (domain 70, an AFTER record: test/data/m3_5/captures/);
  # DetectObject trigger="true" (a capture) against a fake on-demand detector (domain 71); the generic
  # ProcessEvent node and the ROS process controller's retained publishers (domain 72).
  trainit_m25_test(test_m3_5_follow_path_range 70 600 test_m3_5_follow_path_range)
  target_compile_definitions(test_m3_5_follow_path_range PRIVATE
    TRAINIT_M35_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data/m3_5")
  ament_target_dependencies(test_m3_5_follow_path_range tf2_ros)
  ament_add_gtest(test_m3_5_detect_trigger test/test_m3_5_detect_trigger.cpp TIMEOUT 120
    ENV ROS_DOMAIN_ID=71 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp)
  target_link_libraries(test_m3_5_detect_trigger trainit_motion_runtime)
  link_btcpp(test_m3_5_detect_trigger)
  ament_add_gtest(test_m3_5_process_event test/test_m3_5_process_event.cpp TIMEOUT 120
    ENV ROS_DOMAIN_ID=72 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp)
  target_link_libraries(test_m3_5_process_event trainit_motion_runtime)
  link_btcpp(test_m3_5_process_event)
  # vNext M3.6 (ADR-0026): the grasp-candidate contract. Pure (no ROS graph): validation, the Detection3D
  # projection and its hemisphere, align_approach; the two nodes against in-process producers (domain 73).
  ament_add_gtest(test_m3_6_grasp_candidates test/test_m3_6_grasp_candidates.cpp TIMEOUT 60)
  target_link_libraries(test_m3_6_grasp_candidates trainit_motion_runtime)
  ament_add_gtest(test_m3_6_candidate_nodes test/test_m3_6_candidate_nodes.cpp TIMEOUT 180
    ENV ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp)
  target_link_libraries(test_m3_6_candidate_nodes trainit_motion_runtime)
  link_btcpp(test_m3_6_candidate_nodes)
  # vNext M3.6 / M3-D (ADR-0026): Validation C -- the trees TSA generates (byte copies), ticked with the REAL
  # nodes against the M2.5 fake move_group and the M2.6 fake end effector, with two synthetic producers
  # (domain 74, an AFTER record: test/data/m3_6/captures/).
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/m3_6_actual)
  trainit_m26_test(test_m3_6_validation_c 74 600 test_m3_6_validation_c)
  target_compile_definitions(test_m3_6_validation_c PRIVATE TRAINIT_M26_CONTRACT_DIR="${_m26c}"
    TRAINIT_M26_SCRATCH_DIR="${CMAKE_CURRENT_BINARY_DIR}/m3_6_actual"
    TRAINIT_M36_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data/m3_6")
  # ... pure (no ROS graph): every BEFORE -> AFTER difference M3.4 made, exactly, scenario by scenario
  # (the new families and the legacy ones moved into test/data/m3_4/legacy_after/)
  ament_add_gtest(test_m3_4_capture_diff test/test_m3_4_capture_diff.cpp TIMEOUT 60)
  target_link_libraries(test_m3_4_capture_diff trainit_motion_runtime)
  target_include_directories(test_m3_4_capture_diff PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
  target_compile_definitions(test_m3_4_capture_diff PRIVATE
    TRAINIT_M25_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data/m2_5"
    TRAINIT_M25_ACTUAL_DIR="${CMAKE_CURRENT_BINARY_DIR}/m2_5_actual"
    TRAINIT_M27_NS_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data/m2_7_ns"
    TRAINIT_M34_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data/m3_4"
    TRAINIT_MOVEIT_VERSION="${moveit_ros_planning_interface_VERSION}"
    TRAINIT_BTCPP_VERSION="${behaviortree_cpp_VERSION}")
endif()

ament_package()
