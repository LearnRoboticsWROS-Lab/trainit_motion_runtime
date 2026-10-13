# vNext M2.5: the MotionRuntime door names no MoveIt C++ API -- checked on the INCLUDE CLOSURE.
#
# The compiler's dependency file of test_m2_5_interface_purity.cpp (a unit that includes only
# motion_runtime.hpp) lists every header that unit really reads. None may be a MoveIt C++ header
# (a path with a /moveit/ directory, e.g. include/moveit_core/moveit/...); MoveIt MESSAGES
# (include/moveit_msgs/moveit_msgs/...) are the ROS contract and allowed. The positive control is
# the dependency file of a unit that does include MoveIt: it must contain /moveit/ paths, or this
# check would pass blind. A missing dependency file fails (fail closed).
#   cmake -DPURITY_DEPFILE=... -DCONTROL_DEPFILE=... -P interface_closure.cmake
foreach(var PURITY_DEPFILE CONTROL_DEPFILE)
  if(NOT EXISTS "${${var}}")
    message(FATAL_ERROR "no compiler dependency file at ${${var}} (${var}): cannot check the include closure")
  endif()
endforeach()
file(STRINGS "${CONTROL_DEPFILE}" control REGEX "/moveit/")
list(LENGTH control n_control)
if(n_control EQUAL 0)
  message(FATAL_ERROR "the positive control ${CONTROL_DEPFILE} names no /moveit/ header: the check is blind")
endif()
file(STRINGS "${PURITY_DEPFILE}" leaked REGEX "/moveit/")
list(LENGTH leaked n_leaked)
if(NOT n_leaked EQUAL 0)
  message(FATAL_ERROR "motion_runtime.hpp's include closure reaches MoveIt's C++ API:\n${leaked}")
endif()
file(STRINGS "${PURITY_DEPFILE}" all)
list(LENGTH all n_all)
message(STATUS "motion_runtime.hpp: ${n_all} dependency lines, 0 MoveIt C++ headers (control: ${n_control})")
