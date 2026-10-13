// vNext M2.5 test harness: what every TMR motion consumer sends to MoveIt, captured against fakes.
//
// The MotionRuntime seam (M2.5) must not change a single request TMR sends to MoveIt, nor how a
// MoveIt answer becomes a BT status. The harness ticks REAL BT nodes (and TaskRunner) on a REAL
// MoveItMotionRuntime whose MoveGroupInterface, PlanningSceneInterface and PlanningSceneMonitor
// talk to in-process FAKES that record every goal and request and answer from a script:
//
//   capture.hpp          the deterministic text of a scenario, the family file, the watchdog
//   fake_move_group.hpp  FakeMoveGroup, FakeJointStates, FakeServo, FakeGripper
//   test_arm.hpp         the inline test arm, the rig, the scenario runner, the isolated main
//
// The committed captures (test/data/m2_5/) were recorded on TMR 1.8.0 (9cfb601) BEFORE the seam
// touched a production line; every later commit must reproduce them byte for byte.
#ifndef TRAINIT_MOTION_RUNTIME_TEST_M2_5_MOTION_HARNESS_HPP_
#define TRAINIT_MOTION_RUNTIME_TEST_M2_5_MOTION_HARNESS_HPP_

#include "capture.hpp"
#include "fake_move_group.hpp"
#include "test_arm.hpp"

#endif  // TRAINIT_MOTION_RUNTIME_TEST_M2_5_MOTION_HARNESS_HPP_
