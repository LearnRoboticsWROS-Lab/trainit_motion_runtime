# Behavior Tree migration plan

BehaviorTree.CPP is **not** integrated yet (no dependency added). The runtime is
designed so the operations become BT leaf nodes with no rework.

## Operations → future BT nodes
Each maps to a `MotionRuntime` / controller call already implemented:

| BT node | Backing call |
|---|---|
| `MovePtp` | `MotionRuntime::movePtp` / `moveCollisionFree` (PILZ) |
| `MoveLinear` | `MotionRuntime::moveLinear` |
| `MoveCircular` | `MotionRuntime::moveCircular` |
| `MoveCollisionFree` | `MotionRuntime::moveCollisionFree` |
| `ExecuteProcessPath` | `executeCartesianProcess` / `executeProcessSequence` |
| `EnableProcess` / `DisableProcess` | `ProcessController::executeEvent` |
| `OpenGripper` / `CloseGripper` | `GripperController::command` |
| `AttachObject` / `DetachObject` | `PlanningSceneManager::attachBox/detachObject` |
| `ServoTrack` | `OnlineServoBackend` (future) |

## Node contract (already satisfied)
Every operation: explicit typed inputs, a typed `MotionResult`, no global state,
no hidden side effects, does not block indefinitely (action timeouts). Adding
cancellation is the main BT-specific addition.

## Migration steps (when ready)
1. Add `behaviortree_cpp` dependency to the **application** package (not the engine).
2. Wrap each call above in a `BT::SyncActionNode` / `StatefulActionNode`
   (async + cancellable), reading ports for targets/options.
3. Provide blackboard entries for named poses / paths (load from the same task
   YAML, or from the BT XML).
4. Author the tree (XML). Example:
```xml
<Sequence name="DispensingTask">
  <MoveCollisionFree target="{safe_start}"/>
  <MoveLinear        target="{process_start}"/>
  <EnableProcess     channel="glue"/>
  <ExecuteProcessPath path="{glue_path}"/>
  <DisableProcess    channel="glue"/>
  <MoveLinear        target="{safe_start}"/>
  <MoveCollisionFree target="{home}"/>
</Sequence>
```
The engine stays unchanged; the BT lives in the application bundle, alongside (or
instead of) the linear task YAML.
