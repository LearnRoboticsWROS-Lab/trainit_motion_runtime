# =============================================================================
# trainit_moveit_config.py — assemble a complete MoveIt configuration from a
# robot-specific moveit_config package PLUS the framework's GENERIC planning
# pipelines (those TMR's planner registry loads: today ompl / pilz / chomp). This is
# how a robot application "pulls" the moveit pipelines from the TrainIt Motion
# Runtime framework: it owns only the robot-specific data (URDF/SRDF/limits/
# controllers); the planners come here.
#
# Usage in a launch file:
#   from trainit_motion_runtime.trainit_moveit_config import build_move_group_params
#   params, mc = build_move_group_params(use_isaac=True)
#   Node(package="moveit_ros_move_group", executable="move_group", parameters=[params, {"use_sim_time": True}])
# =============================================================================
import os

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

# The planner registry this helper reads (config/planner_profiles.yaml, ADR-0021).
# The registry schemas this helper reads. Schema 2 (vNext M3.1, ADR-0025) is schema 1 plus the
# provider capability sections, which this helper does not need: it reads only the sections both
# schemas share. Any other number is refused, never half-read.
REGISTRY_SCHEMA_VERSIONS = (1, 2)
REGISTRY_SCHEMA_VERSION = 2   # the schema TMR's own registry is written in


def _load_yaml(path):
    with open(path, "r") as handle:
        return yaml.safe_load(handle)


def planner_registry_path() -> str:
    try:
        share = get_package_share_directory("trainit_motion_runtime")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "cannot read TMR's planner registry: trainit_motion_runtime must be installed and "
            "its workspace sourced before this helper is imported") from exc
    return os.path.join(share, "config", "planner_profiles.yaml")


def planner_registry() -> dict:
    """TMR's planner registry, as data: which pipelines exist, which are loaded, the modes.

    Read from this package's installed share (found through the ament index), never from a
    variable of the launch that imports this module.
    """
    path = planner_registry_path()
    registry = _load_yaml(path) or {}
    version = registry.get("schema_version")
    if version not in REGISTRY_SCHEMA_VERSIONS:
        raise RuntimeError(
            f"{path}: planner registry schema_version {version!r}; "
            f"this trainit_moveit_config reads schemas "
            f"{', '.join(str(v) for v in REGISTRY_SCHEMA_VERSIONS)}")
    return registry


_REGISTRY = planner_registry()

# Generic, robot-agnostic pipeline configs shipped by this framework: every registry
# pipeline that has a parameters file.
PIPELINE_FILES = {name: spec["parameters"]
                  for name, spec in _REGISTRY["pipelines"].items() if spec.get("parameters")}

# The default loaded set, in registry order: what move_group loads, and what the
# runtime's availability guard is told is available.
DEFAULT_PIPELINES = tuple(name for name, spec in _REGISTRY["pipelines"].items()
                          if spec.get("loaded"))

DEFAULT_PIPELINE = _REGISTRY["default_pipeline"]

# move_group capability plugins required for the Pilz sequence (process path) backend.
PILZ_SEQUENCE_CAPABILITIES = (
    "pilz_industrial_motion_planner/MoveGroupSequenceAction "
    "pilz_industrial_motion_planner/MoveGroupSequenceService"
)


def generic_pipelines_dir():
    return os.path.join(
        get_package_share_directory("trainit_motion_runtime"), "config", "pipelines")


def generic_pipeline_params(pipelines=DEFAULT_PIPELINES):
    """Return {pipeline_name: <pipeline params dict>} from the framework configs."""
    pipe_dir = generic_pipelines_dir()
    params = {}
    for name in pipelines:
        if name not in PIPELINE_FILES:
            raise ValueError(f"unknown generic pipeline '{name}'")
        params[name] = _load_yaml(os.path.join(pipe_dir, PIPELINE_FILES[name]))
    return params


def _hardware_mappings(use_isaac=False, backend=None):
    """The ros2_control hardware-plugin xacro args for a backend (ADR-0008).

    Backward compatible: with ``backend=None`` this returns exactly the legacy
    ``{"use_isaac": ...}`` mapping (so non-gazebo bundles/xacros are unchanged). When a
    ``backend`` token is given it also sets ``use_gazebo`` — a gazebo-capable xacro selects
    ``gazebo_ros2_control/GazeboSystem`` on ``use_gazebo``, ``TopicBasedSystem`` on
    ``use_isaac``, else ``mock_components/GenericSystem``.
    """
    if backend is None:
        return {"use_isaac": "true" if use_isaac else "false"}
    b = str(backend)
    return {"use_isaac": "true" if b == "isaac" else "false",
            "use_gazebo": "true" if b == "gazebo" else "false"}


def build_moveit_configs(robot_name="fr3wml",
                         moveit_config_package="fr3wml_app_moveit_config",
                         use_isaac=False,
                         backend=None):
    """Robot-specific MoveIt configs (description/semantic/kinematics/limits/controllers).

    ``backend`` (ADR-0008): a backend token (mock|isaac|gazebo|real) selecting the
    ros2_control hardware plugin via the xacro. Legacy ``use_isaac`` still works when
    ``backend`` is omitted.
    """
    return (
        MoveItConfigsBuilder(robot_name, package_name=moveit_config_package)
        .robot_description(mappings=_hardware_mappings(use_isaac, backend))
        .to_moveit_configs()
    )


# Trajectory-execution defaults. MoveIt's built-in allowed_start_tolerance is only
# 0.01 rad: in a LIVE sim (Isaac) or on hardware the reported start state micro-drifts
# (gravity, imperfect drive gains) away from the plan's first point BETWEEN "Plan" and
# "Execute", so move_group aborts a perfectly valid plan with "start point deviates from
# current robot state more than 0.01" — the classic "it plans but won't execute". These
# values tolerate that drift and give slow/convoluted tight-space trajectories enough
# time so duration monitoring doesn't abort them mid-move.
DEFAULT_TRAJECTORY_EXECUTION = {
    "moveit_manage_controllers": True,
    # ~5.7 deg/joint of slack on the start state (Isaac drift is far smaller). Set 0.0
    # to disable the check entirely if a plan must ALWAYS execute regardless of drift.
    "trajectory_execution.allowed_start_tolerance": 0.1,
    # allow the actual move to take up to 3x the planned time before timing out (slow
    # tight-space paths at low velocity scaling + Isaac tracking lag).
    "trajectory_execution.allowed_execution_duration_scaling": 3.0,
    "trajectory_execution.allowed_goal_duration_margin": 2.0,
    "trajectory_execution.execution_duration_monitoring": True,
    "trajectory_execution.wait_for_trajectory_completion": True,
}


def build_move_group_params(robot_name="fr3wml",
                            moveit_config_package="fr3wml_app_moveit_config",
                            use_isaac=False,
                            backend=None,
                            pipelines=DEFAULT_PIPELINES,
                            default_pipeline=DEFAULT_PIPELINE,
                            trajectory_execution=None):
    """Full move_group parameter dict (robot config + generic pipelines + Pilz sequence).

    Returns (params_dict, moveit_configs). The moveit_configs object is handy for
    robot_description / semantic / kinematics when configuring other nodes (RViz,
    robot_state_publisher, the application node).

    ``trajectory_execution``: optional dict overriding DEFAULT_TRAJECTORY_EXECUTION
    (e.g. ``{"trajectory_execution.allowed_start_tolerance": 0.0}`` to disable the
    start-state check).

    ``backend`` (ADR-0008): a backend token (mock|isaac|gazebo|real) selecting the
    ros2_control hardware plugin. Omit it to keep the legacy ``use_isaac`` behaviour.
    """
    moveit_configs = build_moveit_configs(robot_name, moveit_config_package, use_isaac, backend)
    params = moveit_configs.to_dict()

    params["planning_pipelines"] = list(pipelines)
    params["default_planning_pipeline"] = default_pipeline
    params.update(generic_pipeline_params(pipelines))

    # tolerant trajectory-execution params (override MoveIt/builder defaults last)
    te = dict(DEFAULT_TRAJECTORY_EXECUTION)
    if trajectory_execution:
        te.update(trajectory_execution)
    params.update(te)

    # Merge the robot-specific OMPL group settings (group -> planners + projection)
    # onto the framework's generic OMPL pipeline, if the robot package provides them.
    if "ompl" in params:
        robot_ompl = os.path.join(
            get_package_share_directory(moveit_config_package), "config", "ompl_planning.yaml")
        if os.path.exists(robot_ompl):
            overlay = _load_yaml(robot_ompl) or {}
            params["ompl"].update(overlay)

    # Ensure the Pilz sequence capability is loaded (process-path backend).
    params["capabilities"] = PILZ_SEQUENCE_CAPABILITIES

    return params, moveit_configs


def available_pipeline_names(pipelines=DEFAULT_PIPELINES):
    """The pipeline ids actually shipped (for the runtime's STOMP/CHOMP guard).

    Note: 'chomp' is mapped to itself; this list is what the C++ selector checks.
    """
    return list(pipelines)
