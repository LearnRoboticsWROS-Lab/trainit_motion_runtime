// Generic scene loader + dynamic-object attach manager — robot- and
// application-agnostic. Loads a list of collision objects (box | cylinder |
// sphere | mesh) from YAML-style ROS parameters and injects them into the MoveIt
// planning scene, so move_group plans against the real cell instead of an empty
// world. It ALSO mirrors the gripper: on close it attaches the configured dynamic
// objects (e.g. bottles) to the tool link as AttachedCollisionObjects (they turn
// purple in RViz and follow the arm), on open it detaches them (they stay where
// released). The same node is used by the config-phase scene_loader launch and by
// the generated bundle's bringup, so configuration == runtime.
//
// Ported/trimmed from industrial_bt_framework/scene_manager_node.cpp.
//
// Params (node name: scene_manager_node):
//   frame_id            (string, default base_link)
//   force_republish_hz  (double, default 1.0)   re-assert dynamic objects
//   scene_update_wait_ms(int,    default 800)
//   attach_acm_links    (string[]) links allowed to collide with everything
//   object_ids          (string[]) ids to load
//   objects.<id>.type            box|cylinder|sphere|mesh
//   objects.<id>.size            [x,y,z]        (box)
//   objects.<id>.radius/height   (cylinder/sphere)
//   objects.<id>.mesh_path       package://...stl|obj  (mesh)
//   objects.<id>.scale           [sx,sy,sz]     (mesh, default 1,1,1)
//   objects.<id>.position        [x,y,z]
//   objects.<id>.orientation     [x,y,z,w]
//   objects.<id>.dynamic         (bool) manipulation target: re-asserted at
//                                force_republish_hz AND ACM-allowed vs everything
//                                (shown but collision-transparent, since the
//                                gripper / the crate it rests on must touch it).
//   --- gripper-triggered attach (dynamic objects become part of the EE) ---
//   gripper_cmd_topic   (string, default /isaac_gripper_cmd)  Bool, true=close
//   scene_reset_topic   (string, default /isaac_scene_reset)   Bool, latched. On
//                                ~/reset_scene a RISING EDGE (true then false ~300 ms
//                                later) is emitted so the hand-authored Isaac adapter
//                                can teleport its dynamic prims home. Trigger on the
//                                TRANSITION, never on the level.
//   attach_link         (string, default tcp)   link the objects attach to
//   touch_links         (string[]) robot links allowed to touch the attached objs
//   attach_object_ids   (string[]) dynamic ids that attach on gripper close
//   attached_collision_check (bool, default true) per-move flag: ON => attached
//                                objects are CHECKED vs the static/actuated scene
//                                meshes (extension of the EE, the planner routes
//                                around them); OFF => transparent to everything.
//                                Always allowed vs the gripper (touch_links) and
//                                vs the other dynamic objects (crate, bottles).
//                                Flip at runtime with the SetBool service
//                                ~/attached_collision_check.

#include <map>
#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/allowed_collision_entry.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <boost/variant/get.hpp>

#include <Eigen/Geometry>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

using ApplySceneClient = rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>;
using GetSceneClient   = rclcpp::Client<moveit_msgs::srv::GetPlanningScene>;
using Acm              = moveit_msgs::msg::AllowedCollisionMatrix;

namespace trainit_scene {

struct SceneObjectSpec
{
  std::string id;
  std::string type;            // "box" | "cylinder" | "sphere" | "mesh"
  std::vector<double> size;    // box
  double radius = 0.0;
  double height = 0.0;
  std::string mesh_path;
  std::vector<double> scale = {1.0, 1.0, 1.0};
  std::vector<double> position    = {0.0, 0.0, 0.0};
  std::vector<double> orientation = {0.0, 0.0, 0.0, 1.0};
  bool dynamic = false;
  std::vector<double> grasp_box;          // AABB extents (grasp target, attach_box mode)
  std::vector<double> grasp_box_center;   // local AABB centre offset
};

class SceneManagerNode : public rclcpp::Node
{
public:
  SceneManagerNode() : Node("scene_manager_node")
  {
    declareParams();

    client_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    apply_client_ = create_client<moveit_msgs::srv::ApplyPlanningScene>(
      "/apply_planning_scene", rmw_qos_profile_services_default, client_cb_group_);
    get_scene_client_ = create_client<moveit_msgs::srv::GetPlanningScene>(
      "/get_planning_scene", rmw_qos_profile_services_default, client_cb_group_);
    scene_pub_ = create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);

    // gripper mirror: subscribe to the close/open signal + expose the flag service
    const auto grip_topic = get_parameter("gripper_cmd_topic").as_string();
    rclcpp::QoS latched(1); latched.transient_local();
    gripper_sub_ = create_subscription<std_msgs::msg::Bool>(
      grip_topic, latched,
      std::bind(&SceneManagerNode::onGripperCmd, this, std::placeholders::_1));
    check_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/attached_collision_check",
      std::bind(&SceneManagerNode::onSetCheck, this,
                std::placeholders::_1, std::placeholders::_2));

    // Scene reset (sim): put every DYNAMIC object back at its INITIAL pose — the
    // cycle boundary of a looping application (cycle 2 must pick where cycle 1 did).
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/reset_scene",
      std::bind(&SceneManagerNode::onResetScene, this,
                std::placeholders::_1, std::placeholders::_2));
    // Seam for the Isaac side: the sim adapter can subscribe and teleport its prims
    // back too (latched so a late-joining Isaac still sees the last reset).
    rclcpp::QoS reset_qos(1); reset_qos.transient_local();
    reset_pub_ = create_publisher<std_msgs::msg::Bool>(
      get_parameter("scene_reset_topic").as_string(), reset_qos);

    // TF: to place a grasped object relative to the tool at grasp time and re-place it
    // at the tool pose on release (both grasp modes).
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    init_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&SceneManagerNode::loadInitialScene, this));

    // Re-assert dynamic objects periodically ONLY if force_republish_hz > 0. Re-adding
    // the (mesh) objects every second is heavy and needless once they are loaded, and
    // the frequent /planning_scene churn interferes with RViz "Plan & Execute" (plans
    // but silently does not execute). Default off; the objects persist from the initial
    // load and MoveIt re-adds detached objects itself.
    const double hz = get_parameter("force_republish_hz").as_double();
    if (hz > 0.0) {
      force_timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / hz)),
        std::bind(&SceneManagerNode::reassertDynamic, this));
    }
  }

private:
  void declareParams()
  {
    declare_parameter<std::string>("frame_id", "base_link");
    declare_parameter<double>("force_republish_hz", 1.0);
    declare_parameter<int>("scene_update_wait_ms", 800);
    declare_parameter<std::string>("gripper_cmd_topic", "/isaac_gripper_cmd");
    declare_parameter<std::string>("scene_reset_topic", "/isaac_scene_reset");
    declare_parameter<std::string>("attach_link", "tcp");
    // how a grasped object is represented while held: "remove" (disappears on grasp,
    // reappears at the tool on release — meshes stay meshes) | "attach_box" (attaches
    // as its AABB box so attached_collision_check works).
    declare_parameter<std::string>("grasp_attach_mode", "remove");
    // default OFF: at the pick the crate (and its bottles) sit ON the belt, so an
    // attached bottle is in contact with the belt mesh -> with the check ON the start
    // state is invalid and the arm can't move. OFF = transparent so the arm always
    // moves; turn it ON per-move (service) for a transfer where the held bottles must
    // avoid the prewash, once they are clear of the belt/crate.
    declare_parameter<bool>("attached_collision_check", false);
    // dynamic typing so an empty YAML list ([]) doesn't fail the typed declare.
    rcl_interfaces::msg::ParameterDescriptor dyn; dyn.dynamic_typing = true;
    declare_parameter("attach_acm_links", rclcpp::ParameterValue(std::vector<std::string>{}), dyn);
    declare_parameter("object_ids", rclcpp::ParameterValue(std::vector<std::string>{}), dyn);
    declare_parameter("attach_object_ids", rclcpp::ParameterValue(std::vector<std::string>{}), dyn);
    declare_parameter("touch_links",
      rclcpp::ParameterValue(std::vector<std::string>{"end_effector", "tcp", "wrist3_link"}), dyn);
  }

  std::vector<std::string> getStringArray(const std::string & name)
  {
    const auto p = get_parameter(name);
    return (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
      ? p.as_string_array() : std::vector<std::string>{};
  }

  bool loadObjectSpec(const std::string & id, SceneObjectSpec & out)
  {
    out.id = id;
    const std::string prefix = "objects." + id + ".";
    declare_parameter<std::string>(prefix + "type", "");
    out.type = get_parameter(prefix + "type").as_string();
    declare_parameter<std::vector<double>>(prefix + "size", std::vector<double>{});
    out.size = get_parameter(prefix + "size").as_double_array();
    declare_parameter<double>(prefix + "radius", 0.0);
    out.radius = get_parameter(prefix + "radius").as_double();
    declare_parameter<double>(prefix + "height", 0.0);
    out.height = get_parameter(prefix + "height").as_double();
    declare_parameter<std::string>(prefix + "mesh_path", "");
    out.mesh_path = get_parameter(prefix + "mesh_path").as_string();
    declare_parameter<std::vector<double>>(prefix + "scale", std::vector<double>{1.0, 1.0, 1.0});
    out.scale = get_parameter(prefix + "scale").as_double_array();
    declare_parameter<std::vector<double>>(prefix + "position", std::vector<double>{0.0, 0.0, 0.0});
    out.position = get_parameter(prefix + "position").as_double_array();
    declare_parameter<std::vector<double>>(prefix + "orientation",
      std::vector<double>{0.0, 0.0, 0.0, 1.0});
    out.orientation = get_parameter(prefix + "orientation").as_double_array();
    declare_parameter<bool>(prefix + "dynamic", false);
    out.dynamic = get_parameter(prefix + "dynamic").as_bool();
    declare_parameter<std::vector<double>>(prefix + "grasp_box", std::vector<double>{});
    out.grasp_box = get_parameter(prefix + "grasp_box").as_double_array();
    declare_parameter<std::vector<double>>(prefix + "grasp_box_center", std::vector<double>{});
    out.grasp_box_center = get_parameter(prefix + "grasp_box_center").as_double_array();
    return !out.type.empty();
  }

  geometry_msgs::msg::Pose makePose(const std::vector<double> & p, const std::vector<double> & q)
  {
    geometry_msgs::msg::Pose pose;
    if (p.size() == 3) { pose.position.x = p[0]; pose.position.y = p[1]; pose.position.z = p[2]; }
    if (q.size() == 4) {
      pose.orientation.x = q[0]; pose.orientation.y = q[1];
      pose.orientation.z = q[2]; pose.orientation.w = q[3];
    } else { pose.orientation.w = 1.0; }
    return pose;
  }

  bool buildCollisionObject(const SceneObjectSpec & spec, moveit_msgs::msg::CollisionObject & out)
  {
    out.header.frame_id = frame_id_;
    out.id = spec.id;
    out.operation = moveit_msgs::msg::CollisionObject::ADD;
    const auto pose = makePose(spec.position, spec.orientation);

    if (spec.type == "box") {
      if (spec.size.size() != 3) {
        RCLCPP_ERROR(get_logger(), "object '%s': box.size must have 3 values.", spec.id.c_str());
        return false;
      }
      shape_msgs::msg::SolidPrimitive prim;
      prim.type = shape_msgs::msg::SolidPrimitive::BOX;
      prim.dimensions = {spec.size[0], spec.size[1], spec.size[2]};
      out.primitives.push_back(prim);
      out.primitive_poses.push_back(pose);
      return true;
    }
    if (spec.type == "cylinder") {
      shape_msgs::msg::SolidPrimitive prim;
      prim.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      prim.dimensions = {spec.height, spec.radius};
      out.primitives.push_back(prim);
      out.primitive_poses.push_back(pose);
      return true;
    }
    if (spec.type == "sphere") {
      shape_msgs::msg::SolidPrimitive prim;
      prim.type = shape_msgs::msg::SolidPrimitive::SPHERE;
      prim.dimensions = {spec.radius};
      out.primitives.push_back(prim);
      out.primitive_poses.push_back(pose);
      return true;
    }
    if (spec.type == "mesh") {
      if (spec.mesh_path.empty()) {
        RCLCPP_WARN(get_logger(), "object '%s': mesh_path empty, skipping.", spec.id.c_str());
        return false;
      }
      const Eigen::Vector3d scale(
        spec.scale.size() >= 3 ? spec.scale[0] : 1.0,
        spec.scale.size() >= 3 ? spec.scale[1] : 1.0,
        spec.scale.size() >= 3 ? spec.scale[2] : 1.0);
      shapes::Mesh * mesh = shapes::createMeshFromResource(spec.mesh_path, scale);
      if (!mesh) {
        RCLCPP_ERROR(get_logger(), "object '%s': failed to load mesh '%s'.",
                     spec.id.c_str(), spec.mesh_path.c_str());
        return false;
      }
      shapes::ShapeMsg mesh_msg;
      shapes::constructMsgFromShape(mesh, mesh_msg);
      auto concrete = boost::get<shape_msgs::msg::Mesh>(mesh_msg);
      delete mesh;
      out.meshes.push_back(concrete);
      out.mesh_poses.push_back(pose);
      return true;
    }

    RCLCPP_ERROR(get_logger(), "object '%s': unknown type '%s'.",
                 spec.id.c_str(), spec.type.c_str());
    return false;
  }

  void publishDiffOnTopic(const std::vector<moveit_msgs::msg::CollisionObject> & objs)
  {
    moveit_msgs::msg::PlanningScene ps;
    ps.is_diff = true;
    ps.robot_state.is_diff = true;
    ps.world.collision_objects = objs;
    scene_pub_->publish(ps);
  }

  // Apply a PlanningScene diff via /apply_planning_scene (reliable + synchronous).
  bool applyScene(moveit_msgs::msg::PlanningScene ps, const std::string & label)
  {
    if (!apply_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "[%s] /apply_planning_scene unavailable.", label.c_str());
      return false;
    }
    ps.is_diff = true;
    ps.robot_state.is_diff = true;
    auto req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    req->scene = ps;
    auto fut = apply_client_->async_send_request(req);
    if (fut.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "[%s] timed out.", label.c_str());
      return false;
    }
    if (!fut.get()->success) {
      RCLCPP_ERROR(get_logger(), "[%s] returned FAILURE.", label.c_str());
      return false;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(scene_update_wait_ms_));
    return true;
  }

  bool applyDiffMany(const std::vector<moveit_msgs::msg::CollisionObject> & objs,
                     const std::string & label)
  {
    moveit_msgs::msg::PlanningScene ps;
    ps.world.collision_objects = objs;
    if (!applyScene(ps, label)) return false;
    RCLCPP_INFO(get_logger(), "[%s] OK (%zu objects).", label.c_str(), objs.size());
    return true;
  }

  // ---- ACM helpers ---------------------------------------------------------
  bool fetchAcm(Acm & out)
  {
    if (!get_scene_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "[ACM] /get_planning_scene unavailable.");
      return false;
    }
    auto greq = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    greq->components.components =
      moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto gfut = get_scene_client_->async_send_request(greq);
    if (gfut.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "[ACM] get_planning_scene timed out.");
      return false;
    }
    out = gfut.get()->scene.allowed_collision_matrix;
    if (out.entry_names.empty())
      RCLCPP_WARN(get_logger(), "[ACM] current ACM is empty; move_group may not be ready.");
    return true;
  }

  bool applyAcm(const Acm & acm)
  {
    moveit_msgs::msg::PlanningScene ps;
    ps.allowed_collision_matrix = acm;   // entry_names non-empty -> move_group applies it
    return applyScene(ps, "ACM");
  }

  static void setAcmDefault(Acm & acm, const std::string & name, bool value)
  {
    auto it = std::find(acm.default_entry_names.begin(), acm.default_entry_names.end(), name);
    if (it == acm.default_entry_names.end()) {
      acm.default_entry_names.push_back(name);
      acm.default_entry_values.push_back(value);
    } else {
      acm.default_entry_values[std::distance(acm.default_entry_names.begin(), it)] = value;
    }
  }

  static size_t ensureAcmName(Acm & acm, const std::string & name)
  {
    auto it = std::find(acm.entry_names.begin(), acm.entry_names.end(), name);
    if (it != acm.entry_names.end()) return std::distance(acm.entry_names.begin(), it);
    acm.entry_names.push_back(name);
    const size_t n = acm.entry_names.size();
    for (auto & row : acm.entry_values) row.enabled.push_back(false);
    moveit_msgs::msg::AllowedCollisionEntry row; row.enabled.assign(n, false);
    acm.entry_values.push_back(row);
    return n - 1;
  }

  // Explicit pairwise entry (overrides the defaults). true = allowed.
  static void setAcmEntry(Acm & acm, const std::string & a, const std::string & b, bool value)
  {
    const size_t i = ensureAcmName(acm, a), j = ensureAcmName(acm, b);
    acm.entry_values[i].enabled[j] = value;
    acm.entry_values[j].enabled[i] = value;
  }

  // Allow the named entities (dynamic object ids and/or robot links) to collide
  // with everything. move_group only applies an ACM diff when entry_names is
  // non-empty (a default-only diff is silently ignored), and a partial ACM would
  // WIPE the SRDF disables — so fetchAcm() returns the full current matrix, we add
  // a default entry (true) per name, and re-apply the full ACM.
  void allowCollisions(const std::vector<std::string> & allow_names)
  {
    Acm acm;
    if (!fetchAcm(acm)) return;
    for (const auto & name : allow_names) setAcmDefault(acm, name, true);
    if (!applyAcm(acm)) {
      RCLCPP_ERROR(get_logger(), "[ACM] failed to apply allowed-collision entries.");
      return;
    }
    RCLCPP_INFO(get_logger(),
                "[ACM] %zu entity(ies) allowed to collide with everything: %s",
                allow_names.size(), join(allow_names).c_str());
  }

  static std::string join(const std::vector<std::string> & v)
  {
    std::string s;
    for (size_t i = 0; i < v.size(); ++i) { s += v[i]; if (i + 1 < v.size()) s += ", "; }
    return s;
  }

  void loadInitialScene()
  {
    if (initial_loaded_) return;
    init_timer_->cancel();
    try {
      frame_id_ = get_parameter("frame_id").as_string();
      scene_update_wait_ms_ = get_parameter("scene_update_wait_ms").as_int();
      attach_link_ = get_parameter("attach_link").as_string();
      collision_check_ = get_parameter("attached_collision_check").as_bool();
      grasp_mode_ = get_parameter("grasp_attach_mode").as_string();
      touch_links_ = getStringArray("touch_links");
      attach_ids_ = getStringArray("attach_object_ids");
      const auto ids = getStringArray("object_ids");
      RCLCPP_INFO(get_logger(), "scene_manager: loading %zu object(s) (frame_id=%s).",
                  ids.size(), frame_id_.c_str());

      if (!ids.empty()) {
        moveit::planning_interface::PlanningSceneInterface psi;
        psi.removeCollisionObjects(ids);
        rclcpp::sleep_for(std::chrono::milliseconds(scene_update_wait_ms_));
      }

      std::vector<moveit_msgs::msg::CollisionObject> objs;
      std::vector<std::string> dynamic_ids;   // manipulation targets -> ACM-allowed
      {
        std::lock_guard<std::mutex> lock(scene_mutex_);
        static_ids_.clear();
        for (const auto & id : ids) {
          SceneObjectSpec spec;
          if (!loadObjectSpec(id, spec)) {
            RCLCPP_WARN(get_logger(), "Skipping malformed object '%s'.", id.c_str());
            continue;
          }
          moveit_msgs::msg::CollisionObject obj;
          if (!buildCollisionObject(spec, obj)) continue;
          objs.push_back(obj);
          if (spec.dynamic) {
            forced_[id] = obj;        // republished as a WORLD object at force_republish_hz
            dyn_objects_[id] = obj;   // persistent geometry, reused when attaching
            world_pose_[id] = objectPose(obj);          // current world pose (tracked)
            initial_pose_[id] = world_pose_[id];        // reset_scene restores THIS
            grasp_box_[id] = spec.grasp_box;            // AABB for attach_box mode
            grasp_box_center_[id] = spec.grasp_box_center;
            dynamic_ids.push_back(id);
          } else {
            static_ids_.push_back(id);   // checked meshes attached objs route around
          }
        }
      }

      if (!objs.empty()) applyDiffMany(objs, "INITIAL scene");

      // ACM: dynamic objects (bottles, crate, ...) + any attach_acm_links are
      // allowed to collide with EVERYTHING (shown but collision-transparent, so the
      // gripper / crate may touch them). Attaching later re-tightens the held ones.
      std::vector<std::string> allow_names = dynamic_ids;
      const auto acm_links = getStringArray("attach_acm_links");
      allow_names.insert(allow_names.end(), acm_links.begin(), acm_links.end());
      if (!allow_names.empty()) allowCollisions(allow_names);

      initial_loaded_ = true;
      RCLCPP_INFO(get_logger(),
                  "scene_manager: ready. attach: %zu object(s) -> '%s' on close of '%s' "
                  "(collision_check=%s vs %zu static mesh(es)).",
                  attach_ids_.size(), attach_link_.c_str(),
                  get_parameter("gripper_cmd_topic").as_string().c_str(),
                  collision_check_ ? "ON" : "OFF", static_ids_.size());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Initial scene load failed: %s", e.what());
    }
  }

  void reassertDynamic()
  {
    std::lock_guard<std::mutex> lock(scene_mutex_);
    if (forced_.empty()) return;
    std::vector<moveit_msgs::msg::CollisionObject> objs;
    objs.reserve(forced_.size());
    for (const auto & kv : forced_) objs.push_back(kv.second);
    publishDiffOnTopic(objs);
  }

  // ---- geometry helpers ----------------------------------------------------
  static geometry_msgs::msg::Pose objectPose(const moveit_msgs::msg::CollisionObject & o)
  {
    if (!o.mesh_poses.empty())      return o.mesh_poses[0];
    if (!o.primitive_poses.empty()) return o.primitive_poses[0];
    return geometry_msgs::msg::Pose();
  }

  static void setObjectPose(moveit_msgs::msg::CollisionObject & o,
                            const geometry_msgs::msg::Pose & p)
  {
    for (auto & mp : o.mesh_poses) mp = p;
    for (auto & pp : o.primitive_poses) pp = p;
  }

  bool tcpInBase(Eigen::Isometry3d & out)
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        frame_id_, attach_link_, tf2::TimePointZero, tf2::durationFromSec(2.0));
      out = tf2::transformToEigen(tf);
      return true;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "[GRASP] TF %s->%s failed: %s",
                   frame_id_.c_str(), attach_link_.c_str(), e.what());
      return false;
    }
  }

  // ---- gripper-triggered grasp / release -----------------------------------
  void onGripperCmd(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool closed = msg->data;
    if (closed == gripper_closed_) return;   // edge-triggered
    gripper_closed_ = closed;
    // diagnostic: if this line never prints on gripper close, the /isaac_gripper_cmd
    // signal isn't reaching the node (check the bridge / QoS), not the grasp itself.
    RCLCPP_INFO(get_logger(), "[GRASP] gripper cmd = %s (mode=%s, scene loaded=%s)",
                closed ? "CLOSE" : "OPEN", grasp_mode_.c_str(),
                initial_loaded_ ? "yes" : "no");
    if (!initial_loaded_) return;            // scene not up yet
    if (closed) graspObjects();
    else        releaseObjects();
  }

  // On CLOSE: record each object's pose relative to the tool, REMOVE its mesh from the
  // world (meshes are never attached — that is what froze RViz), and in attach_box mode
  // also attach a cheap AABB box to the tool so the planner stays payload-aware.
  void graspObjects()
  {
    if (attach_ids_.empty()) return;
    Eigen::Isometry3d T_tcp;
    if (!tcpInBase(T_tcp)) return;
    moveit_msgs::msg::PlanningScene ps;
    // STAGED state: committed only once /apply_planning_scene has accepted the diff.
    // Mutating grasp_offset_/attached_/forced_ up front left the node believing it held
    // an object the scene never received, so the next release "re-placed" a payload that
    // was never attached — a silent desync that outlives the failed grasp.
    std::map<std::string, Eigen::Isometry3d> pending_offset;
    std::vector<std::string> pending_attached;
    size_t n = 0;
    {
      std::lock_guard<std::mutex> lock(scene_mutex_);
      for (const auto & id : attach_ids_) {
        if (dyn_objects_.find(id) == dyn_objects_.end()) continue;
        Eigen::Isometry3d T_obj;
        tf2::fromMsg(world_pose_.count(id) ? world_pose_[id]
                                           : objectPose(dyn_objects_[id]), T_obj);
        pending_offset[id] = T_tcp.inverse() * T_obj;  // object in the tool frame

        moveit_msgs::msg::CollisionObject rm;           // remove the world mesh
        rm.id = id; rm.header.frame_id = frame_id_;
        rm.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        ps.world.collision_objects.push_back(rm);

        if (grasp_mode_ == "attach_box" && grasp_box_[id].size() == 3) {
          moveit_msgs::msg::AttachedCollisionObject aco;
          aco.link_name = attach_link_;
          aco.object.id = id;
          aco.object.header.frame_id = attach_link_;    // box pose given in the tool frame
          aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;
          shape_msgs::msg::SolidPrimitive box;
          box.type = shape_msgs::msg::SolidPrimitive::BOX;
          box.dimensions = {grasp_box_[id][0], grasp_box_[id][1], grasp_box_[id][2]};
          aco.object.primitives.push_back(box);
          Eigen::Isometry3d T_box = pending_offset[id];  // centre the box on the AABB
          const auto & c = grasp_box_center_[id];
          if (c.size() == 3) T_box = T_box * Eigen::Translation3d(c[0], c[1], c[2]);
          aco.object.primitive_poses.push_back(tf2::toMsg(T_box));
          aco.touch_links = touch_links_;
          ps.robot_state.attached_collision_objects.push_back(aco);
          pending_attached.push_back(id);
        }
        ++n;
      }
    }
    if (n == 0) return;
    if (!applyScene(ps, "GRASP")) {
      RCLCPP_ERROR(get_logger(),
                   "[GRASP] scene NOT updated -> nothing attached and the node state is "
                   "left untouched. The planner stays payload-blind for this transfer. "
                   "A 'does not exist in this scene' warning from move_group usually "
                   "means a SECOND scene_manager_node is fighting over the same scene.");
      return;
    }
    {                                     // commit only what the scene actually accepted
      std::lock_guard<std::mutex> lock(scene_mutex_);
      for (const auto & kv : pending_offset) {
        grasp_offset_[kv.first] = kv.second;
        forced_.erase(kv.first);
      }
      for (const auto & id : pending_attached) attached_.insert(id);
    }
    if (grasp_mode_ == "attach_box") applyAttachedAcm();
    RCLCPP_INFO(get_logger(), "[GRASP] CLOSE -> %zu object(s) %s.", n,
                grasp_mode_ == "attach_box" ? "attached as AABB boxes"
                                            : "removed from the scene (held)");
  }

  // On OPEN: detach any box, and re-ADD each held object's MESH at the tool pose x the
  // recorded grasp offset -> it reappears where the EE released it.
  void releaseObjects()
  {
    Eigen::Isometry3d T_tcp;
    const bool have_tcp = tcpInBase(T_tcp);
    moveit_msgs::msg::PlanningScene ps;
    std::vector<std::string> released;
    // STAGED (mirrors graspObjects): build the diff WITHOUT mutating attached_/grasp_offset_/
    // world_pose_/forced_, and commit only once /apply_planning_scene has ACCEPTED the detach.
    // Erasing the state up front meant a single failed apply (e.g. move_group momentarily busy)
    // left the node believing it had released while RViz kept the object ATTACHED (purple)
    // forever, and no later OPEN could retry — a silent, permanent desync. The DETACHLINK weld
    // in Gazebo (a separate Bool subscriber) succeeds independently, so the symptom was
    // "detached in Gazebo, stuck purple in RViz".
    std::vector<std::string> pending_detach;
    std::map<std::string, geometry_msgs::msg::Pose> pending_world_pose;
    std::map<std::string, moveit_msgs::msg::CollisionObject> pending_forced;
    {
      std::lock_guard<std::mutex> lock(scene_mutex_);
      for (const auto & id : attach_ids_) {
        if (grasp_offset_.find(id) == grasp_offset_.end()) continue;  // wasn't grasped
        if (attached_.count(id)) {
          moveit_msgs::msg::AttachedCollisionObject aco;   // detach the box
          aco.link_name = attach_link_;
          aco.object.id = id;
          aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
          ps.robot_state.attached_collision_objects.push_back(aco);
          pending_detach.push_back(id);
        }
        geometry_msgs::msg::Pose newP =
          have_tcp ? tf2::toMsg(Eigen::Isometry3d(T_tcp * grasp_offset_[id]))
                   : objectPose(dyn_objects_[id]);
        auto obj = dyn_objects_[id];                       // cached collision object (mesh/box)
        obj.operation = moveit_msgs::msg::CollisionObject::ADD;
        obj.header.frame_id = frame_id_;
        setObjectPose(obj, newP);
        ps.world.collision_objects.push_back(obj);
        pending_world_pose[id] = newP;                     // track for re-pick (commit on success)
        pending_forced[id] = obj;                          // dynamic world object again
        released.push_back(id);
      }
    }
    if (released.empty()) return;
    if (!applyScene(ps, "RELEASE")) {
      RCLCPP_ERROR(get_logger(),
                   "[RELEASE] scene NOT updated -> object stays ATTACHED (purple); node state "
                   "left intact so the NEXT open retries (no silent desync).");
      return;
    }
    {                                     // commit only what the scene actually accepted
      std::lock_guard<std::mutex> lock(scene_mutex_);
      for (const auto & id : pending_detach) attached_.erase(id);
      for (const auto & kv : pending_world_pose) world_pose_[kv.first] = kv.second;
      for (const auto & kv : pending_forced) forced_[kv.first] = kv.second;
      for (const auto & id : released) grasp_offset_.erase(id);
    }
    Acm acm;                                               // released -> transparent again
    if (fetchAcm(acm)) {
      for (const auto & id : released) {
        setAcmDefault(acm, id, true);
        for (const auto & s : static_ids_) setAcmEntry(acm, id, s, true);
      }
      applyAcm(acm);
    }
    RCLCPP_INFO(get_logger(), "[GRASP] OPEN -> %zu object(s) reappear at the tool pose.",
                released.size());
  }

  // Attached objects: always allowed vs everything by DEFAULT (so vs the gripper —
  // touch_links — and vs the other dynamic objects, crate/bottles), then EXPLICITLY
  // checked vs each static/actuated mesh when the flag is ON. Explicit entries win
  // over the default, and using per-static explicit entries (few) sidesteps the
  // "NEVER wins" default-combine rule that would otherwise also block the crate.
  void applyAttachedAcm()
  {
    std::set<std::string> attached_copy;
    { std::lock_guard<std::mutex> lock(scene_mutex_); attached_copy = attached_; }
    if (attached_copy.empty()) return;
    Acm acm;
    if (!fetchAcm(acm)) return;
    for (const auto & a : attached_copy) {
      setAcmDefault(acm, a, true);
      for (const auto & s : static_ids_)
        setAcmEntry(acm, a, s, !collision_check_);   // ON -> checked(false); OFF -> allowed(true)
    }
    if (!applyAcm(acm)) return;
    RCLCPP_INFO(get_logger(),
                "[ATTACH] ACM: %zu attached object(s), collision_check=%s vs %zu static mesh(es).",
                attached_copy.size(), collision_check_ ? "ON" : "OFF", static_ids_.size());
  }

  // Scene reset (sim): detach anything held and re-ADD every dynamic object at its
  // INITIAL pose. Called by the ResetScene BT node at the loop's cycle boundary.
  // Isaac-side seam. Publishing a permanent `true` on a LATCHED topic is unusable by a
  // subscriber: dedupe on value and it fires once per session; act on every compute and
  // the adapter pins its prims home every frame. So emit a RISING EDGE — true, then false
  // shortly after — and let the adapter trigger on the transition. A late joiner reads the
  // trailing `false`, i.e. "no reset pending", which is the correct resting state.
  void publishResetPulse()
  {
    std_msgs::msg::Bool up; up.data = true;
    reset_pub_->publish(up);
    reset_pulse_timer_ = create_wall_timer(
      std::chrono::milliseconds(reset_pulse_ms_),
      [this]() {
        std_msgs::msg::Bool down; down.data = false;
        reset_pub_->publish(down);
        reset_pulse_timer_->cancel();       // one-shot
      });
  }

  void onResetScene(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    moveit_msgs::msg::PlanningScene ps;
    std::vector<std::string> dyn_ids;
    {
      std::lock_guard<std::mutex> lock(scene_mutex_);
      for (auto & kv : dyn_objects_) {
        const std::string & id = kv.first;
        if (attached_.count(id)) {                    // still held as a box: detach it
          moveit_msgs::msg::AttachedCollisionObject aco;
          aco.link_name = attach_link_;
          aco.object.id = id;
          aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
          ps.robot_state.attached_collision_objects.push_back(aco);
        }
        auto obj = kv.second;                         // cached geometry
        obj.operation = moveit_msgs::msg::CollisionObject::ADD;
        obj.header.frame_id = frame_id_;
        setObjectPose(obj, initial_pose_[id]);
        ps.world.collision_objects.push_back(obj);
        world_pose_[id] = initial_pose_[id];
        forced_[id] = obj;
        grasp_offset_.erase(id);
        dyn_ids.push_back(id);
      }
      attached_.clear();
    }
    if (dyn_ids.empty()) {
      res->success = false;
      res->message = "no dynamic objects to reset";
      return;
    }
    if (!applyScene(ps, "RESET")) {
      res->success = false;
      res->message = "reset diff failed";
      return;
    }
    allowCollisions(dyn_ids);                          // transparent again, like startup
    publishResetPulse();
    res->success = true;
    res->message = "scene reset: " + std::to_string(dyn_ids.size()) +
                   " dynamic object(s) back to initial poses";
    RCLCPP_INFO(get_logger(), "[RESET] %s", res->message.c_str());
  }

  void onSetCheck(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                  std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    collision_check_ = req->data;
    applyAttachedAcm();   // re-apply for currently attached objects
    res->success = true;
    res->message = std::string("attached_collision_check=") +
      (collision_check_ ? "ON (attached objects CHECKED vs static meshes)"
                        : "OFF (attached objects transparent to everything)");
    RCLCPP_INFO(get_logger(), "[ATTACH] %s", res->message.c_str());
  }

  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr force_timer_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  ApplySceneClient::SharedPtr apply_client_;
  GetSceneClient::SharedPtr get_scene_client_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr check_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_pub_;
  rclcpp::TimerBase::SharedPtr reset_pulse_timer_;
  int reset_pulse_ms_ = 300;      // width of the reset rising edge

  std::mutex scene_mutex_;
  bool initial_loaded_ = false;
  std::string frame_id_ = "base_link";
  int scene_update_wait_ms_ = 800;
  std::unordered_map<std::string, moveit_msgs::msg::CollisionObject> forced_;
  std::unordered_map<std::string, moveit_msgs::msg::CollisionObject> dyn_objects_;  // geometry cache

  // attach state
  std::string attach_link_ = "tcp";
  std::vector<std::string> touch_links_;
  std::vector<std::string> attach_ids_;    // objects that attach on close
  std::vector<std::string> static_ids_;    // checked meshes attached objs route around
  std::set<std::string> attached_;         // currently attached (attach_box mode)
  bool collision_check_ = true;
  bool gripper_closed_ = false;

  // grasp handling
  std::string grasp_mode_ = "remove";
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unordered_map<std::string, geometry_msgs::msg::Pose> world_pose_;      // current world pose
  std::unordered_map<std::string, geometry_msgs::msg::Pose> initial_pose_;    // scene.yaml pose (reset)
  std::unordered_map<std::string, Eigen::Isometry3d> grasp_offset_;          // object in tool frame
  std::unordered_map<std::string, std::vector<double>> grasp_box_;           // AABB extents
  std::unordered_map<std::string, std::vector<double>> grasp_box_center_;    // local AABB centre
};

}  // namespace trainit_scene

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<trainit_scene::SceneManagerNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
