// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §10, ICD-MC-001 §9).

#include "health_monitor/health_monitor_node.hpp"

#include <chrono>
#include <cmath>
#include <iterator>
#include <utility>

#include "health_monitor/systemd_restart_client.hpp"

namespace health_monitor
{

namespace
{
// MAV_SEVERITY (MAVLink common.xml): 0=EMERGENCY..2=CRITICAL..7=DEBUG.
constexpr uint8_t kSeverityCritical = 2;
constexpr uint8_t kSeverityWarning = 4;
constexpr const char * kMissionExecService = "mission-exec.service";
constexpr int kManagerMaxRestarts = 3;

// MAV_CMD IDs (ICD-MC-001 §10.5). No shared header defines these across
// packages yet -- mission_manager_node.cpp keeps its own local copies of
// the ones it needs too.
constexpr uint32_t kMavCmdNavLoiterUnlim = 17;      // "HOLD"
constexpr uint32_t kMavCmdNavReturnToLaunch = 20;   // "RTL"

// MC_CMD_GCS_OVERRIDE (ICD §4.5/§10.5, custom command 50004): param1 =
// 1=enable/0=disable, param2 = override type (ICD §10.7).
constexpr uint32_t kMcCmdGcsOverride = 50004;
constexpr uint8_t kOverrideNone = 0;
constexpr uint8_t kOverrideLowBatteryRtl = 1;
constexpr uint8_t kOverrideGcsLinkHold = 2;

constexpr float kDefaultLowBatteryThreshold = 0.20F;
constexpr double kThrottledInferenceFps = 5.0;

// SDD-MC-001 §10.3's mission-phase set that gates GCS_LINK_LOST -> HOLD.
bool is_gcs_supervised_phase(uint8_t phase)
{
  switch (phase) {
    case mc_msgs::msg::MissionPhase::INGRESS:
    case mc_msgs::msg::MissionPhase::MUSTER:
    case mc_msgs::msg::MissionPhase::ISR_SELECTION:
    case mc_msgs::msg::MissionPhase::VORONOI_PARTITION:
    case mc_msgs::msg::MissionPhase::SEARCH:
      return true;
    default:
      return false;
  }
}
}  // namespace

HealthMonitorNode::HealthMonitorNode(const rclcpp::NodeOptions & options)
: Node("health_monitor_node", options)
{
  ResourceSamplerConfig config;
  config.proc_stat_path = declare_parameter("proc_stat_path", config.proc_stat_path);
  config.meminfo_path = declare_parameter("meminfo_path", config.meminfo_path);
  config.disk_path = declare_parameter("disk_path", config.disk_path);
  config.cpu_thermal_zone_paths =
    declare_parameter("cpu_thermal_zone_paths", config.cpu_thermal_zone_paths);
  config.gpu_temp_command = declare_parameter("gpu_temp_command", config.gpu_temp_command);
  config.vision_cgroup_memory_current_path = declare_parameter(
    "vision_cgroup_memory_current_path", config.vision_cgroup_memory_current_path);
  config.vision_cgroup_memory_max_path = declare_parameter(
    "vision_cgroup_memory_max_path", config.vision_cgroup_memory_max_path);
  sampler_ = ResourceSampler(config);

  const double sample_period_s = declare_parameter("sample_period_s", 1.0);

  start_time_ = now();
  last_manager_action_time_ = start_time_;

  pub_health_ = create_publisher<mc_msgs::msg::SystemHealth>("/mc/health", rclcpp::QoS(10));
  pub_fault_ = create_publisher<mc_msgs::msg::FaultEvent>("/mc/fault", rclcpp::QoS(10));
  pub_send_command_ = create_publisher<mc_msgs::msg::FccCommand>(
    "/mc/fcc/send_command", rclcpp::QoS(10));
  pub_status_text_ = create_publisher<mc_msgs::msg::StatusText>(
    "/mc/status_text", rclcpp::QoS(10));

  manager_heartbeat_timeout_s_ =
    declare_parameter("manager_heartbeat_timeout_s", manager_heartbeat_timeout_s_);
  manager_restart_retry_s_ =
    declare_parameter("manager_restart_retry_s", manager_restart_retry_s_);
  gpu_overtemp_threshold_c_ = static_cast<uint8_t>(
    declare_parameter(
      "gpu_overtemp_threshold_c", static_cast<int64_t>(gpu_overtemp_threshold_c_)));
  vision_mem_threshold_pct_ = static_cast<uint8_t>(
    declare_parameter(
      "vision_mem_threshold_pct", static_cast<int64_t>(vision_mem_threshold_pct_)));
  vision_mem_sustained_s_ =
    declare_parameter("vision_mem_sustained_s", vision_mem_sustained_s_);
  watchdog_socket_path_ = declare_parameter(
    "watchdog_socket_path", std::string("/var/run/drone-mc/watchdog.sock"));
  watchdog_timeout_ms_ = static_cast<int>(
    declare_parameter("watchdog_timeout_ms", static_cast<int64_t>(watchdog_timeout_ms_)));

  restart_trigger_ = [this](const std::string & service) {
      const auto result = request_systemd_restart(
        watchdog_socket_path_, service, watchdog_timeout_ms_);
      if (result.ok) {
        RCLCPP_INFO(get_logger(), "systemd restart requested for '%s'", service.c_str());
      } else {
        RCLCPP_ERROR(
          get_logger(), "systemd restart request for '%s' failed: %s",
          service.c_str(), result.reason.c_str());
      }
    };

  sub_mission_phase_ = create_subscription<mc_msgs::msg::MissionPhase>(
    "/mc/mission_phase", rclcpp::QoS(10),
    [this](mc_msgs::msg::MissionPhase::SharedPtr msg) {latest_mission_phase_ = msg;});
  sub_mission_upload_ = create_subscription<mc_msgs::msg::MissionUpload>(
    "/mc/gcs/mission_upload", rclcpp::QoS(10),
    [this](mc_msgs::msg::MissionUpload::SharedPtr msg) {latest_mission_upload_ = msg;});
  sub_gcs_command_ = create_subscription<mc_msgs::msg::GcsCommand>(
    "/mc/gcs/command", rclcpp::QoS(10),
    [this](mc_msgs::msg::GcsCommand::SharedPtr msg) {on_gcs_command(msg);});

  wire_node_watches();

  sample_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(sample_period_s)),
    [this] {sample_and_publish();});

  RCLCPP_INFO(get_logger(), "health_monitor_node started");
}

void HealthMonitorNode::set_restart_trigger(std::function<void(const std::string &)> trigger)
{
  restart_trigger_ = std::move(trigger);
}

void HealthMonitorNode::wire_node_watches()
{
  // Critical nodes and heartbeat topics/timeouts, SDD-MC-001 §10.2. Each
  // per-node timeout is also exposed as its own ROS parameter
  // ("<node_name>_timeout_s") so a test can shrink it instead of waiting
  // out the real SDD-specified window. mission_manager_node's timeout is
  // `manager_heartbeat_timeout_s` instead (already declared above) since
  // that same value doubles as the FR-074 watchdog threshold.
  struct Spec
  {
    const char * node_name;
    const char * topic;
    const char * type;
    uint16_t mask_bit;
    double default_timeout_s;
  };

  const Spec specs[] = {
    {
      "mission_manager_node", "/mc/manager/heartbeat", "std_msgs/msg/Header",
      mc_msgs::msg::SystemHealth::MISSION_MANAGER_ALIVE, manager_heartbeat_timeout_s_
    },
    {
      "fcc_bridge_node", "/mc/fcc/vehicle_state", "mc_msgs/msg/VehicleState",
      mc_msgs::msg::SystemHealth::FCC_BRIDGE_ALIVE, 2.0
    },
    {
      "gcs_mavlink_node", "/mc/gcs/link_status", "mc_msgs/msg/LinkStatus",
      mc_msgs::msg::SystemHealth::GCS_MAVLINK_ALIVE, 2.0
    },
    {
      "swarm_coordinator_node", "/mc/swarm/peer_states", "mc_msgs/msg/SwarmPeerArray",
      mc_msgs::msg::SystemHealth::SWARM_COORDINATOR_ALIVE, 3.0
    },
    {
      "camera_node", "/camera/image_raw", "sensor_msgs/msg/Image",
      mc_msgs::msg::SystemHealth::CAMERA_ALIVE, 3.0
    },
    {
      "perception_down_node", "/mc/detections", "mc_msgs/msg/DetectionArray",
      mc_msgs::msg::SystemHealth::PERCEPTION_DOWN_ALIVE, 10.0
    },
    {
      "visual_localisation_node", "/mc/local_pose",
      "geometry_msgs/msg/PoseWithCovarianceStamped",
      mc_msgs::msg::SystemHealth::VISUAL_LOCALISATION_ALIVE, 5.0
    },
    {
      "perception_front_node", "/mc/perception/front/tracks", "mc_msgs/msg/TrackArray",
      mc_msgs::msg::SystemHealth::PERCEPTION_FRONT_ALIVE, 3.0
    },
    {
      "video_mux_node", "/mc/video/downlink_status", "mc_msgs/msg/VideoDownlinkStatus",
      mc_msgs::msg::SystemHealth::VIDEO_MUX_ALIVE, 2.0
    },
  };

  node_watches_.reserve(std::size(specs));
  for (const auto & spec : specs) {
    const bool is_manager = std::string(spec.node_name) == "mission_manager_node";

    NodeWatch watch;
    watch.node_name = spec.node_name;
    watch.mask_bit = spec.mask_bit;
    watch.timeout_s = is_manager ?
      manager_heartbeat_timeout_s_ :
      declare_parameter(
      std::string(spec.node_name) + "_timeout_s", spec.default_timeout_s);
    node_watches_.push_back(watch);

    const std::size_t idx = node_watches_.size() - 1;
    if (is_manager) {
      manager_watch_index_ = idx;
    }

    const std::string node_name = spec.node_name;
    if (node_name == "fcc_bridge_node") {
      // HEALTH-003 needs vehicle_state content (fcc_link_ok, battery_pct),
      // not just arrival timing -- typed subscription doubles as this
      // watch's liveness bookkeeping.
      node_watches_[idx].sub = create_subscription<mc_msgs::msg::VehicleState>(
        spec.topic, rclcpp::QoS(10),
        [this, idx](mc_msgs::msg::VehicleState::SharedPtr msg) {
          node_watches_[idx].last_received = now();
          node_watches_[idx].ever_received = true;
          latest_vehicle_state_ = msg;
        });
    } else if (node_name == "gcs_mavlink_node") {
      // HEALTH-003 needs gcs_link_ok content, same reasoning as above.
      node_watches_[idx].sub = create_subscription<mc_msgs::msg::LinkStatus>(
        spec.topic, rclcpp::QoS(10),
        [this, idx](mc_msgs::msg::LinkStatus::SharedPtr msg) {
          node_watches_[idx].last_received = now();
          node_watches_[idx].ever_received = true;
          latest_link_status_ = msg;
        });
    } else {
      node_watches_[idx].sub = create_generic_subscription(
        spec.topic, spec.type, rclcpp::QoS(10),
        [this, idx](std::shared_ptr<rclcpp::SerializedMessage>) {
          node_watches_[idx].last_received = now();
          node_watches_[idx].ever_received = true;
        });
    }
  }
}

bool HealthMonitorNode::is_alive(const NodeWatch & watch, const rclcpp::Time & current) const
{
  if (!watch.ever_received) {
    return false;
  }
  return (current - watch.last_received).seconds() < watch.timeout_s;
}

void HealthMonitorNode::publish_fault(
  uint16_t fault_id, uint8_t severity, const std::string & description, uint8_t action)
{
  mc_msgs::msg::FaultEvent fault;
  fault.header.stamp = now();
  fault.fault_id = fault_id;
  fault.severity = severity;
  fault.description = description;
  fault.failsafe_action = action;
  pub_fault_->publish(fault);
  RCLCPP_ERROR(get_logger(), "%s", description.c_str());
}

void HealthMonitorNode::publish_status_text(uint8_t severity, const std::string & text)
{
  mc_msgs::msg::StatusText msg;
  msg.header.stamp = now();
  msg.severity = severity;
  msg.text = text;
  pub_status_text_->publish(msg);
}

void HealthMonitorNode::send_fcc_command(uint32_t cmd)
{
  mc_msgs::msg::FccCommand fwd;
  fwd.header.stamp = now();
  fwd.cmd = cmd;
  pub_send_command_->publish(fwd);
}

void HealthMonitorNode::on_gcs_command(mc_msgs::msg::GcsCommand::SharedPtr msg)
{
  if (msg->command != kMcCmdGcsOverride) {
    return;
  }
  const bool enable = msg->params[0] != 0.0F;
  const auto override_type = static_cast<uint8_t>(std::lround(msg->params[1]));
  switch (override_type) {
    case kOverrideNone:
      override_low_battery_rtl_ = false;
      override_gcs_link_hold_ = false;
      break;
    case kOverrideLowBatteryRtl:
      override_low_battery_rtl_ = enable;
      break;
    case kOverrideGcsLinkHold:
      override_gcs_link_hold_ = enable;
      break;
    default:
      RCLCPP_WARN(
        get_logger(), "MC_CMD_GCS_OVERRIDE: unrecognised override type=%u", override_type);
      break;
  }
}

bool HealthMonitorNode::mission_phase_requires_gcs_link_hold() const
{
  return latest_mission_phase_ && is_gcs_supervised_phase(latest_mission_phase_->phase);
}

float HealthMonitorNode::effective_low_battery_threshold() const
{
  if (latest_mission_upload_ && latest_mission_upload_->low_battery_threshold > 0.0F) {
    return latest_mission_upload_->low_battery_threshold;
  }
  return kDefaultLowBatteryThreshold;
}

void HealthMonitorNode::throttle_inference_rate()
{
  if (!perception_down_param_client_) {
    perception_down_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      get_node_base_interface(), get_node_topics_interface(), get_node_graph_interface(),
      get_node_services_interface(), "perception_down_node");
  }
  // Best-effort: perception_down_node may not exist yet (HEALTH-003 lands
  // ahead of VIS-004) -- an absent target just means the throttle never
  // takes effect, which is logged here rather than escalated further, and
  // starts working automatically once that node declares this parameter.
  perception_down_param_client_->set_parameters(
    {rclcpp::Parameter("inference_rate_limit_fps", kThrottledInferenceFps)},
    [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
      bool all_ok = true;
      for (const auto & result : future.get()) {
        all_ok = all_ok && result.successful;
      }
      if (all_ok) {
        RCLCPP_INFO(get_logger(), "perception_down_node inference_rate_limit_fps throttled");
      } else {
        RCLCPP_WARN(get_logger(), "perception_down_node inference_rate_limit_fps throttle failed");
      }
    });
}

void HealthMonitorNode::check_manager_heartbeat(bool alive_now, const rclcpp::Time & current)
{
  if (alive_now) {
    manager_restart_count_ = 0;
    manager_escalated_ = false;
    manager_was_alive_ = true;
    fault_mask_ &= static_cast<uint16_t>(~mc_msgs::msg::FaultEvent::MANAGER_DEAD);
    return;
  }

  fault_mask_ |= mc_msgs::msg::FaultEvent::MANAGER_DEAD;
  const bool just_died = manager_was_alive_;
  manager_was_alive_ = false;

  // Once escalated to a direct RTL, stop acting until the manager recovers
  // -- this is what keeps the branch edge-triggered rather than repeatedly
  // re-firing every tick while the outage persists.
  if (manager_escalated_) {
    return;
  }

  // Retry on a cooldown, not a single edge: a node that never comes back
  // on its own only ever produces ONE alive->dead edge for the whole
  // outage, so edge-only logic could never reach "3 failed restart
  // attempts" (FR-075) against the exact case that matters most. Instead,
  // re-evaluate once per manager_restart_retry_s_ -- long enough to give
  // systemd's restart and the process's own re-init a real chance to land
  // a fresh heartbeat before the next attempt is judged.
  const bool retry_due = just_died ||
    (current - last_manager_action_time_).seconds() >= manager_restart_retry_s_;
  if (!retry_due) {
    return;
  }
  last_manager_action_time_ = current;
  ++manager_restart_count_;

  if (manager_restart_count_ <= kManagerMaxRestarts) {
    restart_trigger_(kMissionExecService);
    publish_fault(
      mc_msgs::msg::FaultEvent::MANAGER_DEAD, kSeverityCritical,
      "mission_manager_node heartbeat silent > " +
      std::to_string(manager_heartbeat_timeout_s_) + " s -- requested restart attempt " +
      std::to_string(manager_restart_count_) + "/" + std::to_string(kManagerMaxRestarts),
      mc_msgs::msg::FaultEvent::ACTION_NONE);
  } else {
    manager_escalated_ = true;
    // FR-075's explicit, narrow exception to FR-026: mission_manager_node
    // is presumed unreachable to react to its own death, so this is the
    // one branch where health_monitor_node commands the FCC directly.
    send_fcc_command(kMavCmdNavReturnToLaunch);
    publish_fault(
      mc_msgs::msg::FaultEvent::MANAGER_DEAD, kSeverityCritical,
      "mission_manager_node heartbeat silent, restart budget (" +
      std::to_string(kManagerMaxRestarts) + ") exhausted -- commanded RTL directly (FR-075)",
      mc_msgs::msg::FaultEvent::ACTION_RTL);
  }
}

void HealthMonitorNode::check_fcc_link()
{
  const bool link_ok = latest_vehicle_state_ && latest_vehicle_state_->fcc_link_ok;
  if (fcc_link_was_ok_ && !link_ok) {
    // FR-026: this node never calls send_command() for FCC_LINK_LOST.
    // fcc_bridge_node already publishes its own FCC_LINK_LOST fault, and
    // mission_manager_node's on_fault()/is_critical_fault() already treats
    // that fault_id as unconditionally critical and auto-RTLs -- this is a
    // second, independent observation of the same condition (derived from
    // vehicle_state.fcc_link_ok rather than fcc_bridge_node's own internal
    // link timer) for /mc/health visibility and defense-in-depth, not a
    // new command path. ACTION_RTL below documents what will actually
    // happen via that pre-existing mechanism, not an action taken here --
    // SDD §10.3's literal HOLD-then-RTL text is moot regardless: neither
    // is reachable over the very link that just went down.
    publish_fault(
      mc_msgs::msg::FaultEvent::FCC_LINK_LOST, kSeverityCritical,
      "uXRCE-DDS FCC link lost -- mission_manager_node's existing fault-handling will RTL",
      mc_msgs::msg::FaultEvent::ACTION_RTL);
  }
  fcc_link_was_ok_ = link_ok;
  if (link_ok) {
    fault_mask_ &= static_cast<uint16_t>(~mc_msgs::msg::FaultEvent::FCC_LINK_LOST);
  } else {
    fault_mask_ |= mc_msgs::msg::FaultEvent::FCC_LINK_LOST;
  }
}

void HealthMonitorNode::check_low_battery()
{
  const bool low = latest_vehicle_state_ &&
    latest_vehicle_state_->battery_pct < effective_low_battery_threshold();
  if (!low_battery_was_active_ && low) {
    if (override_low_battery_rtl_) {
      publish_fault(
        mc_msgs::msg::FaultEvent::LOW_BATTERY, kSeverityWarning,
        "battery_pct below low_battery_threshold -- RTL suppressed by GCS "
        "OVERRIDE_LOW_BATTERY_RTL",
        mc_msgs::msg::FaultEvent::ACTION_NONE);
    } else {
      // FR-026: no send_command() here either -- severity CRITICAL is
      // enough for mission_manager_node's existing is_critical_fault()
      // (severity <= CRITICAL) to auto-RTL on its own, same as
      // FCC_LINK_LOST above. ACTION_RTL documents that outcome.
      publish_fault(
        mc_msgs::msg::FaultEvent::LOW_BATTERY, kSeverityCritical,
        "battery_pct below low_battery_threshold -- mission_manager_node's existing "
        "fault-handling will RTL",
        mc_msgs::msg::FaultEvent::ACTION_RTL);
    }
  }
  low_battery_was_active_ = low;
  if (low) {
    fault_mask_ |= mc_msgs::msg::FaultEvent::LOW_BATTERY;
  } else {
    fault_mask_ &= static_cast<uint16_t>(~mc_msgs::msg::FaultEvent::LOW_BATTERY);
  }
}

void HealthMonitorNode::check_gcs_link()
{
  const bool link_ok = latest_link_status_ && latest_link_status_->gcs_link_ok;
  if (gcs_link_was_ok_ && !link_ok) {
    const bool phase_gates_hold = mission_phase_requires_gcs_link_hold();
    if (phase_gates_hold && !override_gcs_link_hold_) {
      // FR-026: no send_command() -- HOLD has no existing fault-consumption
      // path (mission_manager_node's on_fault() only ever RTLs) and adding
      // one is a mission_exec change outside this ticket's file scope.
      // Severity is deliberately WARNING (not CRITICAL) so this fault does
      // NOT trip mission_manager_node's blanket critical-fault->RTL
      // reaction -- an unintended RTL would be worse than the documented
      // gap of HOLD not actually being enacted (known gap, same class as
      // OFFBOARD_STREAM_STALE/GIMBAL_LINK_LOST).
      publish_fault(
        mc_msgs::msg::FaultEvent::GCS_LINK_LOST, kSeverityWarning,
        "GCS MAVLink link lost during a GCS-supervised phase -- HOLD is the intended "
        "action (SDD §10.3) but not yet enacted by any existing fault-consumption path",
        mc_msgs::msg::FaultEvent::ACTION_HOLD);
    } else {
      publish_fault(
        mc_msgs::msg::FaultEvent::GCS_LINK_LOST, kSeverityWarning,
        phase_gates_hold ?
        "GCS MAVLink link lost -- HOLD suppressed by GCS OVERRIDE_GCS_LINK_HOLD" :
        "GCS MAVLink link lost outside a GCS-supervised phase -- no action",
        mc_msgs::msg::FaultEvent::ACTION_NONE);
    }
  }
  gcs_link_was_ok_ = link_ok;
  if (link_ok) {
    fault_mask_ &= static_cast<uint16_t>(~mc_msgs::msg::FaultEvent::GCS_LINK_LOST);
  } else {
    fault_mask_ |= mc_msgs::msg::FaultEvent::GCS_LINK_LOST;
  }
}

void HealthMonitorNode::check_gpu_overtemp(uint8_t gpu_temp_c)
{
  const bool overtemp = gpu_temp_c > gpu_overtemp_threshold_c_;
  if (!gpu_overtemp_was_active_ && overtemp) {
    // No flight command involved at all -- FR-026 doesn't apply here.
    throttle_inference_rate();
    publish_status_text(
      mc_msgs::msg::StatusText::SEVERITY_WARNING,
      "GPU temperature " + std::to_string(gpu_temp_c) + "C exceeds threshold -- "
      "inference_rate_limit_fps reduced to " + std::to_string(static_cast<int>(
        kThrottledInferenceFps)) + " FPS");
    publish_fault(
      mc_msgs::msg::FaultEvent::GPU_OVERTEMP, kSeverityWarning,
      "GPU temperature exceeds threshold -- inference rate throttled",
      mc_msgs::msg::FaultEvent::ACTION_NONE);
  }
  gpu_overtemp_was_active_ = overtemp;
  if (overtemp) {
    fault_mask_ |= mc_msgs::msg::FaultEvent::GPU_OVERTEMP;
  } else {
    fault_mask_ &= static_cast<uint16_t>(~mc_msgs::msg::FaultEvent::GPU_OVERTEMP);
  }
}

void HealthMonitorNode::check_vision_memory(const rclcpp::Time & current)
{
  const std::optional<uint8_t> pct = sampler_.sample_vision_mem_pct();
  bool sustained_high = false;
  if (pct.has_value() && *pct >= vision_mem_threshold_pct_) {
    if (!vision_mem_over_since_.has_value()) {
      vision_mem_over_since_ = current;
    }
    sustained_high = (current - *vision_mem_over_since_).seconds() >= vision_mem_sustained_s_;
  } else {
    vision_mem_over_since_.reset();
  }

  if (!vision_mem_high_active_ && sustained_high) {
    // No flight command involved -- FR-026 doesn't apply here, same as
    // GPU_OVERTEMP.
    publish_fault(
      mc_msgs::msg::FaultEvent::VISION_MEM_HIGH, kSeverityWarning,
      "vision.service cgroup memory usage sustained at or above threshold -- "
      "MemoryMax=6G budget at risk",
      mc_msgs::msg::FaultEvent::ACTION_NONE);
  }
  vision_mem_high_active_ = sustained_high;
  if (sustained_high) {
    fault_mask_ |= mc_msgs::msg::FaultEvent::VISION_MEM_HIGH;
  } else {
    fault_mask_ &= static_cast<uint16_t>(~mc_msgs::msg::FaultEvent::VISION_MEM_HIGH);
  }
}

void HealthMonitorNode::sample_and_publish()
{
  mc_msgs::msg::SystemHealth msg;
  msg.header.stamp = now();
  sampler_.sample(msg);
  msg.uptime_s = static_cast<uint32_t>((now() - start_time_).seconds());

  const rclcpp::Time current = now();
  msg.node_alive_mask = mc_msgs::msg::SystemHealth::HEALTH_MONITOR_ALIVE;
  bool manager_alive_now = false;
  for (std::size_t i = 0; i < node_watches_.size(); ++i) {
    const bool alive = is_alive(node_watches_[i], current);
    if (alive) {
      msg.node_alive_mask |= node_watches_[i].mask_bit;
    }
    if (i == manager_watch_index_) {
      manager_alive_now = alive;
    }
  }

  check_manager_heartbeat(manager_alive_now, current);
  check_fcc_link();
  check_low_battery();
  check_gcs_link();
  check_gpu_overtemp(msg.gpu_temp_c);
  check_vision_memory(current);

  msg.fault_mask = fault_mask_;
  pub_health_->publish(msg);
}

}  // namespace health_monitor
