// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §10, ICD-MC-001 §9).

#include "health_monitor/health_monitor_node.hpp"

#include <chrono>
#include <iterator>
#include <utility>

namespace health_monitor
{

namespace
{
constexpr uint8_t kSeverityCritical = 2;
constexpr const char * kMissionExecService = "mission-exec.service";
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
  sampler_ = ResourceSampler(config);

  const double sample_period_s = declare_parameter("sample_period_s", 1.0);

  start_time_ = now();

  pub_health_ = create_publisher<mc_msgs::msg::SystemHealth>("/mc/health", rclcpp::QoS(10));
  pub_fault_ = create_publisher<mc_msgs::msg::FaultEvent>("/mc/fault", rclcpp::QoS(10));

  manager_heartbeat_timeout_s_ =
    declare_parameter("manager_heartbeat_timeout_s", manager_heartbeat_timeout_s_);

  restart_trigger_ = [this](const std::string & service) {
      RCLCPP_WARN(
        get_logger(),
        "restart trigger for '%s' invoked but no systemd-restart transport is wired up yet "
        "(HEALTH-003, SDD-MC-001 §10.4)",
        service.c_str());
    };

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

    node_watches_[idx].sub = create_generic_subscription(
      spec.topic, spec.type, rclcpp::QoS(10),
      [this, idx](std::shared_ptr<rclcpp::SerializedMessage>) {
        node_watches_[idx].last_received = now();
        node_watches_[idx].ever_received = true;
      });
  }
}

bool HealthMonitorNode::is_alive(const NodeWatch & watch, const rclcpp::Time & current) const
{
  if (!watch.ever_received) {
    return false;
  }
  return (current - watch.last_received).seconds() < watch.timeout_s;
}

void HealthMonitorNode::on_manager_heartbeat_lost()
{
  mc_msgs::msg::FaultEvent fault;
  fault.header.stamp = now();
  fault.fault_id = mc_msgs::msg::FaultEvent::MANAGER_DEAD;
  fault.severity = kSeverityCritical;
  fault.description =
    "mission_manager_node heartbeat (/mc/manager/heartbeat) silent > " +
    std::to_string(manager_heartbeat_timeout_s_) + " s";
  fault.failsafe_action = mc_msgs::msg::FaultEvent::ACTION_NONE;
  pub_fault_->publish(fault);
  RCLCPP_ERROR(get_logger(), "%s", fault.description.c_str());

  restart_trigger_(kMissionExecService);
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

  if (manager_was_alive_ && !manager_alive_now) {
    on_manager_heartbeat_lost();
  }
  manager_was_alive_ = manager_alive_now;

  pub_health_->publish(msg);
}

}  // namespace health_monitor
