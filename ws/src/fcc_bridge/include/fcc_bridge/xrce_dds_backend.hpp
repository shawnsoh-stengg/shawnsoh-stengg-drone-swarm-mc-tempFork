// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

#pragma once

#include <netinet/in.h>

#include <mavlink/v2.0/common/mavlink.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fcc_bridge/fcc_backend.hpp"
#include "fcc_bridge/mission_protocol_client.hpp"
#include "fcc_bridge/optical_flow_relay.hpp"

// FCC-002: production FccBackend over uXRCE-DDS (SDD §3.2, ICD §3).
namespace fcc_bridge
{

class XrceDdsBackend : public FccBackend
{
public:
  // Reads FCC_TRANSPORT (serial|ethernet), FCC_SERIAL_DEVICE, FCC_SERIAL_BAUD,
  // FCC_ETH_IP, FCC_ETH_PORT, FCC_MISSION_PORT, FCC_FLOW_PORT from the
  // environment (ICD §9.4). Throws std::runtime_error if the mission-protocol
  // or flow-relay socket cannot be created/bound.
  explicit XrceDdsBackend(rclcpp::Node * node);
  ~XrceDdsBackend() override;

  bool upload_mission(const std::vector<MissionItem> & items) override;
  bool update_mission_item(uint16_t seq, const MissionItem & item) override;
  void stream_offboard_setpoint(const OffboardSetpoint & sp) override;
  void send_command(uint32_t cmd, const std::array<float, 7> & params) override;
  void publish_visual_odometry(const VisualOdometry & vo) override;
  VehicleState get_vehicle_state() const override;
  OpticalFlowSample get_optical_flow() const override;
  BaroAltitudeSample get_baro_altitude() const override;
  bool is_link_alive(double timeout_s = 1.0) const override;

private:
  void on_local_pos(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void on_global_pos(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
  void on_vehicle_status(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void on_battery(const px4_msgs::msg::BatteryStatus::SharedPtr msg);
  void on_attitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
  void on_odometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void on_imu(const px4_msgs::msg::SensorCombined::SharedPtr msg);
  // PX4's selected GNSS receiver -> VehicleState fix_type/hdop/vdop/gnss_ok.
  // A fix PX4 got from HIL_GPS (gnss_sim_node's own injected stream, IGS-004)
  // is reported as "no fix": VehicleState's GNSS fields mean a *real*
  // receiver, and gnss_sim_node's real-GPS lockout (IGS-005) reads them --
  // counting its own stream would lock it out of itself (PX4 also leaves
  // HIL_GPS's hdop at 0, so it would always pass the threshold).
  void on_gps(const px4_msgs::msg::SensorGps::SharedPtr msg);

  void touch_link_alive();
  int open_mission_socket();

  int open_flow_socket();
  void flow_recv_loop();

  rclcpp::Node * node_;
  // DRONE_ID: this MC's MAVLink system ID and, by the DRONE_ID == MAV_SYS_ID
  // rule, the FCC's -- the target of every vehicle command and mission item
  // (mavlink_ids.hpp).
  uint8_t drone_id_;
  // Last VehicleStatus.system_id seen (0 = none yet), so a DRONE_ID mismatch
  // is logged once per change rather than at the status rate.
  uint8_t reported_fcc_system_id_{0};

  // Subscriptions  /fmu/out/  (uXRCE-DDS)
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_pos_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr sub_global_pos_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr sub_battery_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr sub_attitude_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sub_imu_;
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr sub_gps_;

  // Publishers  /fmu/in/  (uXRCE-DDS)
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_vehicle_command_;
  // vehicle_visual_odometry is typed VehicleOdometry in PX4's dds_topics.yaml
  // — there is no separate VehicleVisualOdometry message.
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr pub_visual_odometry_;
  // Published together on each stream_offboard_setpoint() call (FCC-009,
  // SDD §4.1a/§5.3a) — only while an OFFBOARD-flagged leg is active; mode
  // entry/exit sequencing lives in mission_manager_node, not here.
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offboard_control_mode_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_trajectory_setpoint_;

  // Mission protocol — MAVLink 2/UDP, always FCC_ETH_IP:FCC_MISSION_PORT,
  // independent of FCC_TRANSPORT (ICD §3.1, IR-007). Not DDS.
  int mission_sock_fd_{-1};
  std::unique_ptr<MissionProtocolClient> mission_client_;

  // Optical-flow relay — a second, independent MAVLink 2/UDP channel, always
  // FCC_ETH_IP -> FCC_FLOW_PORT (ICD §3.3a, HID-MC-001 §10). Receive-only
  // (FCC -> MC), so unlike the mission channel this has no request/response
  // client and no socket contention to serialize against — its own thread,
  // own socket, own MAVLINK_COMM_2 parser state (distinct from
  // MissionProtocolClient's MAVLINK_COMM_0 and MavlinkUdpBackend's
  // MAVLINK_COMM_1, mavlink_udp_backend.cpp's recv_loop() doc comment).
  // OpticalFlowRelay itself is thread-safe (its own doc comment) — no
  // additional lock needed around flow_relay_ here.
  int flow_sock_fd_{-1};
  std::thread flow_recv_thread_;
  std::atomic<bool> flow_stop_{false};
  OpticalFlowRelay flow_relay_;

  mutable std::mutex state_mtx_;
  VehicleState state_{};
  rclcpp::Time last_fcc_msg_time_;
};

}  // namespace fcc_bridge
