// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

#pragma once

#include <netinet/in.h>

#include <memory>
#include <mutex>
#include <vector>

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fcc_bridge/fcc_backend.hpp"
#include "fcc_bridge/mission_protocol_client.hpp"

// FCC-002: production FccBackend over uXRCE-DDS (SDD §3.2, ICD §3).
namespace fcc_bridge
{

class XrceDdsBackend : public FccBackend
{
public:
  // Reads FCC_TRANSPORT (serial|ethernet), FCC_SERIAL_DEVICE, FCC_SERIAL_BAUD,
  // FCC_ETH_IP, FCC_ETH_PORT, FCC_MISSION_PORT from the environment
  // (ICD §9.4). Throws std::runtime_error if the mission-protocol socket
  // cannot be created/bound.
  explicit XrceDdsBackend(rclcpp::Node * node);
  ~XrceDdsBackend() override;

  bool upload_mission(const std::vector<MissionItem> & items) override;
  bool update_mission_item(uint16_t seq, const MissionItem & item) override;
  void send_command(uint32_t cmd, const std::array<float, 7> & params) override;
  void publish_visual_odometry(const VisualOdometry & vo) override;
  VehicleState get_vehicle_state() const override;
  bool is_link_alive(double timeout_s = 1.0) const override;

private:
  void on_local_pos(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void on_global_pos(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
  void on_vehicle_status(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void on_battery(const px4_msgs::msg::BatteryStatus::SharedPtr msg);
  void on_attitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
  void on_odometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void on_imu(const px4_msgs::msg::SensorCombined::SharedPtr msg);

  void touch_link_alive();
  int open_mission_socket();

  rclcpp::Node * node_;

  // Subscriptions  /fmu/out/  (uXRCE-DDS)
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_pos_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr sub_global_pos_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr sub_battery_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr sub_attitude_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sub_imu_;

  // Publishers  /fmu/in/  (uXRCE-DDS)
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_vehicle_command_;
  // vehicle_visual_odometry is typed VehicleOdometry in PX4's dds_topics.yaml
  // — there is no separate VehicleVisualOdometry message.
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr pub_visual_odometry_;

  // Mission protocol — MAVLink 2/UDP, always 192.168.12.x:FCC_MISSION_PORT,
  // independent of FCC_TRANSPORT (ICD §3.1, IR-007). Not DDS.
  int mission_sock_fd_{-1};
  std::unique_ptr<MissionProtocolClient> mission_client_;

  mutable std::mutex state_mtx_;
  VehicleState state_{};
  rclcpp::Time last_fcc_msg_time_;
};

}  // namespace fcc_bridge
