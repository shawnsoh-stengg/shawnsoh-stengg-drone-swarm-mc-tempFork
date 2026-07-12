// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §11, ICD-MC-001 §4).

#pragma once

#include <netinet/in.h>

#include <mavlink/v2.0/common/mavlink.h>

#include <cstdint>

#include "mc_msgs/msg/gcs_command.hpp"
#include "mc_msgs/msg/link_status.hpp"
#include "rclcpp/rclcpp.hpp"

// GCS-001: gcs_mavlink_node — GCS<->MC Channel B transport (ICD §4.1/§4.2),
// GCS command decode (§4.5), link-status heartbeat monitoring (§4.8, SDD
// §11.1). Zero mission/telemetry business logic here: the mission-upload
// state machine (SDD §11.2) and MC_STATUS/MC_DETECTION/MC_HEALTH telemetry
// multiplexer (SDD §11.3) are GCS-002/GCS-003 — this node only decodes
// standard COMMAND_LONG/HEARTBEAT traffic and republishes it, matching
// fcc_bridge_node's "thin adapter" philosophy for the equivalent MC<->FCC
// seam.
namespace gcs_comms
{

class GcsMavlinkNode : public rclcpp::Node
{
public:
  explicit GcsMavlinkNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~GcsMavlinkNode() override;

private:
  // Drains every datagram currently queued on the Channel B loopback socket
  // (ICD §4.2 `GCS_MAVLINK_NODE` endpoint) and decodes each MAVLink frame.
  void on_recv_timer();
  // 1 Hz: sends the MC's own HEARTBEAT (§4.4) and publishes /mc/gcs/link_status
  // (SDD §11.1 pseudocode).
  void on_heartbeat_timer();

  void handle_message(const mavlink_message_t & msg, const sockaddr_in & from);
  void handle_heartbeat(const mavlink_message_t & msg);
  void handle_command_long(const mavlink_message_t & msg, const sockaddr_in & from);

  void send_heartbeat();
  void send_command_ack(const sockaddr_in & to, uint16_t command, uint8_t result);
  void send_frame(const mavlink_message_t & msg, const sockaddr_in & to);
  void publish_link_status();

  int sock_fd_{-1};
  uint8_t system_id_;                    // Drone ID (ICD §4.1)
  static constexpr uint8_t kComponentId = 191;  // MAV_COMP_ID_ONBOARD_COMPUTER
  double gcs_link_timeout_s_;

  // Learned from the source address of the last frame mavlink-router relayed
  // in — mavlink-router's Channel B `GCS_MAVLINK_NODE` endpoint is a "Normal"
  // UDP endpoint bound to an OS-assigned ephemeral local port (ICD §4.2), not
  // a fixed, advertisable address, so outbound frames must be addressed by
  // mirroring whatever source port last sent us a frame rather than a
  // preconfigured target.
  sockaddr_in last_gcs_endpoint_{};
  bool has_gcs_endpoint_{false};

  bool has_gcs_heartbeat_{false};
  rclcpp::Time last_gcs_heartbeat_;
  uint32_t rx_packet_count_{0};
  uint32_t tx_packet_count_{0};
  uint32_t rx_drop_count_{0};

  rclcpp::Publisher<mc_msgs::msg::GcsCommand>::SharedPtr pub_command_;
  rclcpp::Publisher<mc_msgs::msg::LinkStatus>::SharedPtr pub_link_status_;

  // Both timers touch last_gcs_heartbeat_/rx_*/tx_* counters; sharing one
  // mutually-exclusive group serializes that access without a separate
  // mutex (SDD §2.1 threading model: mutually exclusive for timer callbacks).
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp::TimerBase::SharedPtr recv_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace gcs_comms
