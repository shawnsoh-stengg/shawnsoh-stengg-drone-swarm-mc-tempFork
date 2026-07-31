// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §11, ICD-MC-001 §4).

#pragma once

#include <netinet/in.h>

#include <mavlink/v2.0/common/mavlink.h>

#include <cstdint>

#include "mc_msgs/msg/gcs_command.hpp"
#include "mc_msgs/msg/link_status.hpp"
#include "mc_msgs/msg/mission_upload.hpp"
#include "rclcpp/rclcpp.hpp"

// GCS-001/GCS-002: gcs_mavlink_node — GCS<->MC Channel B transport (ICD
// §4.1/§4.2), GCS command decode (§4.5), link-status heartbeat monitoring
// (§4.8, SDD §11.1), and the mission-upload state machine (SDD §11.2, ICD
// §4.6 MC_MISSION_UPLOAD). MC_STATUS/MC_DETECTION/MC_HEALTH telemetry
// multiplexer (SDD §11.3) is still GCS-003 — that piece only reads other
// topics and republishes them, matching fcc_bridge_node's "thin adapter"
// philosophy for the equivalent MC<->FCC seam.
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

  // --- GCS-002: mission upload state machine (SDD §11.2, ICD §4.6) ---

  enum class MissionUploadState : uint8_t
  {
    IDLE,
    RECEIVING,
  };

  void handle_mission_count(const mavlink_message_t & msg);
  void handle_mission_item_int(const mavlink_message_t & msg);
  // Called every on_recv_timer tick (10 ms): resends MISSION_REQUEST_INT if
  // kMissionRequestRetryS has elapsed without progress, aborts the upload
  // past kMissionUploadTimeoutS total (SDD §11.2 "on timeout").
  void check_mission_upload_timers();

  void send_mission_request_int(uint16_t seq);
  void send_mission_ack(uint8_t result);
  void abort_mission_upload(const char * reason);

  // One handler per ICD §4.6 MC_MISSION_UPLOAD `command` value. Each decodes
  // its slice of `mavlink_mission_item_int_t` into the matching
  // mc_msgs/MissionUpload field — see the ICD §4.6 gap notes for the
  // encoding choices (mm-scaled N/E offsets, muster point folded into
  // ingress_waypoints, reconstructed swarm-peer IP, etc).
  void decode_ingress_or_muster_item(const mavlink_mission_item_int_t & item);
  void decode_formation_offset_item(const mavlink_mission_item_int_t & item);
  void decode_search_area_vertex_item(const mavlink_mission_item_int_t & item);
  void decode_swarm_peer_entry_item(const mavlink_mission_item_int_t & item);

  MissionUploadState mission_state_{MissionUploadState::IDLE};
  uint16_t mission_expected_count_{0};
  uint16_t mission_items_received_{0};
  mc_msgs::msg::MissionUpload mission_upload_msg_;
  uint32_t next_upload_id_{1};
  // The GCS peer's own MAVLink identity, captured from the MISSION_COUNT
  // frame that opened this upload — outbound MISSION_REQUEST_INT/MISSION_ACK
  // target these rather than a hardcoded broadcast address (unlike
  // send_command_ack's target_system=255, which never needs to reach a
  // specific requester back).
  uint8_t mission_gcs_sysid_{0};
  uint8_t mission_gcs_compid_{0};
  rclcpp::Time mission_last_activity_;      // last progress; gates mission_upload_timeout_s_
  rclcpp::Time mission_last_request_sent_;  // gates mission_request_retry_s_
  // Declared parameters (defaults match SDD §11.2 / ICD §4.8) rather than
  // fixed constants so tests don't have to burn wall-clock seconds on the
  // retry/timeout paths — same rationale as gcs_link_timeout_s_ below.
  double mission_request_retry_s_;
  double mission_upload_timeout_s_;

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
  rclcpp::Publisher<mc_msgs::msg::MissionUpload>::SharedPtr pub_mission_upload_;

  // Both timers touch last_gcs_heartbeat_/rx_*/tx_* counters; sharing one
  // mutually-exclusive group serializes that access without a separate
  // mutex (SDD §2.1 threading model: mutually exclusive for timer callbacks).
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp::TimerBase::SharedPtr recv_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace gcs_comms
