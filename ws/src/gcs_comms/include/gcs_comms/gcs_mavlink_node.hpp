// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §11, ICD-MC-001 §4).

#pragma once

#include <netinet/in.h>

#include <mavlink/v2.0/common/mavlink.h>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

#include "mc_msgs/msg/detection.hpp"
#include "mc_msgs/msg/detection_array.hpp"
#include "mc_msgs/msg/gcs_command.hpp"
#include "mc_msgs/msg/gcs_command_ack.hpp"
#include "mc_msgs/msg/link_status.hpp"
#include "mc_msgs/msg/localisation_mode.hpp"
#include "mc_msgs/msg/mission_phase.hpp"
#include "mc_msgs/msg/mission_upload.hpp"
#include "mc_msgs/msg/status_text.hpp"
#include "mc_msgs/msg/system_health.hpp"
#include "mc_msgs/msg/vehicle_state.hpp"
#include "gcs_comms/upload_id.hpp"
#include "rclcpp/rclcpp.hpp"

// GCS-001/GCS-002/GCS-003: gcs_mavlink_node — GCS<->MC Channel B transport
// (ICD §4.1/§4.2), GCS command decode (§4.5), link-status heartbeat
// monitoring (§4.8, SDD §11.1), the mission-upload state machine (SDD §11.2,
// ICD §4.6 MC_MISSION_UPLOAD), and the MC_STATUS/MC_DETECTION/MC_HEALTH
// telemetry multiplexer (SDD §11.3). Matches fcc_bridge_node's "thin
// adapter" philosophy for the equivalent MC<->FCC seam: it only reads other
// topics and republishes them, zero business logic.
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
  // false => param1 isn't a flyable skip_phase_mask; the caller rejects
  // the whole upload rather than letting it fail pre-arm later.
  bool decode_mission_config_item(const mavlink_mission_item_int_t & item);

  // --- GCS-003: telemetry multiplexer (SDD §11.3, ICD §4.6 MC_STATUS/
  // MC_DETECTION/MC_HEALTH). Subscriptions run on mux_sub_cb_group_
  // (reentrant) and the aggregation timer on mux_timer_cb_group_ (mutually
  // exclusive) — both distinct from timer_cb_group_ above, so a burst of
  // detection traffic can never queue behind, or delay, GCS-001's
  // command-decode path on recv_timer_ (SDD §2.1 threading model:
  // reentrant for subscription callbacks, mutually exclusive for timers).
  void on_vehicle_state(const mc_msgs::msg::VehicleState::SharedPtr msg);
  void on_mission_phase(const mc_msgs::msg::MissionPhase::SharedPtr msg);
  void on_localisation_mode(const mc_msgs::msg::LocalisationMode::SharedPtr msg);
  // Also the event-driven trigger for an immediate MC_HEALTH publish (SDD
  // §11.3: "health updates are event-driven").
  void on_health(const mc_msgs::msg::SystemHealth::SharedPtr msg);
  // Event-driven: emits one MC_DETECTION per element (ICD §4.6 MC_DETECTION
  // is a single-target message; DetectionArray may carry several per frame).
  void on_detections(const mc_msgs::msg::DetectionArray::SharedPtr msg);
  // 500 ms (SDD §11.3): aggregates the cached latest_* values below into one
  // outgoing MC_STATUS.
  void on_mc_status_timer();

  void send_mc_status();
  void send_mc_detection(const mc_msgs::msg::Detection & detection);
  void send_mc_health(const mc_msgs::msg::SystemHealth & health);

  // TEST-007 finding: mission_manager_node's outcome-level ack and its
  // /mc/status_text reasons (e.g. EXEC-002 pre-arm failures) never reached
  // the GCS, so a rejected MC_CMD_PHASE_TRANSITION looked identical to an
  // accepted one. handle_command_long() now answers that command with
  // MAV_RESULT_IN_PROGRESS, and on_command_ack() sends the final
  // COMMAND_ACK (ACCEPTED/DENIED) plus a STATUSTEXT carrying the reason.
  void on_command_ack(const mc_msgs::msg::GcsCommandAck::SharedPtr msg);
  // Relays every /mc/status_text (mission_manager_node, health_monitor_node,
  // gnss_sim_node) as a MAVLink STATUSTEXT (ICD §4.7).
  void on_status_text(const mc_msgs::msg::StatusText::SharedPtr msg);
  // Splits texts over 50 chars into MAVLink v2 STATUSTEXT chunks; anything
  // beyond kMaxStatustextChunks * 50 chars is cut.
  void send_statustext(uint8_t severity, const std::string & text);
  static constexpr size_t kMaxStatustextChunks = 4;

  mc_msgs::msg::VehicleState latest_vehicle_state_;
  // Gates on_mc_status_timer() — nothing meaningful to report before
  // fcc_bridge_node's first /mc/fcc/vehicle_state publish.
  bool has_vehicle_state_{false};
  uint8_t latest_mission_phase_{mc_msgs::msg::MissionPhase::PREFLIGHT};
  uint8_t latest_localisation_mode_{mc_msgs::msg::LocalisationMode::MODE_GNSS_PRIMARY};
  // Cached from /mc/health for MC_STATUS.fault_mask; the full SystemHealth
  // message itself is relayed immediately as MC_HEALTH in on_health(),
  // rather than waiting for the next timer tick (SDD §11.3).
  uint16_t latest_fault_mask_{0};

  rclcpp::Subscription<mc_msgs::msg::VehicleState>::SharedPtr sub_vehicle_state_;
  rclcpp::Subscription<mc_msgs::msg::MissionPhase>::SharedPtr sub_mission_phase_;
  rclcpp::Subscription<mc_msgs::msg::LocalisationMode>::SharedPtr sub_localisation_mode_;
  rclcpp::Subscription<mc_msgs::msg::SystemHealth>::SharedPtr sub_health_;
  rclcpp::Subscription<mc_msgs::msg::DetectionArray>::SharedPtr sub_detections_;
  rclcpp::Subscription<mc_msgs::msg::GcsCommandAck>::SharedPtr sub_command_ack_;
  rclcpp::Subscription<mc_msgs::msg::StatusText>::SharedPtr sub_status_text_;
  rclcpp::CallbackGroup::SharedPtr mux_sub_cb_group_;
  rclcpp::CallbackGroup::SharedPtr mux_timer_cb_group_;
  rclcpp::TimerBase::SharedPtr mc_status_timer_;

  // Guards last_gcs_endpoint_/has_gcs_endpoint_ and serializes every
  // pack+finalize+send call site (existing ones included). Before GCS-003,
  // every outbound frame was sent from recv_timer_/heartbeat_timer_, which
  // already shared one MutuallyExclusive group — safe by construction.
  // GCS-003 adds a second, concurrently-running execution context
  // (mux_sub_cb_group_/mux_timer_cb_group_ above) that also sends, and
  // mavlink_finalize_message() (mavlink_helpers.h) mutates process-global
  // per-channel state (MAVLINK_COMM_0's outbound sequence counter) that was
  // never designed for concurrent callers — this mutex is the fix.
  std::mutex send_mutex_;
  // Last STATUSTEXT chunk-sequence id used (send_mutex_); 0 is never sent
  // for a chunked text.
  uint16_t statustext_id_{0};

  MissionUploadState mission_state_{MissionUploadState::IDLE};
  uint16_t mission_expected_count_{0};
  uint16_t mission_items_received_{0};
  mc_msgs::msg::MissionUpload mission_upload_msg_;
  UploadIdAllocator upload_ids_{UploadSource::kChannelB};  // see upload_id.hpp
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
  // Written under send_mutex_ (every send site), but read by
  // publish_link_status() on timer_cb_group_ without taking that lock —
  // atomic rather than requiring a lock just for this one read.
  std::atomic<uint32_t> tx_packet_count_{0};
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
