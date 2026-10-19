// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §11, ICD-MC-001 §12).

#pragma once

#include <netinet/in.h>

#include <mavlink/v2.0/common/mavlink.h>

#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "mc_msgs/msg/mission_phase.hpp"
#include "mc_msgs/msg/mission_upload.hpp"
#include "gcs_comms/upload_id.hpp"
#include "rclcpp/rclcpp.hpp"

// GCS-004: mission_orchestrator_node — MAVLink Channel C mission-plan
// library (MISSION-ORCHESTRATOR-MAVLINK-INTERFACE-SPEC.md, ICD-MC-001 §12).
// Implements an MAVFTP server (adapted from PX4's mavlink_ftp.cpp — single
// session, flat namespace, no subdirectories) over /var/drone-mc/mission_plans/
// for create/update/list/read/delete (ICD §12.3), plus the two custom
// MC_MISSION_PLAN_ACTIVATE/ACK messages (ICD §12.2) for the one operation
// that isn't a filesystem call. Matches gcs_mavlink_node's "thin adapter"
// role for its own channel: activate ends by publishing exactly the same
// mc_msgs/MissionUpload on /mc/gcs/mission_upload that gcs_mavlink_node
// already produces today (ICD §12.5) — mission_manager_node needs zero
// changes.
namespace gcs_comms
{

class MissionOrchestratorNode : public rclcpp::Node
{
public:
  explicit MissionOrchestratorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MissionOrchestratorNode() override;

private:
  // --- Channel C raw-frame transport ---
  //
  // This translation unit only compiles the plain common MAVLink dialect
  // (mc_mavlink_defs.xml's header comment: no mavgen build step). That
  // dialect's generated CRC table has no entry for MC_MISSION_PLAN_ACTIVATE
  // (50104), so mavlink_parse_char() cannot receive it at all — it treats
  // every such frame as a checksum failure and silently drops it rather
  // than decoding it (same finding gcs_comms/test/test_gcs_mavlink_node.cpp's
  // RawMavlinkFrame comment documents for the send side of this exact
  // problem). mavlink-router-channel-c.conf's ORCHESTRATOR_MAVLINK_NODE
  // endpoint is a UDP socket, so each recvfrom() datagram is exactly one
  // complete MAVLink v2 frame — decoding the fixed v2 header directly
  // sidesteps mavlink_parse_char()'s dialect-table lookup entirely, for
  // both FILE_TRANSFER_PROTOCOL (110, standard) and MC_MISSION_PLAN_ACTIVATE
  // (50104, custom) alike.
  struct RawFrame
  {
    uint32_t msgid{0};
    uint8_t sysid{0};
    uint8_t compid{0};
    // 254 covers the largest message this node ever decodes
    // (FILE_TRANSFER_PROTOCOL); zero-filled before the real wire bytes are
    // copied in, matching the zero-pad convention test_gcs_mavlink_node.cpp
    // already established for the same class of hand-decoded frame.
    std::array<uint8_t, 254> payload{};
  };

  void on_recv_timer();
  static bool decode_raw_v2_frame(const uint8_t * buf, ssize_t n, RawFrame & out);

  void handle_frame(const RawFrame & frame, const sockaddr_in & from);
  void handle_ftp_request(const RawFrame & frame, const sockaddr_in & from);
  void handle_activate_request(const RawFrame & frame, const sockaddr_in & from);

  void send_frame(const mavlink_message_t & msg, const sockaddr_in & to);
  void send_statustext(uint8_t severity, const std::string & text);
  void send_mission_plan_ack(
    const sockaddr_in & to, uint32_t client_request_id, const std::string & filename,
    uint8_t result, const std::string & error_message);

  // --- MAVFTP server (ICD §12.3) ---

  struct __attribute__((packed)) FtpHeader
  {
    uint16_t seq_number;
    uint8_t session;
    uint8_t opcode;
    uint8_t size;
    uint8_t req_opcode;
    uint8_t burst_complete;
    uint8_t padding;
    uint32_t offset;
  };

  enum FtpOpcode : uint8_t
  {
    kFtpOpNone = 0,
    kFtpOpTerminateSession = 1,
    kFtpOpResetSessions = 2,
    kFtpOpListDirectory = 3,
    kFtpOpOpenFileRO = 4,
    kFtpOpReadFile = 5,
    kFtpOpCreateFile = 6,
    kFtpOpWriteFile = 7,
    kFtpOpRemoveFile = 8,
    kFtpOpOpenFileWO = 11,
    kFtpOpBurstReadFile = 15,
    kFtpOpAck = 128,
    kFtpOpNak = 129,
  };

  enum FtpError : uint8_t
  {
    kFtpErrNone = 0,
    kFtpErrFail = 1,
    kFtpErrFailErrno = 2,
    kFtpErrInvalidDataSize = 3,
    kFtpErrInvalidSession = 4,
    kFtpErrNoSessionsAvailable = 5,
    kFtpErrEOF = 6,
    kFtpErrUnknownCommand = 7,
    kFtpErrFileExists = 8,
    kFtpErrFileProtected = 9,
    kFtpErrFileNotFound = 10,
  };

  // 251-byte FILE_TRANSFER_PROTOCOL.payload minus the 12-byte FtpHeader —
  // the same value pymavlink's mavftp.py (MAX_Payload) and PX4's
  // mavlink_ftp.h (kMaxDataLength) both use.
  static constexpr uint8_t kFtpMaxData = 239;

  enum class SessionMode
  {
    kNone,
    kRead,
    kWrite,
    kActiveRead,  // synthetic "/_active" read session — no real fd (ICD §12.3)
  };

  FtpError ftp_dispatch(
    const FtpHeader & hdr, const uint8_t * data, uint8_t & out_size,
    uint8_t * out_data);
  FtpError ftp_list_directory(uint32_t entry_offset, uint8_t & out_size, uint8_t * out_data);
  FtpError ftp_open(
    const std::string & virtual_path, bool for_write, bool create_only, uint8_t & out_size,
    uint8_t * out_data);
  FtpError ftp_read(uint32_t offset, uint8_t size, uint8_t & out_size, uint8_t * out_data);
  FtpError ftp_write(
    uint32_t offset, const uint8_t * data, uint8_t size, uint8_t & out_size, uint8_t * out_data);
  FtpError ftp_remove(const std::string & virtual_path);
  FtpError ftp_terminate();
  FtpError ftp_reset();

  void handle_burst_read(
    uint16_t seq, uint8_t sysid, uint8_t compid, uint32_t offset, const sockaddr_in & to);
  void send_ftp_reply(
    uint16_t seq, uint8_t opcode, uint8_t req_opcode, uint8_t burst_complete, uint32_t offset,
    const uint8_t * data, uint8_t size, uint8_t target_sysid, uint8_t target_compid,
    const sockaddr_in & to);

  bool resolve_plan_path(
    const std::string & virtual_path, std::string & out_full_path,
    std::string & out_filename) const;

  // Normal completion (TerminateSession on a write session): truncates to
  // the final written length (ICD §12.3 "Update" row) then validates the
  // JSON on disk, publishing a STATUSTEXT warning if it doesn't parse
  // (ICD §12.6) — the file itself stays on disk regardless (§5.1 item 2 of
  // the interface spec: activate is what actually rejects a bad plan).
  void finish_write_session();
  // Abort path (ResetSessions, or TerminateSession on a non-write session):
  // just closes whatever's open, no truncate/validate side effects — mirrors
  // PX4's own _workReset, which never touches file content on reset.
  void close_session_fd_only();
  void close_session_state();

  // --- Plan library / activate (ICD §12.4/§12.5) ---

  enum ActivateResult : uint8_t
  {
    kActivateAccepted = 0,
    kActivateRejectedInvalidJson = 1,
    kActivateRejectedNotFound = 2,
    kActivateRejectedPhase = 3,
  };
  ActivateResult load_and_activate_plan(
    const std::string & filename,
    mc_msgs::msg::MissionUpload & out_msg);
  bool parse_plan_json(const std::string & full_path, mc_msgs::msg::MissionUpload & out_msg) const;
  void on_mission_phase(const mc_msgs::msg::MissionPhase::SharedPtr msg);

  std::string plans_dir_;
  // ICD §12.3's synthetic "/_active" entry — empty if no plan has ever been
  // activated. In-memory only: does not need to survive a node restart any
  // more than mission_manager_node's own in-memory phase state does.
  std::string active_plan_filename_;
  UploadIdAllocator upload_ids_{UploadSource::kChannelC};  // see upload_id.hpp

  // Gates MC_MISSION_PLAN_ACTIVATE (interface spec §5.2/§9 item 2, resolved
  // here as: restrict to PREFLIGHT, the recommended "before ARMING" window —
  // the implemented mission_manager_node state machine (SDD §5.1) has no
  // HOLD phase, so PREFLIGHT is the only pre-arm phase that exists to allow.
  // Defaults to PREFLIGHT, same as gcs_mavlink_node's latest_mission_phase_,
  // since mission_manager_node publishes that as its own initial phase.
  uint8_t latest_mission_phase_{mc_msgs::msg::MissionPhase::PREFLIGHT};

  SessionMode session_mode_{SessionMode::kNone};
  int session_fd_{-1};
  std::string session_filename_;
  // Max (offset + bytes written) seen this write session — the truncate
  // target at TerminateSession (finish_write_session()).
  uint32_t session_write_high_water_{0};
  std::vector<uint8_t> active_read_buffer_;  // backing store for kActiveRead sessions

  int sock_fd_{-1};
  uint8_t system_id_;                            // Drone ID (ICD §12.1)
  static constexpr uint8_t kComponentId = 191;   // MAV_COMP_ID_ONBOARD_COMPUTER

  // Learned from the source address of the last frame mavlink-router relayed
  // in, same rationale as gcs_mavlink_node's last_gcs_endpoint_ (its own
  // header comment): mavlink-router-channel-c.conf's ORCHESTRATOR_MAVLINK_NODE
  // endpoint has no fixed, advertisable address either.
  sockaddr_in last_orchestrator_endpoint_{};
  bool has_orchestrator_endpoint_{false};

  // Guards every pack+finalize+send call site — mavlink_finalize_message()
  // (mavlink_helpers.h) mutates process-global per-channel state
  // (MAVLINK_COMM_0's outbound sequence counter); on_mission_phase() below
  // never sends, so recv_timer_ is still the only path that does, but every
  // send helper takes this lock anyway to match the rest of this package's
  // established convention (gcs_mavlink_node.hpp's send_mutex_) rather than
  // relying on that happening to hold.
  std::mutex send_mutex_;

  rclcpp::Publisher<mc_msgs::msg::MissionUpload>::SharedPtr pub_mission_upload_;

  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp::TimerBase::SharedPtr recv_timer_;

  // Own callback group, distinct from timer_cb_group_ — same rationale as
  // gcs_mavlink_node's mux_sub_cb_group_ (its own header comment): a mission-
  // phase update should never queue behind Channel C's recv_timer_.
  rclcpp::CallbackGroup::SharedPtr phase_sub_cb_group_;
  rclcpp::Subscription<mc_msgs::msg::MissionPhase>::SharedPtr sub_mission_phase_;
};

}  // namespace gcs_comms
