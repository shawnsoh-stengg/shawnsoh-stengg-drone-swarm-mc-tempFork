// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

#pragma once

#include <netinet/in.h>

#include <mavlink/v2.0/common/mavlink.h>

#include <cstdint>
#include <vector>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "fcc_bridge/mission_item.hpp"

namespace fcc_bridge
{

// Drives the standard MAVLink mission protocol over a UDP socket the caller
// owns (ICD §3.3 "Mission Protocol"). Shared by XrceDdsBackend (dedicated
// port 14555, SDD §3.2) and MavlinkUdpBackend (multiplexed over its existing
// socket, SDD §3.3) — stock PX4 has no DDS-writable mission topic, so this
// is the only way to get waypoints into FCC-side dataman (IR-007).
//
// Not itself a FccBackend — a helper owned by one.
class MissionProtocolClient
{
public:
  // sock_fd: an already-bound, already-connected (or sendto-ready) UDP
  // socket. Not owned — caller manages its lifetime.
  // target_system: the FCC's MAV_SYS_ID. PX4's mission handler
  // (mavlink_mission.cpp CHECK_SYSID_COMPID_MISSION) needs an exact match,
  // not 0/broadcast, and silently ignores anything else.
  // system_id/component_id: this MC's MAVLink identity for outgoing frames.
  MissionProtocolClient(
    int sock_fd, sockaddr_in target_addr, uint8_t target_system, uint8_t system_id,
    uint8_t component_id, rclcpp::Logger logger);

  // Full handshake: MISSION_COUNT -> MISSION_ITEM_INT x N -> MISSION_ACK
  // (search paths, ingress/muster legs — FR-013; also FORMATION_SLOT
  // updates, FR-020/FR-023 — see update_mission_item() below, FCC-010).
  // Blocking, single-threaded request/response loop; caller is expected to
  // run this off the control loop thread if it must not block ≥2 Hz timers.
  // overall_timeout_s bounds the whole handshake (default 5 s for the
  // infrequent initial/full upload; update_mission_item() passes its own,
  // tighter budget). Returns true on MAV_MISSION_ACCEPTED, false on timeout
  // or rejection. On success, caches `items` as current_mission_ so a
  // subsequent update_mission_item() has a base mission to re-upload.
  bool upload_mission(const std::vector<MissionItem> & items, double overall_timeout_s = 5.0);

  // FCC-010: PX4 has no handler for MISSION_WRITE_PARTIAL_LIST at any known
  // firmware version (confirmed against real PX4 source, v1.14.4 through
  // v1.17.0) — a single-item partial write can never be ACKed by real
  // firmware. Reissues the full upload_mission() handshake instead, with
  // just item `seq` replaced in the last successfully uploaded mission
  // (FORMATION_SLOT tracking, ≥2 Hz during INGRESS — FR-020/FR-023; lowered
  // from the original ≥5 Hz design to keep a full re-upload's heavier
  // dataman-write cost within one control-loop period). Requires a prior
  // successful upload_mission() call — returns false immediately (no
  // network I/O) if `seq` is out of range for the cached mission, e.g. no
  // mission uploaded yet.
  // Must not block the caller's control loop for longer than timeout_s;
  // aborts and returns false past that (SDD §5.3 — the MC must
  // self-supervise, PX4 will not).
  bool update_mission_item(uint16_t seq, const MissionItem & item, double timeout_s = 0.5);

private:
  void send_mission_count(uint16_t count);
  void send_mission_item_int(const MissionItem & item);

  // Reads one MAVLink frame within timeout_s. Returns false on timeout.
  bool recv_frame(double timeout_s, mavlink_message_t & out_msg);

  int sock_fd_;
  sockaddr_in target_addr_;
  uint8_t target_system_;
  uint8_t system_id_;
  uint8_t component_id_;
  uint8_t seq_{0};  // MAVLink frame sequence counter, wraps at 256
  rclcpp::Logger logger_;

  // Last mission successfully accepted by upload_mission() — the base
  // update_mission_item() patches a single item into before re-uploading
  // (FCC-010). Empty until the first successful upload_mission() call.
  std::vector<MissionItem> current_mission_;
};

}  // namespace fcc_bridge
