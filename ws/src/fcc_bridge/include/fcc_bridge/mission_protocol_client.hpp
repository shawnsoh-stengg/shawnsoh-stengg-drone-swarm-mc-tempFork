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
  // system_id/component_id: this MC's MAVLink identity for outgoing frames.
  MissionProtocolClient(
    int sock_fd, sockaddr_in target_addr, uint8_t system_id, uint8_t component_id,
    rclcpp::Logger logger);

  // Full handshake: MISSION_COUNT -> MISSION_ITEM_INT x N -> MISSION_ACK
  // (search paths, ingress/muster legs — FR-013). Blocking, single-threaded
  // request/response loop; caller is expected to run this off the control
  // loop thread if it must not block ≥5 Hz timers.
  // Returns true on MAV_MISSION_ACCEPTED, false on timeout or rejection.
  bool upload_mission(const std::vector<MissionItem> & items);

  // Partial update: MISSION_WRITE_PARTIAL_LIST(seq,seq) -> ... ->
  // MISSION_ACK (FORMATION_SLOT tracking, ≥5 Hz during INGRESS — FR-020/
  // FR-023). Must not block the caller's control loop for longer than
  // timeout_s; aborts and returns false past that (SDD §5.3 — the MC must
  // self-supervise, PX4 will not).
  bool update_mission_item(uint16_t seq, const MissionItem & item, double timeout_s = 0.5);

private:
  // Waits up to timeout_s for a MISSION_REQUEST_INT with a matching seq,
  // replies with MISSION_ITEM_INT, then waits for MISSION_ACK.
  // Returns true iff MAV_MISSION_ACCEPTED was received.
  bool serve_single_item_request(const MissionItem & item, double timeout_s);

  void send_mission_count(uint16_t count);
  void send_mission_item_int(const MissionItem & item);
  void send_mission_write_partial_list(uint16_t start_seq, uint16_t end_seq);

  // Reads one MAVLink frame within timeout_s. Returns false on timeout.
  bool recv_frame(double timeout_s, mavlink_message_t & out_msg);

  int sock_fd_;
  sockaddr_in target_addr_;
  uint8_t system_id_;
  uint8_t component_id_;
  uint8_t seq_{0};  // MAVLink frame sequence counter, wraps at 256
  rclcpp::Logger logger_;
};

}  // namespace fcc_bridge
