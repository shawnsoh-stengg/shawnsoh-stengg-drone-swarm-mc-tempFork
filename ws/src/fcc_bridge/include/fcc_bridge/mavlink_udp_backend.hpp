// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

#pragma once

#include <netinet/in.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "fcc_bridge/fcc_backend.hpp"
#include "fcc_bridge/mission_protocol_client.hpp"

// FCC-007: SITL/fallback FccBackend over a single MAVLink 2/UDP socket
// (SDD §3.3). Used when SIMULATION_MODE=true.
namespace fcc_bridge
{

// Unlike XrceDdsBackend (separate DDS state/command channel + a dedicated
// mission-protocol port), this backend has exactly one socket carrying
// vehicle state, commands, and the mission protocol together — there is no
// DDS channel to fall back to in SITL. The peer (PX4 SITL) is not a fixed,
// known-in-advance address like the production FCC: it is whoever sends us
// the first datagram, learned dynamically via recvfrom() (the same
// pattern mavsdk's `udpin://` connection style uses for companion links).
class MavlinkUdpBackend : public FccBackend
{
public:
  // Reads SITL_FCC_PORT (local bind port, default 14540 — SDD §13.1) and
  // DRONE_ID from the environment. Throws std::runtime_error if the UDP
  // socket cannot be created/bound.
  explicit MavlinkUdpBackend(rclcpp::Node * node);
  ~MavlinkUdpBackend() override;

  bool upload_mission(const std::vector<MissionItem> & items) override;
  bool update_mission_item(uint16_t seq, const MissionItem & item) override;
  void stream_offboard_setpoint(const OffboardSetpoint & sp) override;
  void send_command(uint32_t cmd, const std::array<float, 7> & params) override;
  void publish_visual_odometry(const VisualOdometry & vo) override;
  VehicleState get_vehicle_state() const override;
  bool is_link_alive(double timeout_s = 1.0) const override;

private:
  void recv_loop();
  void dispatch(const mavlink_message_t & msg);
  void handle_global_position_int(const mavlink_message_t & msg);
  void handle_attitude(const mavlink_message_t & msg);
  void handle_battery_status(const mavlink_message_t & msg);
  // TEST-002: neither of these was in FCC-007's own 3-message AC (GLOBAL_
  // POSITION_INT/ATTITUDE/BATTERY_STATUS) -- that ticket explicitly deferred
  // "a real PX4 SITL/Gazebo integration test" to this one, and only a real
  // PX4 SITL instance (not FCC-007's own mock-peer unit tests) ever exercises
  // guard_arming_to_takeoff()/guard_takeoff_to_ingress(), which is what
  // surfaced VehicleState.arming_state/x/y/z silently never being populated
  // by this backend. LOCAL_POSITION_NED gives x/y/z (NED, matches
  // VehicleState's own documented frame exactly, no unit conversion needed).
  // HEARTBEAT gives arming_state via the standard MAV_MODE_FLAG_SAFETY_ARMED
  // bit -- deliberately NOT nav_state, which would need decoding PX4's own
  // custom_mode sub-mode encoding (no standard MAVLink field carries it
  // directly); every guard this ticket's SITL scenarios exercise reaches
  // RETURN via the GCS END_MISSION command path (guard_any_return) rather
  // than autonomous AUTO_LOITER detection (guard_search_to_return), so
  // nav_state stays unpopulated here -- a known, narrower follow-up gap,
  // not silently masked.
  void handle_local_position_ned(const mavlink_message_t & msg);
  void handle_heartbeat(const mavlink_message_t & msg);
  // Also not in FCC-007's original 3-message AC (see the two handlers
  // above) -- surfaced because check_visual_localisation_if_gnss_degraded()
  // (pre_arm_checks.cpp) requires VehicleState.gnss_ok, which nothing in
  // *either* backend ever set (fcc_bridge_node.cpp's msg-population code
  // has no gnss_ok/hdop/vdop/fix_type line at all) -- so PREFLIGHT could
  // never reach ARMING via a live backend, only ever in unit tests that
  // construct a synthetic VehicleState with gnss_ok set directly. Fixed
  // here for MavlinkUdpBackend/SITL only; XrceDdsBackend's own equivalent
  // (presumably a vehicle_gps_position DDS topic) is a separate, still-open
  // gap -- see PROJECT-PLAN-5-MONTH.md's TEST-002 entry.
  void handle_gps_raw_int(const mavlink_message_t & msg);
  void note_peer(const sockaddr_in & addr);

  // Lazily constructs mission_client_ against the now-known peer address.
  // Returns false (and logs) if no peer has been seen yet — there is
  // nowhere to send a mission request until PX4 SITL has spoken first.
  // Caller must already hold socket_use_mtx_.
  bool ensure_mission_client();

  rclcpp::Node * node_;
  int sock_fd_{-1};

  std::thread recv_thread_;
  std::atomic<bool> stop_{false};

  // Excludes recv_loop_'s socket reads while a mission-protocol handshake
  // (upload_mission/update_mission_item) is in progress. MissionProtocolClient
  // does its own raw recv() on this same fd and is reused unmodified (SDD
  // §3.3) — it has no built-in tolerance for a state-telemetry datagram
  // landing in the exact slot it expects a MISSION_ACK/MISSION_REQUEST_INT
  // reply (see upload_mission()'s single-shot ack read and
  // serve_single_item_request()), so recv_loop_ must not be pulling
  // datagrams off the same socket concurrently with a handshake. This
  // narrows the race to "PX4 keeps streaming telemetry mid-handshake and
  // outraces the reply in the queue" — a real but low-probability hazard
  // that only a genuine PX4 SITL/Gazebo run could trigger, which is
  // explicitly out of scope for this ticket (mock-peer tests only).
  std::mutex socket_use_mtx_;

  mutable std::mutex peer_mtx_;
  bool peer_known_{false};
  sockaddr_in peer_addr_{};

  std::unique_ptr<MissionProtocolClient> mission_client_;

  mutable std::mutex state_mtx_;
  VehicleState state_{};
  rclcpp::Time last_fcc_msg_time_;
};

}  // namespace fcc_bridge
