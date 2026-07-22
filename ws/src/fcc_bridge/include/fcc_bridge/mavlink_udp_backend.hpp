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
