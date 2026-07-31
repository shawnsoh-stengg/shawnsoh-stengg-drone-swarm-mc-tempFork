// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §11, ICD-MC-001 §4).

#include "gcs_comms/gcs_mavlink_node.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <string>

#include "geographic_msgs/msg/geo_point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mc_msgs/msg/formation_offset.hpp"
#include "mc_msgs/msg/swarm_peer.hpp"

namespace gcs_comms
{

namespace
{
constexpr double kDefaultGcsLinkTimeoutS = 10.0;  // ICD §4.8

// GCS-002: mission upload state machine (SDD §11.2, ICD §4.6 MC_MISSION_UPLOAD).
// A custom mission_type distinguishes this handshake from any (channel-
// isolated, so never actually seen here) FCC-bound MAVLINK_MSG_ID_MISSION_*
// traffic — not in the common dialect's MAV_MISSION_TYPE enum, hence the raw
// literal rather than a generated constant.
constexpr uint8_t kMavMissionTypeMc = 3;

// ICD §4.6 mission item `command` values for MC_MISSION_UPLOAD.
constexpr uint16_t kCmdIngressWaypoint = 50200;
constexpr uint16_t kCmdMusterPoint = 50201;
constexpr uint16_t kCmdFormationOffset = 50202;
constexpr uint16_t kCmdSearchAreaVertex = 50203;
constexpr uint16_t kCmdSwarmPeerEntry = 50204;

// Beyond SDD §11.2's pseudocode (a single 5 s abort timeout): re-requests
// the outstanding item at this cadence rather than waiting the full 5 s on
// one dropped UDP frame — standard MAVLink mission-protocol GCS behaviour,
// applied here since gcs_mavlink_node plays the "receiver" role for this
// handshake (SDD §11.2: it sends MISSION_REQUEST_INT and waits). Defaults
// for the mission_request_retry_s_/mission_upload_timeout_s_ parameters.
constexpr double kDefaultMissionRequestRetryS = 1.0;
constexpr double kDefaultMissionUploadTimeoutS = 5.0;  // SDD §11.2

// ICD §11.1 fixes the MC/peer subnet to 192.168.x.x; command 50204 (swarm
// peer entry) only transmits the last two octets (ICD §4.6 gap note).
constexpr uint8_t kSwarmPeerIpOctet1 = 192;
constexpr uint8_t kSwarmPeerIpOctet2 = 168;

std::string env_or(const char * name, const std::string & fallback)
{
  const char * v = std::getenv(name);
  return v != nullptr ? std::string(v) : fallback;
}

int open_loopback_socket(uint16_t port)
{
  const int fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
  if (fd < 0) {
    throw std::runtime_error("GcsMavlinkNode: failed to create Channel B UDP socket");
  }
  constexpr int kReuseAddr = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &kReuseAddr, sizeof(kReuseAddr));

  sockaddr_in local_addr{};
  local_addr.sin_family = AF_INET;
  local_addr.sin_port = htons(port);
  // Loopback only — this is the `GCS_MAVLINK_NODE` endpoint of
  // mavlink-router-channel-b.conf (ICD §4.2), never reachable off-box.
  local_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (::bind(fd, reinterpret_cast<const sockaddr *>(&local_addr), sizeof(local_addr)) != 0) {
    ::close(fd);
    throw std::runtime_error(
            "GcsMavlinkNode: failed to bind Channel B UDP socket to port " +
            std::to_string(port));
  }
  return fd;
}
}  // namespace

GcsMavlinkNode::GcsMavlinkNode(const rclcpp::NodeOptions & options)
: Node("gcs_mavlink_node", options),
  mission_last_activity_(now()),
  mission_last_request_sent_(now()),
  mission_request_retry_s_(
    declare_parameter("mission_request_retry_s", kDefaultMissionRequestRetryS)),
  mission_upload_timeout_s_(
    declare_parameter("mission_upload_timeout_s", kDefaultMissionUploadTimeoutS)),
  gcs_link_timeout_s_(declare_parameter("gcs_link_timeout_s", kDefaultGcsLinkTimeoutS)),
  last_gcs_heartbeat_(now())
{
  system_id_ = static_cast<uint8_t>(std::stoi(env_or("DRONE_ID", "1")));
  const uint16_t loopback_port = static_cast<uint16_t>(
    std::stoi(env_or("GCS_MAVLINK_LOOPBACK_PORT", "14560")));
  sock_fd_ = open_loopback_socket(loopback_port);

  pub_command_ = create_publisher<mc_msgs::msg::GcsCommand>("/mc/gcs/command", rclcpp::QoS(10));
  pub_link_status_ =
    create_publisher<mc_msgs::msg::LinkStatus>("/mc/gcs/link_status", rclcpp::QoS(10));
  pub_mission_upload_ =
    create_publisher<mc_msgs::msg::MissionUpload>("/mc/gcs/mission_upload", rclcpp::QoS(10));

  timer_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  recv_timer_ = create_wall_timer(
    std::chrono::milliseconds(10),
    [this] {on_recv_timer();},
    timer_cb_group_);
  heartbeat_timer_ = create_wall_timer(
    std::chrono::milliseconds(1000),
    [this] {on_heartbeat_timer();},
    timer_cb_group_);

  RCLCPP_INFO(
    get_logger(),
    "gcs_mavlink_node started: system_id=%u, Channel B loopback port=%u, gcs_link_timeout=%.1f s",
    system_id_, loopback_port, gcs_link_timeout_s_);
}

GcsMavlinkNode::~GcsMavlinkNode()
{
  if (sock_fd_ >= 0) {
    ::close(sock_fd_);
  }
}

void GcsMavlinkNode::on_recv_timer()
{
  // Time-based, not event-based — must run every tick regardless of whether
  // any datagram arrived this cycle (SDD §11.2 retry/timeout).
  check_mission_upload_timers();

  uint8_t buf[512];
  sockaddr_in from{};
  socklen_t from_len = sizeof(from);

  while (true) {
    const ssize_t n = ::recvfrom(
      sock_fd_, buf, sizeof(buf), 0, reinterpret_cast<sockaddr *>(&from), &from_len);
    if (n <= 0) {
      break;  // EAGAIN/EWOULDBLOCK — nothing more queued this tick
    }

    mavlink_message_t msg;
    mavlink_status_t status;
    for (ssize_t i = 0; i < n; ++i) {
      if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
        ++rx_packet_count_;
        // Cumulative since channel init, not a per-call delta — overwrite,
        // don't accumulate.
        rx_drop_count_ = status.packet_rx_drop_count;
        handle_message(msg, from);
      }
    }
  }
}

void GcsMavlinkNode::handle_message(const mavlink_message_t & msg, const sockaddr_in & from)
{
  // Learn/refresh mavlink-router's relay endpoint from every frame it
  // forwards us, not just heartbeats, so replies always target the freshest
  // known source (header comment, last_gcs_endpoint_).
  last_gcs_endpoint_ = from;
  has_gcs_endpoint_ = true;

  switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
      handle_heartbeat(msg);
      break;
    case MAVLINK_MSG_ID_COMMAND_LONG:
      handle_command_long(msg, from);
      break;
    case MAVLINK_MSG_ID_MISSION_COUNT:
      handle_mission_count(msg);
      break;
    case MAVLINK_MSG_ID_MISSION_ITEM_INT:
      handle_mission_item_int(msg);
      break;
    default:
      break;
  }
}

void GcsMavlinkNode::handle_heartbeat(const mavlink_message_t & msg)
{
  mavlink_heartbeat_t hb{};
  mavlink_msg_heartbeat_decode(&msg, &hb);

  // ICD §4.3: GCS heartbeat is type=6 (MAV_TYPE_GCS), autopilot=8
  // (MAV_AUTOPILOT_INVALID) — filters out any other stray heartbeat traffic
  // that might reach this loopback (there should be none given Channel B
  // isolation, but this keeps the check meaningful rather than tautological).
  if (hb.type != MAV_TYPE_GCS || hb.autopilot != MAV_AUTOPILOT_INVALID) {
    return;
  }
  has_gcs_heartbeat_ = true;
  last_gcs_heartbeat_ = now();
}

void GcsMavlinkNode::handle_command_long(const mavlink_message_t & msg, const sockaddr_in & from)
{
  mavlink_command_long_t cmd{};
  mavlink_msg_command_long_decode(&msg, &cmd);

  // ICD §4.5: all MC-directed commands target this MC's system_id and
  // MAV_COMP_ID_ONBOARD_COMPUTER (191).
  if (cmd.target_system != system_id_ || cmd.target_component != kComponentId) {
    return;
  }

  mc_msgs::msg::GcsCommand out;
  out.header.stamp = now();
  out.command = cmd.command;
  out.params = {cmd.param1, cmd.param2, cmd.param3, cmd.param4, cmd.param5, cmd.param6,
    cmd.param7};
  out.target_system = cmd.target_system;
  out.target_component = cmd.target_component;
  out.confirmation = cmd.confirmation;
  pub_command_->publish(out);

  // Acknowledges receipt/decode only — actual command execution and its
  // result belong to whichever node consumes /mc/gcs/command (SDD §11
  // "zero business logic" adapter philosophy, matching fcc_bridge_node).
  send_command_ack(from, cmd.command, MAV_RESULT_ACCEPTED);
}

void GcsMavlinkNode::on_heartbeat_timer()
{
  send_heartbeat();
  publish_link_status();
}

void GcsMavlinkNode::send_heartbeat()
{
  // Nowhere to send until mavlink-router has relayed at least one inbound
  // frame — its Channel B `GCS_MAVLINK_NODE` endpoint is a "Normal" UDP
  // endpoint on an OS-assigned ephemeral port (header comment,
  // last_gcs_endpoint_), not a fixed address we can address up front. This
  // self-resolves within ~1 s of the GCS coming online since it heartbeats
  // at 1 Hz too (ICD §4.3) and mavlink-router relays immediately.
  if (!has_gcs_endpoint_) {
    return;
  }

  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(
    system_id_, kComponentId, &msg, MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID,
    /*base_mode=*/ 0, /*custom_mode=*/ 0, MAV_STATE_ACTIVE);
  send_frame(msg, last_gcs_endpoint_);
}

void GcsMavlinkNode::send_command_ack(const sockaddr_in & to, uint16_t command, uint8_t result)
{
  mavlink_message_t msg;
  mavlink_msg_command_ack_pack(
    system_id_, kComponentId, &msg, command, result, /*progress=*/ 255, /*result_param2=*/ 0,
    /*target_system=*/ 255, /*target_component=*/ 0);
  send_frame(msg, to);
}

void GcsMavlinkNode::send_frame(const mavlink_message_t & msg, const sockaddr_in & to)
{
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  ::sendto(sock_fd_, buf, len, 0, reinterpret_cast<const sockaddr *>(&to), sizeof(to));
  ++tx_packet_count_;
}

void GcsMavlinkNode::publish_link_status()
{
  const double age_s = (now() - last_gcs_heartbeat_).seconds();
  const bool link_ok = has_gcs_heartbeat_ && age_s < gcs_link_timeout_s_;

  mc_msgs::msg::LinkStatus msg;
  msg.header.stamp = now();
  msg.gcs_link_ok = link_ok;
  msg.last_heartbeat_us =
    has_gcs_heartbeat_ ? static_cast<uint64_t>(last_gcs_heartbeat_.nanoseconds() / 1000) : 0U;
  // No RSSI available over a UDP/loopback transport (ICD §7.2 marks this
  // RSSI-derived value as "if available") — binary heuristic instead.
  msg.link_quality = link_ok ? 1.0F : 0.0F;
  msg.rx_packet_count = rx_packet_count_;
  msg.tx_packet_count = tx_packet_count_;
  msg.rx_drop_count = rx_drop_count_;
  pub_link_status_->publish(msg);
}

void GcsMavlinkNode::handle_mission_count(const mavlink_message_t & msg)
{
  mavlink_mission_count_t count_msg{};
  mavlink_msg_mission_count_decode(&msg, &count_msg);

  if (count_msg.target_system != system_id_ || count_msg.target_component != kComponentId ||
    count_msg.mission_type != kMavMissionTypeMc)
  {
    return;  // not an MC_MISSION_UPLOAD handshake for this drone (ICD §4.1/§4.6)
  }

  // A fresh MISSION_COUNT always (re)starts the upload, even mid-RECEIVING —
  // matches how GCS-side implementations retry a stalled handshake from
  // scratch rather than resuming (SDD §11.2 has no explicit resume state).
  mission_state_ = MissionUploadState::RECEIVING;
  mission_expected_count_ = count_msg.count;
  mission_items_received_ = 0;
  mission_gcs_sysid_ = msg.sysid;
  mission_gcs_compid_ = msg.compid;
  mission_upload_msg_ = mc_msgs::msg::MissionUpload{};
  mission_upload_msg_.upload_id = next_upload_id_++;
  mission_last_activity_ = now();

  RCLCPP_INFO(
    get_logger(), "mission upload %u: expecting %u items", mission_upload_msg_.upload_id,
    mission_expected_count_);

  if (mission_expected_count_ == 0) {
    // Degenerate empty upload — nothing to request, accept immediately.
    send_mission_ack(MAV_MISSION_ACCEPTED);
    mission_upload_msg_.header.stamp = now();
    pub_mission_upload_->publish(mission_upload_msg_);
    mission_state_ = MissionUploadState::IDLE;
    return;
  }

  send_mission_request_int(0);
}

void GcsMavlinkNode::handle_mission_item_int(const mavlink_message_t & msg)
{
  if (mission_state_ != MissionUploadState::RECEIVING) {
    return;
  }

  mavlink_mission_item_int_t item{};
  mavlink_msg_mission_item_int_decode(&msg, &item);

  if (item.target_system != system_id_ || item.target_component != kComponentId ||
    item.mission_type != kMavMissionTypeMc)
  {
    return;
  }

  // Stale/duplicate/out-of-order item (SDD §11.2 "validate seq ==
  // items_received") — ignored; the retry timer will re-request whatever
  // seq we actually still need.
  if (item.seq != mission_items_received_) {
    return;
  }

  switch (item.command) {
    case kCmdIngressWaypoint:
    case kCmdMusterPoint:
      decode_ingress_or_muster_item(item);
      break;
    case kCmdFormationOffset:
      decode_formation_offset_item(item);
      break;
    case kCmdSearchAreaVertex:
      decode_search_area_vertex_item(item);
      break;
    case kCmdSwarmPeerEntry:
      decode_swarm_peer_entry_item(item);
      break;
    default:
      RCLCPP_WARN(
        get_logger(), "mission upload %u: unrecognised item command %u at seq %u — skipped",
        mission_upload_msg_.upload_id, item.command, item.seq);
      break;
  }

  ++mission_items_received_;
  mission_last_activity_ = now();

  if (mission_items_received_ == mission_expected_count_) {
    send_mission_ack(MAV_MISSION_ACCEPTED);
    mission_upload_msg_.header.stamp = now();
    pub_mission_upload_->publish(mission_upload_msg_);
    RCLCPP_INFO(
      get_logger(), "mission upload %u: complete, %u items", mission_upload_msg_.upload_id,
      mission_items_received_);
    mission_state_ = MissionUploadState::IDLE;
  } else {
    send_mission_request_int(mission_items_received_);
  }
}

void GcsMavlinkNode::check_mission_upload_timers()
{
  if (mission_state_ != MissionUploadState::RECEIVING) {
    return;
  }

  if ((now() - mission_last_activity_).seconds() >= mission_upload_timeout_s_) {
    abort_mission_upload("timeout");
    return;
  }

  if ((now() - mission_last_request_sent_).seconds() >= mission_request_retry_s_) {
    send_mission_request_int(mission_items_received_);
  }
}

void GcsMavlinkNode::send_mission_request_int(uint16_t seq)
{
  if (!has_gcs_endpoint_) {
    return;  // nowhere to send yet — mirrors send_heartbeat's guard
  }
  mavlink_message_t msg;
  mavlink_msg_mission_request_int_pack(
    system_id_, kComponentId, &msg, mission_gcs_sysid_, mission_gcs_compid_, seq,
    kMavMissionTypeMc);
  send_frame(msg, last_gcs_endpoint_);
  mission_last_request_sent_ = now();
}

void GcsMavlinkNode::send_mission_ack(uint8_t result)
{
  mavlink_message_t msg;
  mavlink_msg_mission_ack_pack(
    system_id_, kComponentId, &msg, mission_gcs_sysid_, mission_gcs_compid_, result,
    kMavMissionTypeMc, mission_upload_msg_.upload_id);
  send_frame(msg, last_gcs_endpoint_);
}

void GcsMavlinkNode::abort_mission_upload(const char * reason)
{
  RCLCPP_WARN(
    get_logger(), "mission upload %u aborted (%s) after %u/%u items",
    mission_upload_msg_.upload_id, reason, mission_items_received_, mission_expected_count_);
  send_mission_ack(MAV_MISSION_ERROR);
  mission_state_ = MissionUploadState::IDLE;
}

void GcsMavlinkNode::decode_ingress_or_muster_item(const mavlink_mission_item_int_t & item)
{
  // ICD §4.6: x=lat×10^7, y=lon×10^7, z=alt(m). param1 (speed, ingress
  // waypoint only) has no home in mc_msgs/MissionUpload.ingress_waypoints
  // (geometry_msgs/PoseStamped carries no velocity field) — decoded but not
  // stored. No yaw is transmitted either; orientation is left identity.
  geometry_msgs::msg::PoseStamped wp;
  wp.header.stamp = now();
  wp.pose.position.x = static_cast<double>(item.x) / 1e7;  // lat, deg
  wp.pose.position.y = static_cast<double>(item.y) / 1e7;  // lon, deg
  wp.pose.position.z = static_cast<double>(item.z);        // alt, m
  wp.pose.orientation.w = 1.0;
  mission_upload_msg_.ingress_waypoints.push_back(wp);
}

void GcsMavlinkNode::decode_formation_offset_item(const mavlink_mission_item_int_t & item)
{
  // ICD §4.6 gap note: x/y are native int32 fields, scaled ×1000 (mm) for
  // the N/E offsets; z is native float, so D needs no scaling.
  mc_msgs::msg::FormationOffset offset;
  offset.drone_id = static_cast<uint8_t>(std::lround(item.param1));
  offset.n = static_cast<float>(item.x) / 1000.0F;
  offset.e = static_cast<float>(item.y) / 1000.0F;
  offset.d = item.z;
  mission_upload_msg_.formation_offsets.push_back(offset);
}

void GcsMavlinkNode::decode_search_area_vertex_item(const mavlink_mission_item_int_t & item)
{
  // vertex_index/total_vertices (param1/param2) place this vertex correctly
  // even if search-area items are interleaved with other item kinds in the
  // flat MISSION_ITEM_INT seq stream (ICD §4.6).
  const auto total_vertices = static_cast<size_t>(std::lround(item.param2));
  const auto vertex_index = static_cast<size_t>(std::lround(item.param1));

  if (total_vertices == 0 || vertex_index >= total_vertices) {
    RCLCPP_WARN(
      get_logger(), "mission upload %u: search area vertex index %zu/%zu out of range — skipped",
      mission_upload_msg_.upload_id, vertex_index, total_vertices);
    return;
  }
  if (mission_upload_msg_.search_polygon.size() != total_vertices) {
    mission_upload_msg_.search_polygon.resize(total_vertices);
  }

  geographic_msgs::msg::GeoPoint vertex;
  vertex.latitude = static_cast<double>(item.x) / 1e7;
  vertex.longitude = static_cast<double>(item.y) / 1e7;
  vertex.altitude = 0.0;  // not transmitted (ICD §4.6: `z` unused for this item)
  mission_upload_msg_.search_polygon[vertex_index] = vertex;
}

void GcsMavlinkNode::decode_swarm_peer_entry_item(const mavlink_mission_item_int_t & item)
{
  // ICD §4.6 gap note: only drone_id + IP octets 3/4 are transmitted;
  // octets 1-2 are the fixed 192.168.x.x subnet (ICD §11.1). Every other
  // mc_msgs/SwarmPeer field is populated later from live peer traffic
  // (SDD §5.4).
  mc_msgs::msg::SwarmPeer peer;
  peer.drone_id = static_cast<uint8_t>(std::lround(item.x));
  const auto octet3 = static_cast<uint8_t>(std::lround(item.y));
  const auto octet4 = static_cast<uint8_t>(std::lround(item.z));
  peer.ip_address = std::to_string(kSwarmPeerIpOctet1) + "." +
    std::to_string(kSwarmPeerIpOctet2) + "." + std::to_string(octet3) + "." +
    std::to_string(octet4);
  mission_upload_msg_.swarm_manifest.push_back(peer);
}

}  // namespace gcs_comms
