// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §11, ICD-MC-001 §4).

#include "gcs_comms/gcs_mavlink_node.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <mutex>
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

// GCS-003: MC_STATUS/MC_DETECTION/MC_HEALTH (ICD §4.6, SDD §11.3) are
// custom messages outside the common dialect ros-humble-mavlink ships
// generated headers for, so there is no mavlink_msg_mc_status_pack() etc.
// to call. Rather than adding a mavgen/pymavlink codegen step to the build
// (this file's own CMakeLists.txt already chose raw mavlink.h over MAVROS
// to keep the toolchain thin — same rationale applies here), these three
// are hand-packed using the same generic, dialect-independent helpers
// mavgen's own output would call: _mav_put_*/_MAV_RETURN_* (protocol.h) and
// mavlink_finalize_message() (mavlink_helpers.h), both already pulled in
// transitively by <mavlink/v2.0/common/mavlink.h>. The wire offsets and
// CRC_EXTRA constants below were derived from
// gcs_comms/mc_mavlink_defs.xml's field list (that file is the
// authoritative source — see its own header comment) by running
// pymavlink's field-sort (descending type size, stable on ties) and X25
// CRC-extra algorithm offline, then verified against an actual `mavgen`
// run of that exact file before being committed. If mc_mavlink_defs.xml's
// field list for these messages ever changes, these constants must be
// re-derived the same way — they do not update themselves.
constexpr uint32_t kMsgIdMcStatus = 50100;
constexpr uint8_t kLenMcStatus = 33;
constexpr uint8_t kCrcExtraMcStatus = 24;

constexpr uint32_t kMsgIdMcDetection = 50101;
constexpr uint8_t kLenMcDetection = 55;
constexpr uint8_t kCrcExtraMcDetection = 122;

constexpr uint32_t kMsgIdMcHealth = 50103;
constexpr uint8_t kLenMcHealth = 21;
constexpr uint8_t kCrcExtraMcHealth = 127;

void pack_mc_status(
  mavlink_message_t & msg, uint8_t sysid, uint8_t compid, uint64_t time_usec, int32_t lat,
  int32_t lon, int32_t alt_msl, uint16_t heading, uint16_t groundspeed, int16_t climb_rate,
  uint16_t fault_mask, uint8_t battery_remaining, uint8_t mission_phase,
  uint8_t localisation_mode, uint8_t peer_count, uint8_t isr_role)
{
  char buf[kLenMcStatus];
  _mav_put_uint64_t(buf, 0, time_usec);
  _mav_put_int32_t(buf, 8, lat);
  _mav_put_int32_t(buf, 12, lon);
  _mav_put_int32_t(buf, 16, alt_msl);
  _mav_put_uint16_t(buf, 20, heading);
  _mav_put_uint16_t(buf, 22, groundspeed);
  _mav_put_int16_t(buf, 24, climb_rate);
  _mav_put_uint16_t(buf, 26, fault_mask);
  _mav_put_uint8_t(buf, 28, battery_remaining);
  _mav_put_uint8_t(buf, 29, mission_phase);
  _mav_put_uint8_t(buf, 30, localisation_mode);
  _mav_put_uint8_t(buf, 31, peer_count);
  _mav_put_uint8_t(buf, 32, isr_role);
  std::memcpy(_MAV_PAYLOAD_NON_CONST(&msg), buf, kLenMcStatus);
  msg.msgid = kMsgIdMcStatus;
  mavlink_finalize_message(&msg, sysid, compid, kLenMcStatus, kLenMcStatus, kCrcExtraMcStatus);
}

void pack_mc_detection(
  mavlink_message_t & msg, uint8_t sysid, uint8_t compid, uint64_t time_usec,
  uint32_t detection_id, uint16_t class_id, uint16_t confidence, int32_t lat, int32_t lon,
  int32_t alt, uint16_t pos_uncertainty, uint8_t drone_id, const char (& class_name)[24])
{
  char buf[kLenMcDetection];
  _mav_put_uint64_t(buf, 0, time_usec);
  _mav_put_uint32_t(buf, 8, detection_id);
  _mav_put_int32_t(buf, 12, lat);
  _mav_put_int32_t(buf, 16, lon);
  _mav_put_int32_t(buf, 20, alt);
  _mav_put_uint16_t(buf, 24, class_id);
  _mav_put_uint16_t(buf, 26, confidence);
  _mav_put_uint16_t(buf, 28, pos_uncertainty);
  _mav_put_uint8_t(buf, 30, drone_id);
  _mav_put_char_array(buf, 31, class_name, 24);
  std::memcpy(_MAV_PAYLOAD_NON_CONST(&msg), buf, kLenMcDetection);
  msg.msgid = kMsgIdMcDetection;
  mavlink_finalize_message(
    &msg, sysid, compid, kLenMcDetection, kLenMcDetection, kCrcExtraMcDetection);
}

void pack_mc_health(
  mavlink_message_t & msg, uint8_t sysid, uint8_t compid, uint64_t time_usec, uint8_t cpu_load,
  uint8_t mem_used, uint8_t disk_used, uint8_t gpu_temp, uint8_t cpu_temp, uint16_t fault_mask,
  uint16_t node_alive_mask, uint32_t uptime_s)
{
  char buf[kLenMcHealth];
  _mav_put_uint64_t(buf, 0, time_usec);
  _mav_put_uint32_t(buf, 8, uptime_s);
  _mav_put_uint16_t(buf, 12, fault_mask);
  _mav_put_uint16_t(buf, 14, node_alive_mask);
  _mav_put_uint8_t(buf, 16, cpu_load);
  _mav_put_uint8_t(buf, 17, mem_used);
  _mav_put_uint8_t(buf, 18, disk_used);
  _mav_put_uint8_t(buf, 19, gpu_temp);
  _mav_put_uint8_t(buf, 20, cpu_temp);
  std::memcpy(_MAV_PAYLOAD_NON_CONST(&msg), buf, kLenMcHealth);
  msg.msgid = kMsgIdMcHealth;
  mavlink_finalize_message(&msg, sysid, compid, kLenMcHealth, kLenMcHealth, kCrcExtraMcHealth);
}

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

  // GCS-003 (SDD §11.3): own callback groups, distinct from timer_cb_group_
  // above, so detection-heavy traffic on mux_sub_cb_group_ can never queue
  // behind (or be queued behind by) GCS-001's command-decode path on
  // recv_timer_.
  mux_sub_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions mux_sub_opts;
  mux_sub_opts.callback_group = mux_sub_cb_group_;

  sub_vehicle_state_ = create_subscription<mc_msgs::msg::VehicleState>(
    "/mc/fcc/vehicle_state", rclcpp::QoS(10),
    [this](const mc_msgs::msg::VehicleState::SharedPtr msg) {on_vehicle_state(msg);},
    mux_sub_opts);
  sub_mission_phase_ = create_subscription<mc_msgs::msg::MissionPhase>(
    "/mc/mission_phase", rclcpp::QoS(10),
    [this](const mc_msgs::msg::MissionPhase::SharedPtr msg) {on_mission_phase(msg);},
    mux_sub_opts);
  sub_localisation_mode_ = create_subscription<mc_msgs::msg::LocalisationMode>(
    "/mc/localisation_mode", rclcpp::QoS(10),
    [this](const mc_msgs::msg::LocalisationMode::SharedPtr msg) {on_localisation_mode(msg);},
    mux_sub_opts);
  sub_health_ = create_subscription<mc_msgs::msg::SystemHealth>(
    "/mc/health", rclcpp::QoS(10),
    [this](const mc_msgs::msg::SystemHealth::SharedPtr msg) {on_health(msg);},
    mux_sub_opts);
  sub_detections_ = create_subscription<mc_msgs::msg::DetectionArray>(
    "/mc/detections", rclcpp::QoS(10),
    [this](const mc_msgs::msg::DetectionArray::SharedPtr msg) {on_detections(msg);},
    mux_sub_opts);

  mux_timer_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  mc_status_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    [this] {on_mc_status_timer();},
    mux_timer_cb_group_);

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
  // known source (header comment, last_gcs_endpoint_). Locked: GCS-003's
  // mux_sub_cb_group_/mux_timer_cb_group_ read these fields concurrently
  // with this write (send_mutex_ header comment).
  {
    std::lock_guard<std::mutex> lock(send_mutex_);
    last_gcs_endpoint_ = from;
    has_gcs_endpoint_ = true;
  }

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
  std::lock_guard<std::mutex> lock(send_mutex_);

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
  std::lock_guard<std::mutex> lock(send_mutex_);
  mavlink_message_t msg;
  mavlink_msg_command_ack_pack(
    system_id_, kComponentId, &msg, command, result, /*progress=*/ 255, /*result_param2=*/ 0,
    /*target_system=*/ 255, /*target_component=*/ 0);
  send_frame(msg, to);
}

// Callers must already hold send_mutex_ — this only serializes the actual
// syscall + counter update, not the has_gcs_endpoint_ check or the
// pack/finalize call that precedes it (both also need the lock, so every
// caller takes it once, up front, for its whole body — see send_mutex_'s
// header comment).
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
  std::lock_guard<std::mutex> lock(send_mutex_);
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
  std::lock_guard<std::mutex> lock(send_mutex_);
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

// --- GCS-003: telemetry multiplexer (SDD §11.3, ICD §4.6) ---

void GcsMavlinkNode::on_vehicle_state(const mc_msgs::msg::VehicleState::SharedPtr msg)
{
  latest_vehicle_state_ = *msg;
  has_vehicle_state_ = true;
}

void GcsMavlinkNode::on_mission_phase(const mc_msgs::msg::MissionPhase::SharedPtr msg)
{
  latest_mission_phase_ = msg->phase;
}

void GcsMavlinkNode::on_localisation_mode(const mc_msgs::msg::LocalisationMode::SharedPtr msg)
{
  latest_localisation_mode_ = msg->mode;
}

void GcsMavlinkNode::on_health(const mc_msgs::msg::SystemHealth::SharedPtr msg)
{
  // Cached for the next MC_STATUS tick (send_mc_status()); the full message
  // itself is relayed as its own MC_HEALTH right now, not deferred to that
  // tick (SDD §11.3: "health updates are event-driven").
  latest_fault_mask_ = msg->fault_mask;
  send_mc_health(*msg);
}

void GcsMavlinkNode::on_detections(const mc_msgs::msg::DetectionArray::SharedPtr msg)
{
  // ICD §4.6 MC_DETECTION is a single-target message; DetectionArray may
  // carry several detections from one frame — one MC_DETECTION per element.
  for (const auto & detection : msg->detections) {
    send_mc_detection(detection);
  }
}

void GcsMavlinkNode::on_mc_status_timer()
{
  if (!has_vehicle_state_) {
    return;  // nothing meaningful to report before fcc_bridge_node's first publish
  }
  send_mc_status();
}

void GcsMavlinkNode::send_mc_status()
{
  std::lock_guard<std::mutex> lock(send_mutex_);
  if (!has_gcs_endpoint_) {
    return;
  }

  const auto & vs = latest_vehicle_state_;

  // ICD §4.6: heading is cdeg in [0, 36000) — VehicleState.heading is rad
  // from North and may be negative, so convert to degrees and wrap.
  double heading_deg = std::fmod(vs.heading * 180.0 / M_PI, 360.0);
  if (heading_deg < 0.0) {
    heading_deg += 360.0;
  }

  mavlink_message_t msg;
  pack_mc_status(
    msg, system_id_, kComponentId, static_cast<uint64_t>(now().nanoseconds() / 1000),
    static_cast<int32_t>(std::lround(vs.lat * 1e7)),
    static_cast<int32_t>(std::lround(vs.lon * 1e7)),
    static_cast<int32_t>(std::lround(vs.alt_msl * 1000.0)),
    static_cast<uint16_t>(std::lround(heading_deg * 100.0)),
    static_cast<uint16_t>(std::lround(vs.groundspeed * 100.0)),
    // Climb rate is positive-up (ICD §4.6); VehicleState.vz is NED down.
    static_cast<int16_t>(std::lround(-vs.vz * 100.0)),
    latest_fault_mask_, static_cast<uint8_t>(std::lround(vs.battery_pct * 100.0)),
    latest_mission_phase_, latest_localisation_mode_,
    // peer_count/isr_role: no swarm-state topic is in this ticket's scope
    // (GCS-003 AC lists only vehicle_state/mission_phase/detections/
    // health/localisation_mode) — left at 0 pending a future ticket wiring
    // /mc/swarm/peer_states and /mc/swarm/isr_role in here too.
    /*peer_count=*/ 0, /*isr_role=*/ 0);
  send_frame(msg, last_gcs_endpoint_);
}

void GcsMavlinkNode::send_mc_detection(const mc_msgs::msg::Detection & detection)
{
  std::lock_guard<std::mutex> lock(send_mutex_);
  if (!has_gcs_endpoint_) {
    return;
  }

  char class_name[24] = {};
  // ICD §4.6: char[24], null-terminated — truncate rather than overflow if
  // a longer class label ever reaches perception_down_node/
  // perception_front_node's class registry.
  std::strncpy(class_name, detection.class_name.c_str(), sizeof(class_name) - 1);

  mavlink_message_t msg;
  pack_mc_detection(
    msg, system_id_, kComponentId, detection.frame_timestamp_us, detection.detection_id,
    detection.class_id, static_cast<uint16_t>(std::lround(detection.confidence * 10000.0)),
    static_cast<int32_t>(std::lround(detection.lat * 1e7)),
    static_cast<int32_t>(std::lround(detection.lon * 1e7)),
    static_cast<int32_t>(std::lround(detection.alt_msl * 1000.0)),
    static_cast<uint16_t>(std::lround(detection.pos_uncertainty_m * 100.0)),
    // This drone's own ID — /mc/detections is this MC's own perception
    // output (ICD §4.6 "Reporting drone ID").
    system_id_, class_name);
  send_frame(msg, last_gcs_endpoint_);
}

void GcsMavlinkNode::send_mc_health(const mc_msgs::msg::SystemHealth & health)
{
  std::lock_guard<std::mutex> lock(send_mutex_);
  if (!has_gcs_endpoint_) {
    return;
  }

  mavlink_message_t msg;
  pack_mc_health(
    msg, system_id_, kComponentId, static_cast<uint64_t>(now().nanoseconds() / 1000),
    health.cpu_pct, health.mem_pct, health.disk_pct, health.gpu_temp_c, health.cpu_temp_c,
    health.fault_mask, health.node_alive_mask, health.uptime_s);
  send_frame(msg, last_gcs_endpoint_);
}

}  // namespace gcs_comms
