// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §11, ICD-MC-001 §12).

#include "gcs_comms/mission_orchestrator_node.hpp"

#include <arpa/inet.h>
#include <dirent.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cctype>
#include <cerrno>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "geographic_msgs/msg/geo_point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mc_msgs/msg/formation_offset.hpp"
#include "mc_msgs/msg/swarm_peer.hpp"
#include <nlohmann/json.hpp>

namespace gcs_comms
{

namespace
{
constexpr char kDefaultPlansDir[] = "/var/drone-mc/mission_plans";  // ICD §12.4

// mc_mavlink_defs.xml (ICD §12.2): custom messages this node hand-packs/
// hand-decodes, same "no mavgen build step" rationale as gcs_mavlink_node.cpp
// — see that file's own comment and mc_mavlink_defs.xml's header comment for
// how these constants were derived (an actual `mavgen` run against that
// file, cross-checked, not hand math).
constexpr uint32_t kMsgIdMcMissionPlanActivate = 50104;
constexpr uint8_t kLenMcMissionPlanActivate = 68;
constexpr uint8_t kCrcExtraMcMissionPlanActivate = 243;

constexpr uint32_t kMsgIdMcMissionPlanAck = 50105;
constexpr uint8_t kLenMcMissionPlanAck = 119;
constexpr uint8_t kCrcExtraMcMissionPlanAck = 185;

// Wire offsets within MC_MISSION_PLAN_ACTIVATE's 68-byte payload (field-sort:
// descending type size, stable on ties — client_request_id(u32) before
// filename(char[64])).
constexpr size_t kActivateOffsetClientRequestId = 0;
constexpr size_t kActivateOffsetFilename = 4;
constexpr size_t kActivateFilenameLen = 64;

std::string env_or(const char * name, const std::string & fallback)
{
  const char * v = std::getenv(name);
  return v != nullptr ? std::string(v) : fallback;
}

int open_loopback_socket(uint16_t port)
{
  const int fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
  if (fd < 0) {
    throw std::runtime_error("MissionOrchestratorNode: failed to create Channel C UDP socket");
  }
  constexpr int kReuseAddr = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &kReuseAddr, sizeof(kReuseAddr));

  sockaddr_in local_addr{};
  local_addr.sin_family = AF_INET;
  local_addr.sin_port = htons(port);
  // Loopback only — this is the `ORCHESTRATOR_MAVLINK_NODE` endpoint of
  // mavlink-router-channel-c.conf (ICD §12.1), never reachable off-box.
  local_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (::bind(fd, reinterpret_cast<const sockaddr *>(&local_addr), sizeof(local_addr)) != 0) {
    ::close(fd);
    throw std::runtime_error(
            "MissionOrchestratorNode: failed to bind Channel C UDP socket to port " +
            std::to_string(port));
  }
  return fd;
}

// Validates a bare "<name>.json" path component (ICD §12.3: "alphanumeric +
// _/-, <= 32 characters") — used both for MAVFTP virtual paths (after
// stripping the leading '/') and for MC_MISSION_PLAN_ACTIVATE's `filename`
// field, which carries the same shape without the leading '/'. Rejecting
// every character outside [A-Za-z0-9_-] also rejects '/' and '.', which is
// what actually blocks path traversal here — there is no separate ".."
// check because ".." itself contains no allowed character.
bool validate_plan_filename(const std::string & component)
{
  constexpr char kSuffix[] = ".json";
  constexpr size_t kSuffixLen = 5;
  constexpr size_t kMaxNameLen = 32;

  if (component.size() <= kSuffixLen || component.size() > kMaxNameLen + kSuffixLen) {
    return false;
  }
  if (component.compare(component.size() - kSuffixLen, kSuffixLen, kSuffix) != 0) {
    return false;
  }
  const std::string name = component.substr(0, component.size() - kSuffixLen);
  for (const char c : name) {
    if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-')) {
      return false;
    }
  }
  return true;
}

}  // namespace

MissionOrchestratorNode::MissionOrchestratorNode(const rclcpp::NodeOptions & options)
: Node("mission_orchestrator_node", options),
  plans_dir_(declare_parameter("plans_dir", std::string(kDefaultPlansDir)))
{
  system_id_ = static_cast<uint8_t>(std::stoi(env_or("DRONE_ID", "1")));
  const uint16_t loopback_port = static_cast<uint16_t>(
    std::stoi(env_or("ORCHESTRATOR_MAVLINK_LOOPBACK_PORT", "14580")));
  sock_fd_ = open_loopback_socket(loopback_port);

  if (::mkdir(plans_dir_.c_str(), 0755) != 0 && errno != EEXIST) {
    ::close(sock_fd_);
    throw std::runtime_error(
            "MissionOrchestratorNode: failed to create plans directory " + plans_dir_);
  }

  pub_mission_upload_ =
    create_publisher<mc_msgs::msg::MissionUpload>("/mc/gcs/mission_upload", rclcpp::QoS(10));

  timer_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  recv_timer_ = create_wall_timer(
    std::chrono::milliseconds(10),
    [this] {on_recv_timer();},
    timer_cb_group_);

  phase_sub_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions phase_sub_opts;
  phase_sub_opts.callback_group = phase_sub_cb_group_;
  sub_mission_phase_ = create_subscription<mc_msgs::msg::MissionPhase>(
    "/mc/mission_phase", rclcpp::QoS(10),
    [this](const mc_msgs::msg::MissionPhase::SharedPtr msg) {on_mission_phase(msg);},
    phase_sub_opts);

  RCLCPP_INFO(
    get_logger(),
    "mission_orchestrator_node started: system_id=%u, Channel C loopback port=%u, plans_dir=%s",
    system_id_, loopback_port, plans_dir_.c_str());
}

MissionOrchestratorNode::~MissionOrchestratorNode()
{
  if (session_fd_ >= 0) {
    ::close(session_fd_);
  }
  if (sock_fd_ >= 0) {
    ::close(sock_fd_);
  }
}

// --- Channel C raw-frame transport (see mission_orchestrator_node.hpp's
// RawFrame comment for why this bypasses mavlink_parse_char()) ---

void MissionOrchestratorNode::on_recv_timer()
{
  uint8_t buf[512];
  sockaddr_in from{};
  socklen_t from_len = sizeof(from);

  while (true) {
    const ssize_t n = ::recvfrom(
      sock_fd_, buf, sizeof(buf), 0, reinterpret_cast<sockaddr *>(&from), &from_len);
    if (n <= 0) {
      break;  // EAGAIN/EWOULDBLOCK — nothing more queued this tick
    }

    RawFrame frame;
    if (decode_raw_v2_frame(buf, n, frame)) {
      handle_frame(frame, from);
    }
  }
}

bool MissionOrchestratorNode::decode_raw_v2_frame(const uint8_t * buf, ssize_t n, RawFrame & out)
{
  constexpr ssize_t kHeaderLen = 10;  // v2 core header: magic..msgid (MAVLINK_CORE_HEADER_LEN+1)
  constexpr ssize_t kCrcLen = 2;
  if (n < kHeaderLen + kCrcLen || buf[0] != MAVLINK_STX) {
    return false;
  }

  const uint8_t incompat_flags = buf[2];
  if (incompat_flags & MAVLINK_IFLAG_SIGNED) {
    // MAVLink 2 signing is flagged as required before this ships, not
    // designed here (interface spec §3.4, ICD §12.1 open item) — a signed
    // frame can't be verified yet, so it's dropped rather than trusted.
    return false;
  }

  const uint8_t len = buf[1];
  if (n < kHeaderLen + len + kCrcLen) {
    return false;
  }

  const uint32_t msgid = static_cast<uint32_t>(buf[7]) | (static_cast<uint32_t>(buf[8]) << 8) |
    (static_cast<uint32_t>(buf[9]) << 16);

  uint8_t crc_extra = 0;
  switch (msgid) {
    case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL:
      crc_extra = MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_CRC;
      break;
    case kMsgIdMcMissionPlanActivate:
      crc_extra = kCrcExtraMcMissionPlanActivate;
      break;
    default:
      return false;  // this node never needs to receive anything else on Channel C
  }

  // Same X25 scheme mavlink_finalize_message() uses on the send side
  // (mavlink_helpers.h: crc over header[1..] + payload, then crc_extra),
  // replicated here because mavlink_parse_char() cannot look up crc_extra
  // for a msgid outside the compiled-in common dialect.
  uint16_t crc;
  crc_init(&crc);
  for (ssize_t i = 1; i < kHeaderLen + len; ++i) {
    crc_accumulate(buf[i], &crc);
  }
  crc_accumulate(crc_extra, &crc);
  const uint16_t received_crc = static_cast<uint16_t>(buf[kHeaderLen + len]) |
    (static_cast<uint16_t>(buf[kHeaderLen + len + 1]) << 8);
  if (crc != received_crc) {
    return false;
  }

  out.msgid = msgid;
  out.sysid = buf[5];
  out.compid = buf[6];
  out.payload.fill(0);
  std::memcpy(out.payload.data(), buf + kHeaderLen, std::min<size_t>(len, out.payload.size()));
  return true;
}

void MissionOrchestratorNode::handle_frame(const RawFrame & frame, const sockaddr_in & from)
{
  last_orchestrator_endpoint_ = from;
  has_orchestrator_endpoint_ = true;

  switch (frame.msgid) {
    case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL:
      handle_ftp_request(frame, from);
      break;
    case kMsgIdMcMissionPlanActivate:
      handle_activate_request(frame, from);
      break;
    default:
      break;
  }
}

void MissionOrchestratorNode::send_frame(const mavlink_message_t & msg, const sockaddr_in & to)
{
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  ::sendto(sock_fd_, buf, len, 0, reinterpret_cast<const sockaddr *>(&to), sizeof(to));
}

void MissionOrchestratorNode::send_statustext(uint8_t severity, const std::string & text)
{
  std::lock_guard<std::mutex> lock(send_mutex_);
  if (!has_orchestrator_endpoint_) {
    return;
  }
  const std::string truncated = text.substr(0, 50);  // STATUSTEXT.text is char[50] (ICD §4.7)
  mavlink_message_t msg;
  mavlink_msg_statustext_pack(
    system_id_, kComponentId, &msg, severity, truncated.c_str(), /*id=*/ 0, /*chunk_seq=*/ 0);
  send_frame(msg, last_orchestrator_endpoint_);
}

void MissionOrchestratorNode::send_mission_plan_ack(
  const sockaddr_in & to, uint32_t client_request_id, const std::string & filename,
  uint8_t result, const std::string & error_message)
{
  char filename_buf[kActivateFilenameLen] = {};
  std::strncpy(filename_buf, filename.c_str(), sizeof(filename_buf) - 1);
  char error_buf[50] = {};
  std::strncpy(error_buf, error_message.c_str(), sizeof(error_buf) - 1);

  char buf[kLenMcMissionPlanAck];
  _mav_put_uint32_t(buf, 0, client_request_id);
  _mav_put_char_array(buf, 4, filename_buf, kActivateFilenameLen);
  _mav_put_uint8_t(buf, 68, result);
  _mav_put_char_array(buf, 69, error_buf, 50);

  std::lock_guard<std::mutex> lock(send_mutex_);
  mavlink_message_t msg;
  std::memcpy(_MAV_PAYLOAD_NON_CONST(&msg), buf, kLenMcMissionPlanAck);
  msg.msgid = kMsgIdMcMissionPlanAck;
  mavlink_finalize_message(
    &msg, system_id_, kComponentId, kLenMcMissionPlanAck, kLenMcMissionPlanAck,
    kCrcExtraMcMissionPlanAck);
  send_frame(msg, to);
}

// --- MAVFTP server (ICD §12.3, adapted from PX4's mavlink_ftp.cpp — single
// session, flat namespace, /var/drone-mc/mission_plans/ root) ---

void MissionOrchestratorNode::handle_ftp_request(const RawFrame & frame, const sockaddr_in & from)
{
  // FILE_TRANSFER_PROTOCOL's own target_system/target_component (payload
  // bytes [1]/[2], ahead of the 12-byte FtpHeader at byte [3]) — 0 means
  // broadcast, matching PX4's own handle_message() check.
  const uint8_t target_system = frame.payload[1];
  const uint8_t target_component = frame.payload[2];
  if ((target_system != system_id_ && target_system != 0) ||
    (target_component != kComponentId && target_component != 0))
  {
    return;
  }

  FtpHeader hdr{};
  std::memcpy(&hdr, frame.payload.data() + 3, sizeof(hdr));
  const uint8_t * data = frame.payload.data() + 3 + sizeof(hdr);

  if (hdr.opcode == kFtpOpBurstReadFile) {
    if (session_mode_ != SessionMode::kRead && session_mode_ != SessionMode::kActiveRead) {
      const uint8_t err = kFtpErrInvalidSession;
      send_ftp_reply(
        hdr.seq_number + 1, kFtpOpNak, hdr.opcode, 1, hdr.offset, &err, 1, frame.sysid,
        frame.compid, from);
      return;
    }
    handle_burst_read(hdr.seq_number, frame.sysid, frame.compid, hdr.offset, from);
    return;
  }

  uint8_t out_data[kFtpMaxData] = {};
  uint8_t out_size = 0;
  const FtpError err = ftp_dispatch(hdr, data, out_size, out_data);

  if (err == kFtpErrNone) {
    send_ftp_reply(
      hdr.seq_number + 1, kFtpOpAck, hdr.opcode, 0, hdr.offset, out_data, out_size, frame.sysid,
      frame.compid, from);
  } else {
    const uint8_t err_byte = static_cast<uint8_t>(err);
    send_ftp_reply(
      hdr.seq_number + 1, kFtpOpNak, hdr.opcode, 0, hdr.offset, &err_byte, 1, frame.sysid,
      frame.compid, from);
  }
}

void MissionOrchestratorNode::send_ftp_reply(
  uint16_t seq, uint8_t opcode, uint8_t req_opcode, uint8_t burst_complete, uint32_t offset,
  const uint8_t * data, uint8_t size, uint8_t target_sysid, uint8_t target_compid,
  const sockaddr_in & to)
{
  uint8_t ftp_payload[251] = {};
  FtpHeader hdr{};
  hdr.seq_number = seq;
  hdr.session = 0;  // single-session model — there is only ever session 0
  hdr.opcode = opcode;
  hdr.size = size;
  hdr.req_opcode = req_opcode;
  hdr.burst_complete = burst_complete;
  hdr.padding = 0;
  hdr.offset = offset;
  std::memcpy(ftp_payload, &hdr, sizeof(hdr));
  if (data != nullptr && size > 0) {
    std::memcpy(ftp_payload + sizeof(hdr), data, size);
  }

  std::lock_guard<std::mutex> lock(send_mutex_);
  mavlink_message_t msg;
  mavlink_msg_file_transfer_protocol_pack(
    system_id_, kComponentId, &msg, /*target_network=*/ 0, target_sysid, target_compid,
    ftp_payload);
  send_frame(msg, to);
}

void MissionOrchestratorNode::handle_burst_read(
  uint16_t seq, uint8_t sysid, uint8_t compid, uint32_t offset, const sockaddr_in & to)
{
  // Small JSON plan files (at most a few KB) fit well within one recv-timer
  // tick's worth of synchronous sends — unlike PX4's own tick-driven
  // send()/get_free_tx_buf() loop (needed there for arbitrary-size log/param
  // files over a bandwidth-limited radio), there is no benefit to spreading
  // this across multiple timer ticks here. Every data-bearing Ack carries
  // burst_complete=0; EOF is always its own terminating Nak, matching PX4's
  // real behaviour for files under its own 35000-byte chunk-transmitted cap
  // (which every plan file here is always under).
  uint16_t reply_seq = static_cast<uint16_t>(seq + 1);
  for (;; ) {
    uint8_t chunk[kFtpMaxData];
    uint8_t chunk_size = 0;
    const FtpError err = ftp_read(offset, kFtpMaxData, chunk_size, chunk);
    if (err != kFtpErrNone) {
      const uint8_t err_byte = static_cast<uint8_t>(err);
      send_ftp_reply(
        reply_seq, kFtpOpNak, kFtpOpBurstReadFile, 1, offset, &err_byte, 1, sysid, compid, to);
      return;
    }
    send_ftp_reply(
      reply_seq, kFtpOpAck, kFtpOpBurstReadFile, 0, offset, chunk, chunk_size, sysid, compid, to);
    offset += chunk_size;
    ++reply_seq;
  }
}

MissionOrchestratorNode::FtpError MissionOrchestratorNode::ftp_dispatch(
  const FtpHeader & hdr, const uint8_t * data, uint8_t & out_size, uint8_t * out_data)
{
  const uint8_t data_len = std::min<uint8_t>(hdr.size, kFtpMaxData);
  out_size = 0;

  switch (hdr.opcode) {
    case kFtpOpNone:
      return kFtpErrNone;
    case kFtpOpResetSessions:
      return ftp_reset();
    case kFtpOpTerminateSession:
      if (hdr.session != 0) {
        return kFtpErrInvalidSession;
      }
      return ftp_terminate();
    case kFtpOpListDirectory:
      return ftp_list_directory(hdr.offset, out_size, out_data);
    case kFtpOpOpenFileRO:
      return ftp_open(
        std::string(reinterpret_cast<const char *>(data), data_len), false, false, out_size,
        out_data);
    case kFtpOpOpenFileWO:
      return ftp_open(
        std::string(reinterpret_cast<const char *>(data), data_len), true, false, out_size,
        out_data);
    case kFtpOpCreateFile:
      return ftp_open(
        std::string(reinterpret_cast<const char *>(data), data_len), true, true, out_size,
        out_data);
    case kFtpOpReadFile:
      if (hdr.session != 0) {
        return kFtpErrInvalidSession;
      }
      return ftp_read(hdr.offset, hdr.size, out_size, out_data);
    case kFtpOpWriteFile:
      if (hdr.session != 0) {
        return kFtpErrInvalidSession;
      }
      return ftp_write(hdr.offset, data, data_len, out_size, out_data);
    case kFtpOpRemoveFile:
      return ftp_remove(std::string(reinterpret_cast<const char *>(data), data_len));
    default:
      return kFtpErrUnknownCommand;
  }
}

MissionOrchestratorNode::FtpError MissionOrchestratorNode::ftp_list_directory(
  uint32_t entry_offset, uint8_t & out_size, uint8_t * out_data)
{
  DIR * dp = ::opendir(plans_dir_.c_str());
  if (dp == nullptr) {
    return kFtpErrFailErrno;
  }

  // ICD §12.3: entries are paginated by count, not by byte — the client
  // (pymavlink's mavftp.py, same as PX4's own real clients) re-issues
  // ListDirectory with `offset` set to however many entries it has already
  // consumed, so this "offset" skips that many directory entries first.
  //
  // Must apply the exact same DT_REG filter as the listing loop below —
  // TEST-004 found (via a real ListDirectory round-trip against a
  // single-plan directory, not just by inspection) that skipping raw
  // readdir() entries here undercounts "." and ".." against the client's
  // own already-consumed-real-entries count, so page 2 of a listing
  // re-returns entries page 1 already returned instead of advancing.
  uint32_t skip = entry_offset;
  while (skip > 0) {
    struct dirent * entry = ::readdir(dp);
    if (entry == nullptr) {
      break;
    }
    if (entry->d_type != DT_REG) {
      continue;
    }
    --skip;
  }

  uint8_t offset = 0;
  for (;; ) {
    errno = 0;
    struct dirent * entry = ::readdir(dp);
    if (entry == nullptr) {
      break;
    }
    if (entry->d_type != DT_REG) {
      continue;  // flat namespace: skip "." / ".." / anything not a regular file
    }

    const std::string full = plans_dir_ + "/" + entry->d_name;
    struct stat st {};
    if (::stat(full.c_str(), &st) != 0) {
      continue;
    }

    char line[288];
    const int written = std::snprintf(
      line, sizeof(line), "F%s\t%" PRId64, entry->d_name, static_cast<int64_t>(st.st_size));
    if (written <= 0) {
      continue;
    }
    const size_t name_len = static_cast<size_t>(written);
    if (offset + name_len + 1 > kFtpMaxData) {
      break;  // no room for this entry — client will re-request at the new offset
    }
    std::memcpy(out_data + offset, line, name_len);
    out_data[offset + name_len] = '\0';
    offset = static_cast<uint8_t>(offset + name_len + 1);
  }
  ::closedir(dp);

  // Matches PX4's _workList: no entries in this reply (whether the
  // directory is genuinely empty or the client has paged past the last
  // entry) signals EOF rather than an empty-but-successful Ack.
  if (offset == 0) {
    return kFtpErrEOF;
  }
  out_size = offset;
  return kFtpErrNone;
}

bool MissionOrchestratorNode::resolve_plan_path(
  const std::string & virtual_path, std::string & out_full_path, std::string & out_filename) const
{
  if (virtual_path.empty() || virtual_path.front() != '/') {
    return false;
  }
  const std::string component = virtual_path.substr(1);
  if (!validate_plan_filename(component)) {
    return false;
  }
  out_filename = component;
  out_full_path = plans_dir_ + "/" + component;
  return true;
}

MissionOrchestratorNode::FtpError MissionOrchestratorNode::ftp_open(
  const std::string & virtual_path, bool for_write, bool create_only, uint8_t & out_size,
  uint8_t * out_data)
{
  if (session_mode_ != SessionMode::kNone) {
    return kFtpErrNoSessionsAvailable;  // ICD §12.3: single MAVFTP session at a time
  }

  if (virtual_path == "/_active") {
    if (for_write) {
      // "_active" is a reserved, read-only synthetic entry (ICD §12.3) —
      // there is no standard MAVFTP error code for "this name is reserved",
      // so FileProtected (the same code delete-protection on the active
      // plan already uses) is the closest fit.
      return kFtpErrFileProtected;
    }
    active_read_buffer_.assign(active_plan_filename_.begin(), active_plan_filename_.end());
    session_mode_ = SessionMode::kActiveRead;
    const uint32_t size32 = static_cast<uint32_t>(active_read_buffer_.size());
    std::memcpy(out_data, &size32, sizeof(size32));
    out_size = sizeof(size32);
    return kFtpErrNone;
  }

  std::string filename, full_path;
  if (!resolve_plan_path(virtual_path, full_path, filename)) {
    return kFtpErrFileProtected;
  }

  struct stat st {};
  const bool exists = ::stat(full_path.c_str(), &st) == 0;
  if (!for_write && !exists) {
    return kFtpErrFileNotFound;
  }
  if (for_write && !create_only && !exists) {
    return kFtpErrFileNotFound;  // OpenFileWO ("update") requires the file to already exist
  }

  const int flags = for_write ? (create_only ? (O_CREAT | O_EXCL | O_WRONLY) : O_WRONLY) : O_RDONLY;
  const int fd = ::open(full_path.c_str(), flags, 0644);
  if (fd < 0) {
    return (errno == EEXIST) ? kFtpErrFileExists : kFtpErrFailErrno;
  }

  session_mode_ = for_write ? SessionMode::kWrite : SessionMode::kRead;
  session_fd_ = fd;
  session_filename_ = filename;
  session_write_high_water_ = 0;

  const uint32_t size32 = exists ? static_cast<uint32_t>(st.st_size) : 0U;
  std::memcpy(out_data, &size32, sizeof(size32));
  out_size = sizeof(size32);
  return kFtpErrNone;
}

MissionOrchestratorNode::FtpError MissionOrchestratorNode::ftp_read(
  uint32_t offset, uint8_t size, uint8_t & out_size, uint8_t * out_data)
{
  const size_t want = std::min<size_t>(size, kFtpMaxData);

  if (session_mode_ == SessionMode::kActiveRead) {
    if (offset >= active_read_buffer_.size()) {
      return kFtpErrEOF;
    }
    const size_t n = std::min(want, active_read_buffer_.size() - offset);
    std::memcpy(out_data, active_read_buffer_.data() + offset, n);
    out_size = static_cast<uint8_t>(n);
    return kFtpErrNone;
  }

  if (session_mode_ != SessionMode::kRead || session_fd_ < 0) {
    return kFtpErrInvalidSession;
  }

  struct stat st {};
  if (::fstat(session_fd_, &st) != 0) {
    return kFtpErrFailErrno;
  }
  if (offset >= static_cast<uint64_t>(st.st_size)) {
    return kFtpErrEOF;  // must be checked ourselves — lseek() alone allows seeking past EOF
  }
  if (::lseek(session_fd_, offset, SEEK_SET) < 0) {
    return kFtpErrFailErrno;
  }
  const ssize_t n = ::read(session_fd_, out_data, want);
  if (n < 0) {
    return kFtpErrFailErrno;
  }
  out_size = static_cast<uint8_t>(n);
  return kFtpErrNone;
}

MissionOrchestratorNode::FtpError MissionOrchestratorNode::ftp_write(
  uint32_t offset, const uint8_t * data, uint8_t size, uint8_t & out_size, uint8_t * out_data)
{
  if (session_mode_ != SessionMode::kWrite || session_fd_ < 0) {
    return kFtpErrInvalidSession;
  }
  if (::lseek(session_fd_, offset, SEEK_SET) < 0) {
    return kFtpErrFailErrno;
  }
  const ssize_t written = ::write(session_fd_, data, size);
  if (written < 0) {
    return kFtpErrFailErrno;
  }
  session_write_high_water_ =
    std::max(session_write_high_water_, offset + static_cast<uint32_t>(written));

  const uint32_t written32 = static_cast<uint32_t>(written);
  std::memcpy(out_data, &written32, sizeof(written32));
  out_size = sizeof(written32);
  return kFtpErrNone;
}

MissionOrchestratorNode::FtpError MissionOrchestratorNode::ftp_remove(
  const std::string & virtual_path)
{
  std::string filename, full_path;
  if (!resolve_plan_path(virtual_path, full_path, filename)) {
    return kFtpErrFileProtected;
  }
  if (filename == active_plan_filename_) {
    return kFtpErrFileProtected;  // ICD §12.6: RemoveFile on the active plan -> Nak(FileProtected)
  }
  if (::unlink(full_path.c_str()) != 0) {
    return (errno == ENOENT) ? kFtpErrFileNotFound : kFtpErrFailErrno;
  }
  return kFtpErrNone;
}

MissionOrchestratorNode::FtpError MissionOrchestratorNode::ftp_terminate()
{
  if (session_mode_ == SessionMode::kNone) {
    return kFtpErrInvalidSession;
  }
  if (session_mode_ == SessionMode::kWrite) {
    finish_write_session();
  } else {
    close_session_fd_only();
  }
  return kFtpErrNone;
}

MissionOrchestratorNode::FtpError MissionOrchestratorNode::ftp_reset()
{
  if (session_mode_ != SessionMode::kNone) {
    close_session_fd_only();
  }
  return kFtpErrNone;
}

void MissionOrchestratorNode::finish_write_session()
{
  ::ftruncate(session_fd_, session_write_high_water_);
  ::close(session_fd_);

  // JSON validation happens here, post-transfer, not during WriteFile —
  // MAVFTP has no hook to reject mid-transfer for a domain-specific reason
  // (interface spec §5.1 item 2). The file stays on disk either way;
  // `activate` is what actually rejects a bad plan later.
  const std::string full_path = plans_dir_ + "/" + session_filename_;
  mc_msgs::msg::MissionUpload discard;
  if (!parse_plan_json(full_path, discard)) {
    send_statustext(
      MAV_SEVERITY_WARNING,
      "mission_orchestrator_node: " + session_filename_ + " has invalid JSON");
  }

  close_session_state();
}

void MissionOrchestratorNode::close_session_fd_only()
{
  if (session_fd_ >= 0) {
    ::close(session_fd_);
  }
  close_session_state();
}

void MissionOrchestratorNode::close_session_state()
{
  session_fd_ = -1;
  session_mode_ = SessionMode::kNone;
  session_filename_.clear();
  session_write_high_water_ = 0;
  active_read_buffer_.clear();
}

// --- MC_MISSION_PLAN_ACTIVATE/ACK + plan library (ICD §12.2/§12.4/§12.5) ---

void MissionOrchestratorNode::handle_activate_request(
  const RawFrame & frame,
  const sockaddr_in & from)
{
  uint32_t client_request_id = 0;
  std::memcpy(
    &client_request_id, frame.payload.data() + kActivateOffsetClientRequestId,
    sizeof(client_request_id));

  char filename_buf[kActivateFilenameLen + 1] = {};
  std::memcpy(filename_buf, frame.payload.data() + kActivateOffsetFilename, kActivateFilenameLen);
  const std::string filename(filename_buf);

  mc_msgs::msg::MissionUpload upload;
  const ActivateResult result = load_and_activate_plan(filename, upload);

  std::string error_message;
  switch (result) {
    case kActivateAccepted:
      pub_mission_upload_->publish(upload);
      active_plan_filename_ = filename;
      RCLCPP_INFO(
        get_logger(), "activated mission plan %s (upload_id=%u)", filename.c_str(),
        upload.upload_id);
      break;
    case kActivateRejectedInvalidJson:
      error_message = "invalid mission plan JSON";
      break;
    case kActivateRejectedNotFound:
      error_message = "plan file not found";
      break;
    case kActivateRejectedPhase:
      error_message = "mission phase does not allow activation";
      break;
  }

  send_mission_plan_ack(from, client_request_id, filename, result, error_message);
}

void MissionOrchestratorNode::on_mission_phase(const mc_msgs::msg::MissionPhase::SharedPtr msg)
{
  latest_mission_phase_ = msg->phase;
}

MissionOrchestratorNode::ActivateResult MissionOrchestratorNode::load_and_activate_plan(
  const std::string & filename, mc_msgs::msg::MissionUpload & out_msg)
{
  // Interface spec §5.2's recommendation, resolved: restrict activate to
  // PREFLIGHT. Checked ahead of the filename/file-existence checks below —
  // whether activation is even allowed right now doesn't depend on which
  // plan was requested.
  if (latest_mission_phase_ != mc_msgs::msg::MissionPhase::PREFLIGHT) {
    return kActivateRejectedPhase;
  }

  if (!validate_plan_filename(filename)) {
    return kActivateRejectedNotFound;  // malformed name can't exist under plans_dir_ either way
  }

  const std::string full_path = plans_dir_ + "/" + filename;
  struct stat st {};
  if (::stat(full_path.c_str(), &st) != 0) {
    return kActivateRejectedNotFound;
  }

  if (!parse_plan_json(full_path, out_msg)) {
    return kActivateRejectedInvalidJson;
  }

  out_msg.header.stamp = now();
  out_msg.upload_id = upload_ids_.next();  // fresh upload_id (interface spec §6.4)
  return kActivateAccepted;
}

bool MissionOrchestratorNode::parse_plan_json(
  const std::string & full_path, mc_msgs::msg::MissionUpload & out_msg) const
{
  std::ifstream file(full_path, std::ios::binary);
  if (!file) {
    return false;
  }
  std::ostringstream ss;
  ss << file.rdbuf();
  const std::string content = ss.str();

  try {
    const auto j = nlohmann::json::parse(content);
    if (!j.is_object() || !j.contains("name") || !j.at("name").is_string()) {
      return false;
    }

    out_msg = mc_msgs::msg::MissionUpload{};

    if (j.contains("ingress_waypoints")) {
      for (const auto & wp : j.at("ingress_waypoints")) {
        geometry_msgs::msg::PoseStamped pose;
        // ICD §12.4/interface spec §6.1: speed_mps has no home in
        // geometry_msgs/PoseStamped (matches gcs_mavlink_node's own
        // ingress-waypoint decode, which drops the equivalent field too).
        pose.pose.position.x = wp.at("lat").get<double>();
        pose.pose.position.y = wp.at("lon").get<double>();
        pose.pose.position.z = wp.at("alt_m").get<double>();
        pose.pose.orientation.w = 1.0;
        out_msg.ingress_waypoints.push_back(pose);
      }
    }
    if (j.contains("formation_offsets")) {
      for (const auto & fo : j.at("formation_offsets")) {
        mc_msgs::msg::FormationOffset offset;
        offset.drone_id = fo.at("drone_id").get<uint8_t>();
        offset.n = fo.at("north_m").get<float>();
        offset.e = fo.at("east_m").get<float>();
        offset.d = fo.at("down_m").get<float>();
        out_msg.formation_offsets.push_back(offset);
      }
    }
    if (j.contains("search_polygon")) {
      for (const auto & v : j.at("search_polygon")) {
        geographic_msgs::msg::GeoPoint vertex;
        vertex.latitude = v.at("lat").get<double>();
        vertex.longitude = v.at("lon").get<double>();
        vertex.altitude = 0.0;  // not part of the JSON format (interface spec §6.1)
        out_msg.search_polygon.push_back(vertex);
      }
    }
    if (j.contains("swarm_manifest")) {
      for (const auto & p : j.at("swarm_manifest")) {
        mc_msgs::msg::SwarmPeer peer;
        peer.drone_id = p.at("drone_id").get<uint8_t>();
        peer.ip_address = p.at("ip").get<std::string>();
        // Every other mc_msgs/SwarmPeer field is populated later from live
        // peer traffic (SDD §5.4) — same as gcs_mavlink_node's equivalent
        // decode for the standard-MAVLink-mission-protocol path.
        out_msg.swarm_manifest.push_back(peer);
      }
    }

    out_msg.arm_authorised = j.value("arm_authorised", false);
    out_msg.low_battery_threshold = j.value("low_battery_threshold", 0.20F);
    out_msg.offboard_phase_mask = j.value("offboard_phase_mask", static_cast<uint16_t>(0));
    // Rejected here (activate -> INVALID_JSON) rather than left to fail
    // pre-arm -- same flyable set as mission_exec's is_valid_skip_phase_mask().
    // Read as a JSON integer first: j.value<uint16_t>() would silently
    // narrow 65776 or 240.9 into a valid mask.
    using U = mc_msgs::msg::MissionUpload;
    out_msg.skip_phase_mask = 0;
    if (const auto it = j.find("skip_phase_mask"); it != j.end()) {
      if (!it->is_number_integer()) {
        return false;
      }
      const auto mask = it->get<int64_t>();
      if (mask != 0 && mask != (U::SKIP_SWARM_PHASES | U::SKIP_SEARCH)) {
        return false;
      }
      out_msg.skip_phase_mask = static_cast<uint16_t>(mask);
    }
  } catch (const nlohmann::json::exception &) {
    return false;
  }
  return true;
}

}  // namespace gcs_comms
