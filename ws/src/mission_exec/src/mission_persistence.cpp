// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §4, ICD-MC-001 §9).

#include "mission_exec/mission_persistence.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <fstream>
#include <sstream>

#include <geographic_msgs/msg/geo_point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mc_msgs/msg/formation_offset.hpp>
#include <mc_msgs/msg/swarm_peer.hpp>

namespace mission_exec
{

namespace
{

// yaml-cpp special-cases (u)int8_t as a single character rather than a
// number, so every uint8 message field is round-tripped via `int` instead.

YAML::Node pose_stamped_to_yaml(const geometry_msgs::msg::PoseStamped & wp)
{
  YAML::Node n;
  n["stamp_sec"] = wp.header.stamp.sec;
  n["stamp_nanosec"] = wp.header.stamp.nanosec;
  n["frame_id"] = wp.header.frame_id;
  n["position"]["x"] = wp.pose.position.x;
  n["position"]["y"] = wp.pose.position.y;
  n["position"]["z"] = wp.pose.position.z;
  n["orientation"]["x"] = wp.pose.orientation.x;
  n["orientation"]["y"] = wp.pose.orientation.y;
  n["orientation"]["z"] = wp.pose.orientation.z;
  n["orientation"]["w"] = wp.pose.orientation.w;
  return n;
}

geometry_msgs::msg::PoseStamped pose_stamped_from_yaml(const YAML::Node & n)
{
  geometry_msgs::msg::PoseStamped wp;
  wp.header.stamp.sec = n["stamp_sec"].as<int32_t>();
  wp.header.stamp.nanosec = n["stamp_nanosec"].as<uint32_t>();
  wp.header.frame_id = n["frame_id"].as<std::string>();
  wp.pose.position.x = n["position"]["x"].as<double>();
  wp.pose.position.y = n["position"]["y"].as<double>();
  wp.pose.position.z = n["position"]["z"].as<double>();
  wp.pose.orientation.x = n["orientation"]["x"].as<double>();
  wp.pose.orientation.y = n["orientation"]["y"].as<double>();
  wp.pose.orientation.z = n["orientation"]["z"].as<double>();
  wp.pose.orientation.w = n["orientation"]["w"].as<double>();
  return wp;
}

YAML::Node formation_offset_to_yaml(const mc_msgs::msg::FormationOffset & o)
{
  YAML::Node n;
  n["drone_id"] = static_cast<int>(o.drone_id);
  n["n"] = o.n;
  n["e"] = o.e;
  n["d"] = o.d;
  return n;
}

mc_msgs::msg::FormationOffset formation_offset_from_yaml(const YAML::Node & n)
{
  mc_msgs::msg::FormationOffset o;
  o.drone_id = static_cast<uint8_t>(n["drone_id"].as<int>());
  o.n = n["n"].as<float>();
  o.e = n["e"].as<float>();
  o.d = n["d"].as<float>();
  return o;
}

YAML::Node geo_point_to_yaml(const geographic_msgs::msg::GeoPoint & p)
{
  YAML::Node n;
  n["latitude"] = p.latitude;
  n["longitude"] = p.longitude;
  n["altitude"] = p.altitude;
  return n;
}

geographic_msgs::msg::GeoPoint geo_point_from_yaml(const YAML::Node & n)
{
  geographic_msgs::msg::GeoPoint p;
  p.latitude = n["latitude"].as<double>();
  p.longitude = n["longitude"].as<double>();
  p.altitude = n["altitude"].as<double>();
  return p;
}

YAML::Node swarm_peer_to_yaml(const mc_msgs::msg::SwarmPeer & p)
{
  YAML::Node n;
  n["drone_id"] = static_cast<int>(p.drone_id);
  n["ip_address"] = p.ip_address;
  n["mission_phase"] = static_cast<int>(p.mission_phase);
  n["lat"] = p.lat;
  n["lon"] = p.lon;
  n["alt_msl"] = p.alt_msl;
  n["vx"] = p.vx;
  n["vy"] = p.vy;
  n["vz"] = p.vz;
  n["battery_pct"] = p.battery_pct;
  n["sensor_flags"] = static_cast<int>(p.sensor_flags);
  n["fault_mask"] = p.fault_mask;
  n["peer_status"] = static_cast<int>(p.peer_status);
  n["is_leader"] = p.is_leader;
  n["isr_role"] = p.isr_role;
  n["last_heartbeat_us"] = p.last_heartbeat_us;
  return n;
}

mc_msgs::msg::SwarmPeer swarm_peer_from_yaml(const YAML::Node & n)
{
  mc_msgs::msg::SwarmPeer p;
  p.drone_id = static_cast<uint8_t>(n["drone_id"].as<int>());
  p.ip_address = n["ip_address"].as<std::string>();
  p.mission_phase = static_cast<uint8_t>(n["mission_phase"].as<int>());
  p.lat = n["lat"].as<double>();
  p.lon = n["lon"].as<double>();
  p.alt_msl = n["alt_msl"].as<float>();
  p.vx = n["vx"].as<float>();
  p.vy = n["vy"].as<float>();
  p.vz = n["vz"].as<float>();
  p.battery_pct = n["battery_pct"].as<float>();
  p.sensor_flags = static_cast<uint8_t>(n["sensor_flags"].as<int>());
  p.fault_mask = n["fault_mask"].as<uint16_t>();
  p.peer_status = static_cast<uint8_t>(n["peer_status"].as<int>());
  p.is_leader = n["is_leader"].as<bool>();
  p.isr_role = n["isr_role"].as<bool>();
  p.last_heartbeat_us = n["last_heartbeat_us"].as<uint64_t>();
  return p;
}

// Writes `contents` to `path` via a sibling `<path>.tmp` file, fsynced and
// then rename()d over `path`. Returns false on any I/O failure.
bool atomic_write(const std::string & path, const std::string & contents)
{
  const std::string tmp_path = path + ".tmp";
  const int fd = ::open(tmp_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0) {
    return false;
  }
  const auto * data = reinterpret_cast<const uint8_t *>(contents.data());
  size_t written = 0;
  bool write_ok = true;
  while (written < contents.size()) {
    const ssize_t n = ::write(fd, data + written, contents.size() - written);
    if (n < 0) {
      write_ok = false;
      break;
    }
    written += static_cast<size_t>(n);
  }
  const bool fsync_ok = write_ok && (::fsync(fd) == 0);
  ::close(fd);
  if (!write_ok || !fsync_ok) {
    ::unlink(tmp_path.c_str());
    return false;
  }
  if (std::rename(tmp_path.c_str(), path.c_str()) != 0) {
    ::unlink(tmp_path.c_str());
    return false;
  }
  return true;
}

}  // namespace

bool save_mission_upload(const std::string & path, const mc_msgs::msg::MissionUpload & upload)
{
  YAML::Node root;
  root["header_stamp_sec"] = upload.header.stamp.sec;
  root["header_stamp_nanosec"] = upload.header.stamp.nanosec;
  root["header_frame_id"] = upload.header.frame_id;
  root["upload_id"] = upload.upload_id;

  YAML::Node waypoints(YAML::NodeType::Sequence);
  for (const auto & wp : upload.ingress_waypoints) {
    waypoints.push_back(pose_stamped_to_yaml(wp));
  }
  root["ingress_waypoints"] = waypoints;

  YAML::Node offsets(YAML::NodeType::Sequence);
  for (const auto & o : upload.formation_offsets) {
    offsets.push_back(formation_offset_to_yaml(o));
  }
  root["formation_offsets"] = offsets;

  YAML::Node polygon(YAML::NodeType::Sequence);
  for (const auto & pt : upload.search_polygon) {
    polygon.push_back(geo_point_to_yaml(pt));
  }
  root["search_polygon"] = polygon;

  YAML::Node manifest(YAML::NodeType::Sequence);
  for (const auto & peer : upload.swarm_manifest) {
    manifest.push_back(swarm_peer_to_yaml(peer));
  }
  root["swarm_manifest"] = manifest;

  root["arm_authorised"] = upload.arm_authorised;
  root["low_battery_threshold"] = upload.low_battery_threshold;
  root["offboard_phase_mask"] = upload.offboard_phase_mask;

  YAML::Emitter emitter;
  emitter << root;
  if (!emitter.good()) {
    return false;
  }

  std::ostringstream oss;
  oss << emitter.c_str() << "\n";
  return atomic_write(path, oss.str());
}

std::optional<mc_msgs::msg::MissionUpload> load_mission_upload(const std::string & path)
{
  std::ifstream check(path);
  if (!check.good()) {
    return std::nullopt;
  }
  check.close();

  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception &) {
    return std::nullopt;
  }
  if (!root || root.IsNull()) {
    return std::nullopt;
  }

  try {
    mc_msgs::msg::MissionUpload upload;
    upload.header.stamp.sec = root["header_stamp_sec"].as<int32_t>();
    upload.header.stamp.nanosec = root["header_stamp_nanosec"].as<uint32_t>();
    upload.header.frame_id = root["header_frame_id"].as<std::string>();
    upload.upload_id = root["upload_id"].as<uint32_t>();

    for (const auto & n : root["ingress_waypoints"]) {
      upload.ingress_waypoints.push_back(pose_stamped_from_yaml(n));
    }
    for (const auto & n : root["formation_offsets"]) {
      upload.formation_offsets.push_back(formation_offset_from_yaml(n));
    }
    for (const auto & n : root["search_polygon"]) {
      upload.search_polygon.push_back(geo_point_from_yaml(n));
    }
    for (const auto & n : root["swarm_manifest"]) {
      upload.swarm_manifest.push_back(swarm_peer_from_yaml(n));
    }

    upload.arm_authorised = root["arm_authorised"].as<bool>();
    upload.low_battery_threshold = root["low_battery_threshold"].as<float>();
    upload.offboard_phase_mask = root["offboard_phase_mask"].as<uint16_t>();
    return upload;
  } catch (const YAML::Exception &) {
    return std::nullopt;
  }
}

}  // namespace mission_exec
