// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §4, ICD-MC-001 §9).

#pragma once

#include <optional>
#include <string>

#include <mc_msgs/msg/mission_upload.hpp>

namespace mission_exec
{

// EXEC-004: Mission upload disk persistence (SDD-MC-001 §4.5, FR-021/060).
// Deliberately framework-free (no rclcpp::Node, no ROS clock) — plain
// functions over a caller-supplied path and message, same "independently
// callable/testable" precedent as pre_arm_checks.hpp/command_arbiter.hpp.
//
// Unrelated to mission_orchestrator_node's JSON plan library under
// /var/drone-mc/mission_plans/ (ICD §12.4) — this is the single
// most-recently-accepted mc_msgs/MissionUpload (from either source), SDD
// §4.5's own "unrelated file, unrelated format" callout.

// Serializes `upload` to YAML and writes it to `path` atomically: the
// encoded document is written to a sibling `<path>.tmp` file first, fsynced,
// then rename()d over `path` — rename() is atomic within the same
// filesystem, so a crash or watchdog-triggered restart mid-write can never
// leave load_mission_upload() looking at a truncated/partial file. Returns
// false on any I/O failure (missing directory, disk full, ...); never
// throws and does not log — callers decide how to surface a failure.
bool save_mission_upload(const std::string & path, const mc_msgs::msg::MissionUpload & upload);

// Reads and parses `path`. Returns std::nullopt if the file doesn't exist,
// is empty, or fails to parse — e.g. first-ever boot with nothing uploaded
// yet. Never throws.
std::optional<mc_msgs::msg::MissionUpload> load_mission_upload(const std::string & path);

// EXEC-006: deletes `path` (and any leftover `<path>.tmp`), so a restart
// after a mission recycle doesn't reload the already-flown upload and its
// ARM authorisation (FR-010). Returns true if the file is gone afterwards,
// including when it never existed. Never throws and does not log.
bool remove_mission_upload(const std::string & path);

}  // namespace mission_exec
