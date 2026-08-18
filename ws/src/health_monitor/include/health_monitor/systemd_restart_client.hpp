// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §10.4).

#pragma once

#include <string>

namespace health_monitor
{

// HEALTH-003: client for the SDD-MC-001 §10.4 systemd-restart interface --
// a Unix domain socket to the privileged `drone-mc-watchdog` helper that
// runs outside Docker and has permission to call `systemctl restart`
// (avoids running this container in privileged mode). Protocol is a
// single-line JSON request/response pair, deliberately hand-built/parsed
// rather than pulling in a JSON library for one fixed-shape boolean field
// -- the same hand-packed-over-full-parser precedent this repo already
// uses for its MAVLink glue.
struct SystemdRestartResult
{
  bool ok{false};
  std::string reason;  // empty when ok; human-readable failure otherwise
};

// Connects to the Unix domain socket at `socket_path`, sends
// {"action":"restart","service":"<service>"}, and reads back the helper's
// one-line JSON response. Every failure mode (missing socket, connect
// refused, write/read error or timeout, malformed/negative response) is
// reported through the returned result rather than an exception --
// callers run this from a 1 Hz timer thread and must never be blocked or
// crashed by an absent/unresponsive watchdog helper. `timeout_ms` bounds
// the send/receive socket calls (SO_SNDTIMEO/SO_RCVTIMEO); AF_UNIX
// SOCK_STREAM connect() itself is not subject to this timeout since it
// either succeeds or fails immediately (no listener, or backlog full).
SystemdRestartResult request_systemd_restart(
  const std::string & socket_path, const std::string & service, int timeout_ms = 500);

}  // namespace health_monitor
