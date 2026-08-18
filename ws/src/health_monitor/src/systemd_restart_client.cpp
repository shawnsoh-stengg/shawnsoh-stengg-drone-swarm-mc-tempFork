// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §10.4).

#include "health_monitor/systemd_restart_client.hpp"

#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace health_monitor
{

SystemdRestartResult request_systemd_restart(
  const std::string & socket_path, const std::string & service, int timeout_ms)
{
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  if (socket_path.size() >= sizeof(addr.sun_path)) {
    return {false, "socket_path too long for sockaddr_un"};
  }
  std::strncpy(addr.sun_path, socket_path.c_str(), sizeof(addr.sun_path) - 1);

  const int fd = socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0) {
    return {false, std::string("socket() failed: ") + std::strerror(errno)};
  }

  const timeval tv{timeout_ms / 1000, (timeout_ms % 1000) * 1000};
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  if (connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    const std::string reason = std::string("connect() failed: ") + std::strerror(errno);
    close(fd);
    return {false, reason};
  }

  const std::string request = R"({"action":"restart","service":")" + service + "\"}\n";
  if (write(fd, request.data(), request.size()) != static_cast<ssize_t>(request.size())) {
    const std::string reason = std::string("write() failed: ") + std::strerror(errno);
    close(fd);
    return {false, reason};
  }

  char buf[256];
  const ssize_t n = read(fd, buf, sizeof(buf) - 1);
  close(fd);
  if (n <= 0) {
    return {
      false,
      n == 0 ? "watchdog closed the connection with no response" :
      std::string("read() failed: ") + std::strerror(errno)
    };
  }

  const std::string response(buf, static_cast<std::size_t>(n));
  // Not a full JSON parse -- the protocol is one fixed boolean field, so a
  // substring match on the true-shaped token is sufficient and avoids a
  // JSON library dependency for this one call site.
  if (response.find("\"ok\":true") != std::string::npos ||
    response.find("\"ok\": true") != std::string::npos)
  {
    return {true, ""};
  }
  return {false, "watchdog response: " + response};
}

}  // namespace health_monitor
