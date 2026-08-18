// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §10.4).

#include <gtest/gtest.h>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include "health_monitor/systemd_restart_client.hpp"

namespace
{

// Minimal one-shot fake `drone-mc-watchdog` (SDD §10.4): binds `path`,
// accepts a single connection, reads the request, writes back
// `response_line`, then closes. Runs on a background thread so the test
// can drive the real client against it over a real Unix domain socket.
class FakeWatchdog
{
public:
  explicit FakeWatchdog(std::string path)
  : path_(std::move(path))
  {
    std::remove(path_.c_str());
    listen_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, path_.c_str(), sizeof(addr.sun_path) - 1);
    bind(listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
    listen(listen_fd_, 1);
  }

  ~FakeWatchdog()
  {
    if (thread_.joinable()) {
      thread_.join();
    }
    close(listen_fd_);
    std::remove(path_.c_str());
  }

  void serve_once(const std::string & response_line)
  {
    thread_ = std::thread(
      [this, response_line] {
        const int conn_fd = accept(listen_fd_, nullptr, nullptr);
        if (conn_fd < 0) {
          return;
        }
        char buf[256];
        read(conn_fd, buf, sizeof(buf));  // drain the request; content unchecked here
        write(conn_fd, response_line.data(), response_line.size());
        close(conn_fd);
      });
  }

private:
  std::string path_;
  int listen_fd_{-1};
  std::thread thread_;
};

std::string unique_socket_path(const std::string & label)
{
  return "/tmp/health_monitor_test_watchdog_" + label + "_" +
         std::to_string(getpid()) + ".sock";
}

}  // namespace

TEST(SystemdRestartClient, SucceedsOnOkTrueResponse)
{
  const auto path = unique_socket_path("ok");
  FakeWatchdog watchdog(path);
  watchdog.serve_once("{\"ok\":true}\n");

  const auto result = health_monitor::request_systemd_restart(path, "mission-exec.service");
  EXPECT_TRUE(result.ok);
  EXPECT_TRUE(result.reason.empty());
}

TEST(SystemdRestartClient, FailsOnOkFalseResponse)
{
  const auto path = unique_socket_path("reject");
  FakeWatchdog watchdog(path);
  watchdog.serve_once(R"({"ok":false,"reason":"disallowed service"})" "\n");

  const auto result = health_monitor::request_systemd_restart(path, "not-allowlisted.service");
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.reason.find("disallowed service"), std::string::npos);
}

TEST(SystemdRestartClient, FailsWhenNoWatchdogIsListening)
{
  const auto path = unique_socket_path("absent");
  std::remove(path.c_str());  // guarantee nothing is listening here

  const auto result = health_monitor::request_systemd_restart(path, "mission-exec.service", 200);
  EXPECT_FALSE(result.ok);
  EXPECT_FALSE(result.reason.empty());
}
