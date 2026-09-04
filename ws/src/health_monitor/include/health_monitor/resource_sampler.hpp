// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §10, ICD-MC-001 §9).

#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <mc_msgs/msg/system_health.hpp>

namespace health_monitor
{

// HEALTH-001: resource-sampling functions behind health_monitor_node's 1 Hz
// timer (SDD-MC-001 §10.1). Every metric is read through a caller-supplied
// path or provider function rather than a hardcoded syscall/subprocess, so
// each one is independently unit-testable against a fixture file/string
// instead of the real /proc, /sys, or tegrastats output — this ticket's own
// "mocked /proc/nvidia-smi-equivalent readings" acceptance criterion.
// node_alive_mask/fault_mask (SDD §10.2/10.3) are HEALTH-002/003 work built
// on top of this same message; this file only fills the 5 resource fields.

// Cumulative jiffie counters from a /proc/stat "cpu " line, used to compute
// utilisation as a delta over the sampling interval rather than an
// instantaneous snapshot (SDD §10.1: "delta over 1 s").
struct CpuTotals
{
  uint64_t idle{0};   // idle + iowait
  uint64_t total{0};  // sum of user+nice+system+idle+iowait+irq+softirq+steal
};

// Parses the leading "cpu  ..." line out of /proc/stat's contents. Returns
// std::nullopt if that line is missing or its first token isn't "cpu".
// Missing trailing fields (older kernels report as few as 4) are treated
// as 0 rather than rejected.
std::optional<CpuTotals> parse_proc_stat(const std::string & contents);

// 0-100 utilisation over the interval between `prev` and `cur`. Returns 0
// if the interval is zero-length or counters went backwards (first-ever
// sample, or a /proc/stat rollback) instead of dividing by zero or going
// negative.
uint8_t cpu_pct_from_totals(const CpuTotals & prev, const CpuTotals & cur);

// Parses MemTotal/MemAvailable (kB) out of /proc/meminfo's contents and
// returns 0-100 % used. Returns std::nullopt if either key is missing or
// MemTotal is 0.
std::optional<uint8_t> parse_meminfo_pct(const std::string & contents);

// Extracts the integer part of a tegrastats "GPU@<N>C" (or "GPU@<N>.<N>C")
// token out of a raw tegrastats output line — the "nvidia-smi-equivalent"
// GPU temperature source on Jetson, which has no nvidia-smi (HID-MC-001
// §8.5, SDD §10.1). Returns std::nullopt if no such token is present.
std::optional<uint8_t> parse_tegrastats_gpu_temp_c(const std::string & line);

// Reads each path in `thermal_zone_paths` (each expected to hold a single
// integer, millidegrees C, as /sys/class/thermal/thermal_zone*/temp does)
// and returns the maximum across all readable paths. Returns std::nullopt
// if none of them are readable/parseable.
std::optional<uint8_t> max_thermal_zone_temp_c(
  const std::vector<std::string> & thermal_zone_paths);

// INFRA-007: parses a cgroup v2 memory.current value and memory.max value
// (as read verbatim from those two files under the vision.service cgroup,
// e.g. /sys/fs/cgroup/be.slice/drone-mc-vision.service/) into a 0-100 %
// usage-of-cap figure. memory.max may hold the literal "max" instead of a
// byte count when no cap is set -- that's std::nullopt (no cap means no
// percentage to alert against), not 0, since the caller (health_monitor_
// node's sustained-threshold check) must not treat "no data" as "healthy".
std::optional<uint8_t> parse_cgroup_mem_pct(
  const std::string & current_contents, const std::string & max_contents);

// Reads current_path/max_path (cgroup v2 memory.current/memory.max) and
// returns parse_cgroup_mem_pct's result, or std::nullopt if either path
// isn't readable. Free function (like max_thermal_zone_temp_c) so a test
// can point it at fixture files directly.
std::optional<uint8_t> read_cgroup_mem_pct(
  const std::string & current_path, const std::string & max_path);

// Every external data source health_monitor_node's timer reads, bundled so
// a test can point each one at a fixture instead of the real filesystem/
// subprocess — the same swap-the-seam precedent as fcc_bridge's
// FccBackend, applied to file/process I/O instead of a wire protocol.
struct ResourceSamplerConfig
{
  std::string proc_stat_path{"/proc/stat"};
  std::string meminfo_path{"/proc/meminfo"};
  std::string disk_path{"/var/drone-mc"};
  std::vector<std::string> cpu_thermal_zone_paths{"/sys/class/thermal/thermal_zone0/temp"};
  // "timeout" bounds tegrastats' own indefinite streaming so the
  // popen()/pclose() pair behind the default GPU temp provider can't block
  // the 1 Hz timer waiting on a process that never exits on its own.
  std::string gpu_temp_command{"timeout 1 tegrastats --interval 500"};
  // INFRA-007: vision.service's cgroup v2 memory accounting files. Default
  // path matches drone-mc-vision.service's Slice=be.slice + unit name
  // (SDD §12.3/§12.4) under the unified cgroup v2 hierarchy's standard
  // mountpoint.
  std::string vision_cgroup_memory_current_path{
    "/sys/fs/cgroup/be.slice/drone-mc-vision.service/memory.current"};
  std::string vision_cgroup_memory_max_path{
    "/sys/fs/cgroup/be.slice/drone-mc-vision.service/memory.max"};
};

// Samples system resource metrics for health_monitor_node's 1 Hz timer
// (SDD §10.1). Holds the previous /proc/stat snapshot across calls so
// cpu_pct is a delta, not an instantaneous read; every other metric is
// stateless per call. Any one metric failing to read leaves its field at 0
// rather than aborting the whole sample — the 5 SDD §10.1 sources are
// independent, and one being unavailable (e.g. no GPU thermal node in a
// dev-workstation sandbox) shouldn't blank out the other four.
class ResourceSampler
{
public:
  explicit ResourceSampler(ResourceSamplerConfig config = {});

  // Overrides the real statvfs(2)-backed disk reader with `provider`,
  // called with config's disk_path each sample. Test seam: unlike the
  // other sources, disk usage can't be pointed at a fixture file the way a
  // file path can — the provider itself is the seam instead.
  void set_disk_usage_provider(
    std::function<std::optional<uint8_t>(const std::string &)> provider);

  // Overrides the real popen(3)-backed tegrastats reader with `provider`.
  // Test seam: avoids spawning a real subprocess in unit tests.
  void set_gpu_temp_line_provider(std::function<std::optional<std::string>()> provider);

  // INFRA-007: overrides the real cgroupfs-backed vision-memory reader with
  // `provider`. Test seam: avoids depending on a real vision.service cgroup
  // existing on the test host.
  void set_vision_mem_pct_provider(std::function<std::optional<uint8_t>()> provider);

  // Fills cpu_pct/mem_pct/disk_pct/gpu_temp_c/cpu_temp_c on `out`. Does not
  // touch header, fault_mask, node_alive_mask, or uptime_s — those are
  // owned by health_monitor_node (liveness/failsafe land in HEALTH-002/
  // 003; uptime is a node-lifetime concern, not a resource sample).
  void sample(mc_msgs::msg::SystemHealth & out);

  // INFRA-007: vision.service's cgroup memory usage as a 0-100 % of its
  // MemoryMax cap, or std::nullopt if unavailable (no cgroup, no cap set,
  // unreadable). Separate from sample()/SystemHealth: this is a per-service
  // figure health_monitor_node's own sustained-threshold check consumes
  // directly, not a system-wide SystemHealth field.
  std::optional<uint8_t> sample_vision_mem_pct();

private:
  ResourceSamplerConfig config_;
  std::optional<CpuTotals> prev_cpu_totals_;
  std::function<std::optional<uint8_t>(const std::string &)> disk_usage_provider_;
  std::function<std::optional<std::string>()> gpu_temp_line_provider_;
  std::function<std::optional<uint8_t>()> vision_mem_pct_provider_;
};

}  // namespace health_monitor
