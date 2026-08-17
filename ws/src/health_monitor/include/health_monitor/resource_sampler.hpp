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

  // Fills cpu_pct/mem_pct/disk_pct/gpu_temp_c/cpu_temp_c on `out`. Does not
  // touch header, fault_mask, node_alive_mask, or uptime_s — those are
  // owned by health_monitor_node (liveness/failsafe land in HEALTH-002/
  // 003; uptime is a node-lifetime concern, not a resource sample).
  void sample(mc_msgs::msg::SystemHealth & out);

private:
  ResourceSamplerConfig config_;
  std::optional<CpuTotals> prev_cpu_totals_;
  std::function<std::optional<uint8_t>(const std::string &)> disk_usage_provider_;
  std::function<std::optional<std::string>()> gpu_temp_line_provider_;
};

}  // namespace health_monitor
