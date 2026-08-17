// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §10, ICD-MC-001 §9).

#include "health_monitor/resource_sampler.hpp"

#include <sys/statvfs.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace health_monitor
{

namespace
{

std::optional<std::string> read_file_to_string(const std::string & path)
{
  std::ifstream file(path);
  if (!file.good()) {
    return std::nullopt;
  }
  std::ostringstream oss;
  oss << file.rdbuf();
  return oss.str();
}

uint8_t clamp_to_pct(double value)
{
  if (value < 0.0) {
    return 0;
  }
  if (value > 100.0) {
    return 100;
  }
  return static_cast<uint8_t>(std::lround(value));
}

// Real popen(3)-backed tegrastats reader: runs `command`, returns its
// first line of stdout (or std::nullopt if it produced none / failed to
// launch).
std::optional<std::string> run_command_first_line(const std::string & command)
{
  FILE * pipe = ::popen(command.c_str(), "r");
  if (pipe == nullptr) {
    return std::nullopt;
  }
  char buf[512];
  const char * line = std::fgets(buf, sizeof(buf), pipe);
  const std::optional<std::string> result =
    (line != nullptr) ? std::make_optional(std::string(line)) : std::nullopt;
  ::pclose(pipe);
  return result;
}

// Real statvfs(2)-backed disk usage reader, matching `df`'s "Use%" (based
// on space available to unprivileged callers, f_bavail, not raw free
// blocks f_bfree).
std::optional<uint8_t> statvfs_disk_pct(const std::string & path)
{
  struct statvfs vfs{};
  if (::statvfs(path.c_str(), &vfs) != 0 || vfs.f_blocks == 0) {
    return std::nullopt;
  }
  const double used = static_cast<double>(vfs.f_blocks - vfs.f_bavail);
  return clamp_to_pct(100.0 * used / static_cast<double>(vfs.f_blocks));
}

}  // namespace

std::optional<CpuTotals> parse_proc_stat(const std::string & contents)
{
  std::istringstream iss(contents);
  std::string label;
  if (!(iss >> label) || label != "cpu") {
    return std::nullopt;
  }
  // user nice system idle iowait irq softirq steal [guest guest_nice ...] —
  // guest/guest_nice are already counted inside user/nice per the kernel's
  // own accounting, so they're read (to advance past them) but excluded
  // from `total`.
  uint64_t fields[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  for (auto & field : fields) {
    if (!(iss >> field)) {
      break;
    }
  }
  CpuTotals totals;
  totals.idle = fields[3] + fields[4];
  for (const uint64_t field : fields) {
    totals.total += field;
  }
  return totals;
}

uint8_t cpu_pct_from_totals(const CpuTotals & prev, const CpuTotals & cur)
{
  if (cur.total <= prev.total || cur.idle < prev.idle) {
    return 0;
  }
  const uint64_t delta_total = cur.total - prev.total;
  const uint64_t delta_idle = cur.idle - prev.idle;
  return clamp_to_pct(100.0 * (1.0 - static_cast<double>(delta_idle) / delta_total));
}

std::optional<uint8_t> parse_meminfo_pct(const std::string & contents)
{
  std::optional<uint64_t> mem_total_kb;
  std::optional<uint64_t> mem_available_kb;

  std::istringstream iss(contents);
  std::string line;
  while (std::getline(iss, line)) {
    std::istringstream line_iss(line);
    std::string key;
    uint64_t value = 0;
    if (!(line_iss >> key >> value)) {
      continue;
    }
    if (key == "MemTotal:") {
      mem_total_kb = value;
    } else if (key == "MemAvailable:") {
      mem_available_kb = value;
    }
  }

  if (!mem_total_kb.has_value() || !mem_available_kb.has_value() || *mem_total_kb == 0) {
    return std::nullopt;
  }
  const uint64_t used_kb = *mem_total_kb - std::min(*mem_available_kb, *mem_total_kb);
  const double used = static_cast<double>(used_kb);
  return clamp_to_pct(100.0 * used / static_cast<double>(*mem_total_kb));
}

std::optional<uint8_t> parse_tegrastats_gpu_temp_c(const std::string & line)
{
  const std::string needle = "GPU@";
  const size_t pos = line.find(needle);
  if (pos == std::string::npos) {
    return std::nullopt;
  }
  const size_t digits_start = pos + needle.size();
  size_t digits_end = digits_start;
  while (digits_end < line.size() && std::isdigit(static_cast<unsigned char>(line[digits_end]))) {
    ++digits_end;
  }
  if (digits_end == digits_start) {
    return std::nullopt;
  }
  try {
    return static_cast<uint8_t>(std::stoul(line.substr(digits_start, digits_end - digits_start)));
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::optional<uint8_t> max_thermal_zone_temp_c(
  const std::vector<std::string> & thermal_zone_paths)
{
  std::optional<int64_t> max_millidegrees;
  for (const std::string & path : thermal_zone_paths) {
    const std::optional<std::string> contents = read_file_to_string(path);
    if (!contents.has_value()) {
      continue;
    }
    try {
      const int64_t millidegrees = std::stoll(*contents);
      if (!max_millidegrees.has_value() || millidegrees > *max_millidegrees) {
        max_millidegrees = millidegrees;
      }
    } catch (const std::exception &) {
      continue;
    }
  }
  if (!max_millidegrees.has_value()) {
    return std::nullopt;
  }
  return clamp_to_pct(static_cast<double>(*max_millidegrees) / 1000.0);
}

ResourceSampler::ResourceSampler(ResourceSamplerConfig config)
: config_(std::move(config)),
  disk_usage_provider_(statvfs_disk_pct),
  gpu_temp_line_provider_(
    [this]() {return run_command_first_line(config_.gpu_temp_command);})
{
}

void ResourceSampler::set_disk_usage_provider(
  std::function<std::optional<uint8_t>(const std::string &)> provider)
{
  disk_usage_provider_ = std::move(provider);
}

void ResourceSampler::set_gpu_temp_line_provider(
  std::function<std::optional<std::string>()> provider)
{
  gpu_temp_line_provider_ = std::move(provider);
}

void ResourceSampler::sample(mc_msgs::msg::SystemHealth & out)
{
  if (const std::optional<std::string> stat_contents = read_file_to_string(config_.proc_stat_path);
    stat_contents.has_value())
  {
    if (const std::optional<CpuTotals> cur = parse_proc_stat(*stat_contents); cur.has_value()) {
      if (prev_cpu_totals_.has_value()) {
        out.cpu_pct = cpu_pct_from_totals(*prev_cpu_totals_, *cur);
      }
      prev_cpu_totals_ = cur;
    }
  }

  if (const std::optional<std::string> meminfo_contents =
    read_file_to_string(config_.meminfo_path); meminfo_contents.has_value())
  {
    if (const std::optional<uint8_t> mem_pct = parse_meminfo_pct(*meminfo_contents);
      mem_pct.has_value())
    {
      out.mem_pct = *mem_pct;
    }
  }

  if (const std::optional<uint8_t> disk_pct = disk_usage_provider_(config_.disk_path);
    disk_pct.has_value())
  {
    out.disk_pct = *disk_pct;
  }

  if (const std::optional<std::string> gpu_line = gpu_temp_line_provider_(); gpu_line.has_value()) {
    if (const std::optional<uint8_t> gpu_temp = parse_tegrastats_gpu_temp_c(*gpu_line);
      gpu_temp.has_value())
    {
      out.gpu_temp_c = *gpu_temp;
    }
  }

  if (const std::optional<uint8_t> cpu_temp =
    max_thermal_zone_temp_c(config_.cpu_thermal_zone_paths); cpu_temp.has_value())
  {
    out.cpu_temp_c = *cpu_temp;
  }
}

}  // namespace health_monitor
