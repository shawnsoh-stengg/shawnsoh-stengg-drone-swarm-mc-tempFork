// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §10, ICD-MC-001 §9).

// HEALTH-001: mocked /proc/nvidia-smi-equivalent readings -> correct
// published values. Pure gtest — no rclcpp::init required, since
// resource_sampler.hpp/.cpp take no dependency on rclcpp::Node or ROS time
// (see its own header doc comment).

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

#include "health_monitor/resource_sampler.hpp"

namespace health_monitor
{
namespace
{

// Writes `contents` to a fresh file under the test's own tmp directory and
// returns its path; each test uses a distinct suffix to avoid collisions
// when run in parallel.
std::string write_fixture(const std::string & suffix, const std::string & contents)
{
  const std::string path = "/tmp/health_monitor_test_" + suffix;
  std::ofstream out(path, std::ios::trunc);
  out << contents;
  out.close();
  return path;
}

}  // namespace

TEST(ParseProcStat, ParsesStandardEightFieldLine)
{
  const auto totals = parse_proc_stat("cpu  100 0 100 700 100 0 0 0\nother line\n");
  ASSERT_TRUE(totals.has_value());
  EXPECT_EQ(totals->idle, 800u);   // idle(700) + iowait(100)
  EXPECT_EQ(totals->total, 1000u);  // sum of all 8 fields
}

TEST(ParseProcStat, RejectsMissingCpuLabel)
{
  EXPECT_FALSE(parse_proc_stat("notcpu 1 2 3 4").has_value());
}

TEST(ParseProcStat, TreatsShortLineFieldsAsZero)
{
  // Some kernels report only user/nice/system/idle.
  const auto totals = parse_proc_stat("cpu 10 0 10 80");
  ASSERT_TRUE(totals.has_value());
  EXPECT_EQ(totals->idle, 80u);
  EXPECT_EQ(totals->total, 100u);
}

TEST(CpuPctFromTotals, ComputesUtilisationAsDeltaOverInterval)
{
  CpuTotals prev;
  prev.idle = 800;
  prev.total = 1000;
  CpuTotals cur;
  cur.idle = 850;
  cur.total = 1100;
  // delta_idle=50, delta_total=100 -> 50% idle -> 50% used.
  EXPECT_EQ(cpu_pct_from_totals(prev, cur), 50);
}

TEST(CpuPctFromTotals, ReturnsZeroOnZeroLengthInterval)
{
  CpuTotals prev;
  prev.idle = 800;
  prev.total = 1000;
  EXPECT_EQ(cpu_pct_from_totals(prev, prev), 0);
}

TEST(CpuPctFromTotals, ReturnsZeroOnCounterRollback)
{
  CpuTotals prev;
  prev.idle = 800;
  prev.total = 1000;
  CpuTotals cur;
  cur.idle = 10;
  cur.total = 20;
  EXPECT_EQ(cpu_pct_from_totals(prev, cur), 0);
}

TEST(ParseMeminfoPct, ComputesPercentUsedFromTotalAndAvailable)
{
  const std::string meminfo =
    "MemTotal:       16000000 kB\n"
    "MemFree:         1000000 kB\n"
    "MemAvailable:    4000000 kB\n";
  const auto pct = parse_meminfo_pct(meminfo);
  ASSERT_TRUE(pct.has_value());
  // (16000000 - 4000000) / 16000000 = 75%
  EXPECT_EQ(*pct, 75);
}

TEST(ParseMeminfoPct, ReturnsNulloptWhenKeysMissing)
{
  EXPECT_FALSE(parse_meminfo_pct("SomeOtherKey: 123 kB\n").has_value());
}

TEST(ParseTegrastatsGpuTempC, ExtractsIntegerDegreesFromToken)
{
  const std::string line =
    "RAM 3245/7854MB ... CPU@[12%,4%,3%,7%] GPU@43C PMIC@50C thermal@45.5C\n";
  const auto temp = parse_tegrastats_gpu_temp_c(line);
  ASSERT_TRUE(temp.has_value());
  EXPECT_EQ(*temp, 43);
}

TEST(ParseTegrastatsGpuTempC, ReturnsNulloptWhenTokenAbsent)
{
  EXPECT_FALSE(parse_tegrastats_gpu_temp_c("no gpu token here").has_value());
}

TEST(MaxThermalZoneTempC, ReturnsMaxAcrossMultipleZonesInMillidegrees)
{
  const std::string zone0 = write_fixture("zone0", "45000\n");
  const std::string zone1 = write_fixture("zone1", "62000\n");
  const auto temp = max_thermal_zone_temp_c({zone0, zone1});
  ASSERT_TRUE(temp.has_value());
  EXPECT_EQ(*temp, 62);
}

TEST(MaxThermalZoneTempC, ReturnsNulloptWhenNoPathIsReadable)
{
  EXPECT_FALSE(max_thermal_zone_temp_c({"/no/such/path/at/all"}).has_value());
}

TEST(ResourceSamplerSample, FillsAllFieldsFromMockedSources)
{
  const std::string proc_stat_path = write_fixture("proc_stat", "cpu  100 0 100 700 100 0 0 0\n");
  const std::string meminfo_path = write_fixture(
    "meminfo", "MemTotal:       10000 kB\nMemAvailable:    2000 kB\n");
  const std::string thermal_path = write_fixture("thermal", "55500\n");

  ResourceSamplerConfig config;
  config.proc_stat_path = proc_stat_path;
  config.meminfo_path = meminfo_path;
  config.cpu_thermal_zone_paths = {thermal_path};

  ResourceSampler sampler(config);
  sampler.set_disk_usage_provider([](const std::string &) {return std::optional<uint8_t>(37);});
  sampler.set_gpu_temp_line_provider(
    [] {return std::optional<std::string>("RAM ... GPU@48C ...");});

  mc_msgs::msg::SystemHealth first;
  sampler.sample(first);
  // First sample has no prior /proc/stat snapshot to delta against.
  EXPECT_EQ(first.cpu_pct, 0);
  EXPECT_EQ(first.mem_pct, 80);  // (10000-2000)/10000
  EXPECT_EQ(first.disk_pct, 37);
  EXPECT_EQ(first.gpu_temp_c, 48);
  EXPECT_EQ(first.cpu_temp_c, 56);  // 55500 millidegrees, rounds up

  // Advance /proc/stat so the second sample has a real delta to compute.
  {
    std::ofstream out(proc_stat_path, std::ios::trunc);
    out << "cpu  200 0 100 750 100 0 0 0\n";
  }
  mc_msgs::msg::SystemHealth second;
  sampler.sample(second);
  // delta_idle = (750+100)-(700+100) = 50, delta_total = 1150-1000 = 150
  // -> 1 - 50/150 = 66.67% -> rounds to 67.
  EXPECT_EQ(second.cpu_pct, 67);
  EXPECT_EQ(second.disk_pct, 37);
  EXPECT_EQ(second.gpu_temp_c, 48);
}

TEST(ResourceSamplerSample, LeavesFieldAtZeroWhenOneSourceIsUnavailable)
{
  ResourceSamplerConfig config;
  config.proc_stat_path = "/no/such/proc/stat";
  config.meminfo_path = "/no/such/meminfo";
  config.cpu_thermal_zone_paths = {"/no/such/thermal/zone"};

  ResourceSampler sampler(config);
  sampler.set_disk_usage_provider(
    [](const std::string &) {return std::optional<uint8_t>(std::nullopt);});
  sampler.set_gpu_temp_line_provider([] {return std::optional<std::string>(std::nullopt);});

  mc_msgs::msg::SystemHealth msg;
  sampler.sample(msg);
  EXPECT_EQ(msg.cpu_pct, 0);
  EXPECT_EQ(msg.mem_pct, 0);
  EXPECT_EQ(msg.disk_pct, 0);
  EXPECT_EQ(msg.gpu_temp_c, 0);
  EXPECT_EQ(msg.cpu_temp_c, 0);
}

}  // namespace health_monitor
