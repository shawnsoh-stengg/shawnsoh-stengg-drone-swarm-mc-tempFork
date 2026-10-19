// upload_id allocation shared by gcs_mavlink_node (Channel B) and
// mission_orchestrator_node (Channel C). Both publish
// mc_msgs/MissionUpload on /mc/gcs/mission_upload, and each used to count
// its own upload_id from 1, so the two channels (and any node restart)
// reissued the same IDs. mission_manager_node's stale-VoronoiCell guard
// compares upload_ids, so a reissued ID could pass a cell from an older
// upload as current (TEST-007 finding 2).
//
// Layout, kept below 2^24 so the ID survives MC_CMD_ARM_AUTHORISE's float
// param1 exactly:
//   bit 23     source: 0 = Channel B, 1 = Channel C
//   bits 0-22  counter, never 0, seeded from wall-clock seconds at startup
//              so a restarted node starts past the IDs it issued before
//              (wraps every ~97 days)
#pragma once

#include <chrono>
#include <cstdint>

namespace gcs_comms
{

enum class UploadSource : uint32_t
{
  kChannelB = 0,
  kChannelC = 1,
};

constexpr uint32_t kUploadIdCounterBits = 23;
constexpr uint32_t kUploadIdCounterMask = (1U << kUploadIdCounterBits) - 1U;

class UploadIdAllocator
{
public:
  UploadIdAllocator(UploadSource source, uint64_t seed)
  : source_bit_(static_cast<uint32_t>(source) << kUploadIdCounterBits),
    counter_(static_cast<uint32_t>(seed) & kUploadIdCounterMask)
  {
  }

  explicit UploadIdAllocator(UploadSource source)
  : UploadIdAllocator(source, wall_clock_seed())
  {
  }

  uint32_t next()
  {
    counter_ = (counter_ + 1U) & kUploadIdCounterMask;
    if (counter_ == 0U) {
      counter_ = 1U;
    }
    return source_bit_ | counter_;
  }

  static uint64_t wall_clock_seed()
  {
    return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
  }

private:
  uint32_t source_bit_;
  uint32_t counter_;
};

}  // namespace gcs_comms
