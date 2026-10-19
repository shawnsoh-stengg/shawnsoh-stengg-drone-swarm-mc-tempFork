// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §11, ICD-MC-001 §4).

// TEST-007 finding 2: Channel B and Channel C upload_ids must never collide,
// across a node restart too, and must survive MC_CMD_ARM_AUTHORISE's float
// param1 exactly.

#include <gtest/gtest.h>

#include <cstdint>
#include <set>

#include "gcs_comms/upload_id.hpp"

using gcs_comms::kUploadIdCounterBits;
using gcs_comms::kUploadIdCounterMask;
using gcs_comms::UploadIdAllocator;
using gcs_comms::UploadSource;

TEST(UploadIdAllocator, ChannelsNeverCollideFromTheSameSeed)
{
  UploadIdAllocator b(UploadSource::kChannelB, 1000);
  UploadIdAllocator c(UploadSource::kChannelC, 1000);
  std::set<uint32_t> seen;
  for (int i = 0; i < 1000; ++i) {
    EXPECT_TRUE(seen.insert(b.next()).second);
    EXPECT_TRUE(seen.insert(c.next()).second);
  }
}

TEST(UploadIdAllocator, SourceBitAndConsecutiveCounter)
{
  UploadIdAllocator b(UploadSource::kChannelB, 41);
  UploadIdAllocator c(UploadSource::kChannelC, 41);
  EXPECT_EQ(b.next(), 42U);
  EXPECT_EQ(b.next(), 43U);
  EXPECT_EQ(c.next(), (1U << kUploadIdCounterBits) | 42U);
}

TEST(UploadIdAllocator, RestartWithLaterSeedStartsPastEarlierIds)
{
  // Seeded from wall-clock seconds: a node restarted 10 s later, having
  // issued fewer than 10 uploads, never reissues one of them.
  UploadIdAllocator before(UploadSource::kChannelB, 5000);
  std::set<uint32_t> issued;
  for (int i = 0; i < 9; ++i) {
    issued.insert(before.next());
  }
  UploadIdAllocator after(UploadSource::kChannelB, 5010);
  EXPECT_EQ(issued.count(after.next()), 0U);
}

TEST(UploadIdAllocator, WrapSkipsZeroAndStaysBelowFloatExactRange)
{
  UploadIdAllocator b(UploadSource::kChannelB, kUploadIdCounterMask - 1U);
  EXPECT_EQ(b.next(), kUploadIdCounterMask);
  EXPECT_EQ(b.next(), 1U);  // wrapped past 0: 0 means "no upload" downstream

  UploadIdAllocator c(UploadSource::kChannelC, kUploadIdCounterMask - 1U);
  const uint32_t top = c.next();
  EXPECT_LT(top, 1U << 24);
  EXPECT_EQ(static_cast<uint32_t>(static_cast<float>(top)), top);
}

TEST(UploadIdAllocator, WallClockSeedIsUsedByDefault)
{
  const uint32_t seed = UploadIdAllocator::wall_clock_seed() & kUploadIdCounterMask;
  UploadIdAllocator b(UploadSource::kChannelB);
  const uint32_t id = b.next();
  // seed + 1, or + 2 if the clock ticked between the two reads.
  EXPECT_GE(id - seed, 1U);
  EXPECT_LE(id - seed, 2U);
}
