#include <gtest/gtest.h>
#include <micromouse/drive/trapezoid_profile.hpp>

using namespace drive;

TEST(TrapezoidProfile, TestSymmetricProfile) {
  TrapezoidProfile::Sample sample;
  TrapezoidProfile profile;

  profile.configure(1000.f, 0.f, 1000.f, 1000.f);

  // Duration.
  ASSERT_FLOAT_EQ(2.f, profile.duration_s());

  // Start.
  sample = profile.sample(0.f);
  ASSERT_FLOAT_EQ(0.f, sample.distance);
  ASSERT_FLOAT_EQ(0.f, sample.velocity);

  // Middle.
  sample = profile.sample(1.f);
  ASSERT_FLOAT_EQ(500.f, sample.distance);
  ASSERT_FLOAT_EQ(1000.f, sample.velocity);

  // End.
  sample = profile.sample(profile.duration_s());
  ASSERT_FLOAT_EQ(1000.f, sample.distance);
  ASSERT_FLOAT_EQ(0.f, sample.velocity);
}

TEST(TrapezoidProfile, TestMultiplePartsProfile) {
  TrapezoidProfile::Sample sample;
  TrapezoidProfile profile;

  // Accelerate part.

  profile.configure(1000.f, 1000.f, 1000.f, 1000.f);

  // Duration.
  ASSERT_FLOAT_EQ(1.5f, profile.duration_s());

  // End of rise.
  sample = profile.sample(1.f);
  ASSERT_FLOAT_EQ(500.f, sample.distance);
  ASSERT_FLOAT_EQ(1000.f, sample.velocity);

  // End
  sample = profile.sample(profile.duration_s());
  ASSERT_FLOAT_EQ(1000.f, sample.distance);
  ASSERT_FLOAT_EQ(1000.f, sample.velocity);

  // Cruise part.

  profile.configure(1000.f, 1000.f, 1000.f, 1000.f);

  // Duration.
  ASSERT_FLOAT_EQ(1.f, profile.duration_s());

  // Start
  sample = profile.sample(0.0f);
  ASSERT_FLOAT_EQ(0.f, sample.distance);
  ASSERT_FLOAT_EQ(1000.f, sample.velocity);

  // Middle
  sample = profile.sample(0.5f);
  ASSERT_FLOAT_EQ(500.f, sample.distance);
  ASSERT_FLOAT_EQ(1000.f, sample.velocity);

  // Somewhere else.
  sample = profile.sample(0.234f);
  ASSERT_FLOAT_EQ(234.f, sample.distance);
  ASSERT_FLOAT_EQ(1000.f, sample.velocity);

  // End
  sample = profile.sample(profile.duration_s());
  ASSERT_FLOAT_EQ(1000.f, sample.distance);
  ASSERT_FLOAT_EQ(1000.f, sample.velocity);

  // Decelerate part.

  profile.configure(1000.f, 0.f, 1000.f, 1000.f);

  // Duration.
  ASSERT_FLOAT_EQ(1.5f, profile.duration_s());

  // Start of fall.
  sample = profile.sample(0.5f);
  ASSERT_FLOAT_EQ(500.f, sample.distance);
  ASSERT_FLOAT_EQ(1000.f, sample.velocity);

  // End
  sample = profile.sample(profile.duration_s());
  ASSERT_FLOAT_EQ(1000.f, sample.distance);
  ASSERT_FLOAT_EQ(0.f, sample.velocity);
}

TEST(TrapezoidProfile, TestCutoffProfile) {
  TrapezoidProfile profile;

  profile.configure(100.f, 1000.f, 1000.f, 1000.f);

  // Make sure the final velocity is not the same as the one requested, because
  // that was impossible.
  ASSERT_NE(1000.f, profile.final_velocity());

  profile.configure(50.f, 0.f, 1000.f, 1000.f);

  ASSERT_NE(0.f, profile.final_velocity());
}

TEST(TrapezoidProfile, TestStoppedProfile) {
  TrapezoidProfile::Sample sample;
  TrapezoidProfile profile;

  profile.configure(0.f, 0.f, 0.f, 0.f);

  ASSERT_FLOAT_EQ(0.f, profile.duration_s());
  ASSERT_FLOAT_EQ(0.f, profile.final_distance());
  ASSERT_FLOAT_EQ(0.f, profile.final_velocity());

  sample = profile.sample(0.f);

  ASSERT_FLOAT_EQ(0.f, sample.distance);
  ASSERT_FLOAT_EQ(0.f, sample.velocity);

  sample = profile.sample(profile.duration_s());

  ASSERT_FLOAT_EQ(0.f, sample.distance);
  ASSERT_FLOAT_EQ(0.f, sample.velocity);

  sample = profile.sample(694.f);

  ASSERT_FLOAT_EQ(0.f, sample.distance);
  ASSERT_FLOAT_EQ(0.f, sample.velocity);
}

TEST(TrapezoidProfile, TestReset) {
  TrapezoidProfile::Sample sample;
  TrapezoidProfile profile;

  profile.configure(1000.f, 0.f, 1000.f, 1000.f);

  profile.reset();

  profile.configure(1000.f, 0.f, 1000.f, 1000.f);

  sample = profile.sample(0.f);

  ASSERT_FLOAT_EQ(0.f, sample.distance);
  ASSERT_FLOAT_EQ(0.f, sample.velocity);
}

TEST(TrapezoidProfile, TestNegative) {
  TrapezoidProfile::Sample sample;
  TrapezoidProfile profile;

  profile.configure(-180, 0.f, 180.f, 180.f);

  // Duration.
  ASSERT_FLOAT_EQ(2.f, profile.duration_s());

  // Start.
  sample = profile.sample(0.f);
  ASSERT_FLOAT_EQ(0.f, sample.distance);
  ASSERT_FLOAT_EQ(0.f, sample.velocity);

  // Middle.
  sample = profile.sample(1.f);
  ASSERT_FLOAT_EQ(-90.f, sample.distance);
  ASSERT_FLOAT_EQ(-180.f, sample.velocity);

  // End.
  sample = profile.sample(2.f);
  ASSERT_FLOAT_EQ(-180.f, sample.distance);
  ASSERT_FLOAT_EQ(0.f, sample.velocity);
}
