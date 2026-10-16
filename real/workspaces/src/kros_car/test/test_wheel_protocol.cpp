#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "kros_car/wheel_protocol.hpp"

namespace wheel = kros_car::wheel;

namespace
{
const char kFrame[] =
  "{\"T\":20012,\"ms\":123456,\"ok\":15,\"mode\":[2,2,2,2],\"spd\":[50,49,-50,-51],"
  "\"pos\":[1234,22222,333,32767],\"tor\":[120,118,-121,-32767],\"err\":[0,0,8,0]}";
}  // namespace

TEST(WheelProtocol, BuildsCommands)
{
  EXPECT_EQ(
    wheel::build_command_all({100, -100, 0, 7}, 1),
    "{\"T\":10011,\"cmd\":[100,-100,0,7],\"act\":1}\n");
  EXPECT_EQ(wheel::build_command_all({}, 0), "{\"T\":10011,\"cmd\":[0,0,0,0],\"act\":0}\n");
  EXPECT_EQ(wheel::build_heartbeat(500), "{\"T\":11001,\"time\":500}\n");
  EXPECT_EQ(wheel::build_heartbeat(-1), "{\"T\":11001,\"time\":-1}\n");
  EXPECT_EQ(wheel::build_change_mode(3, 2), "{\"T\":10012,\"id\":3,\"mode\":2}\n");
}

TEST(WheelProtocol, ParsesFrame)
{
  wheel::Frame frame;
  // println() ends the line with CR LF; the LF is the delimiter, the CR stays.
  ASSERT_TRUE(wheel::parse_frame(std::string(kFrame) + "\r", frame));
  EXPECT_EQ(frame.esp_ms, 123456u);
  EXPECT_EQ(frame.ok_mask, 15u);
  EXPECT_TRUE(frame.ok(0));
  EXPECT_TRUE(frame.ok(3));
  EXPECT_EQ(frame.mode[0], 2);
  EXPECT_EQ(frame.spd_rpm[1], 49);
  EXPECT_EQ(frame.spd_rpm[3], -51);
  EXPECT_EQ(frame.pos_raw[3], 32767);
  EXPECT_EQ(frame.tor_raw[2], -121);
  EXPECT_EQ(frame.tor_raw[3], -32767);
  EXPECT_EQ(frame.err[2], 8);
}

TEST(WheelProtocol, ReportsMissingWheels)
{
  std::string line = kFrame;
  line.replace(line.find("\"ok\":15"), 7, "\"ok\":5");
  wheel::Frame frame;
  ASSERT_TRUE(wheel::parse_frame(line, frame));
  EXPECT_TRUE(frame.ok(0));
  EXPECT_FALSE(frame.ok(1));
  EXPECT_TRUE(frame.ok(2));
  EXPECT_FALSE(frame.ok(3));
}

TEST(WheelProtocol, RejectsEverythingElse)
{
  wheel::Frame frame;
  frame.esp_ms = 7;

  // Other firmware messages.
  EXPECT_FALSE(wheel::parse_frame("{\"T\":20010,\"id\":1,\"mode\":2,\"tor\":0,\"spd\":50,\"pos\":1234,\"err\":0}", frame));
  EXPECT_FALSE(wheel::parse_frame("{\"T\":20011,\"hb\":0}", frame));
  // Noise, empty and truncated lines.
  EXPECT_FALSE(wheel::parse_frame("", frame));
  EXPECT_FALSE(wheel::parse_frame("ets Jul 29 2019 12:21:46", frame));
  EXPECT_FALSE(wheel::parse_frame(std::string(kFrame).substr(0, 60), frame));
  EXPECT_FALSE(wheel::parse_frame(std::string(kFrame).substr(10), frame));

  // Out-of-range or malformed values.
  std::string line = kFrame;
  line.replace(line.find("32767]"), 5, "40000");
  EXPECT_FALSE(wheel::parse_frame(line, frame));
  line = kFrame;
  line.replace(line.find("\"ok\":15"), 7, "\"ok\":16");
  EXPECT_FALSE(wheel::parse_frame(line, frame));
  line = kFrame;
  line.replace(line.find("[50,49,-50,-51]"), 15, "[50,49,-50]");
  EXPECT_FALSE(wheel::parse_frame(line, frame));
  line = kFrame;
  line.replace(line.find("[50,49,-50,-51]"), 15, "[50,49,x,-51]");
  EXPECT_FALSE(wheel::parse_frame(line, frame));

  // A rejected line must not touch the output.
  EXPECT_EQ(frame.esp_ms, 7u);
}

TEST(WheelProtocol, DetectsHeartbeatStop)
{
  EXPECT_TRUE(wheel::is_heartbeat_stop("{\"T\":20011,\"hb\":0}\r"));
  EXPECT_FALSE(wheel::is_heartbeat_stop(kFrame));
}

TEST(WheelProtocol, ConvertsUnits)
{
  EXPECT_NEAR(wheel::rpm_to_rad_per_s(60.0), 2.0 * M_PI, 1e-12);
  EXPECT_NEAR(wheel::rpm_to_rad_per_s(-100.0), -10.471975511965978, 1e-9);
  EXPECT_NEAR(wheel::raw_to_ampere(32767), 8.0, 1e-12);
  EXPECT_NEAR(wheel::raw_to_ampere(-32767), -8.0, 1e-12);
  // The datasheet's own example: 2000 counts.
  EXPECT_NEAR(wheel::raw_to_ampere(2000), 0.48829, 1e-5);
  EXPECT_NEAR(wheel::counts_to_rad(wheel::kPositionModulus), 2.0 * M_PI, 1e-12);
  EXPECT_NEAR(wheel::counts_to_rad(-wheel::kPositionModulus / 2), -M_PI, 1e-12);
}

TEST(WheelProtocol, DescribesErrors)
{
  EXPECT_EQ(wheel::describe_error(0x02), "overcurrent");
  EXPECT_EQ(wheel::describe_error(0x09), "sensor,stall");
  EXPECT_EQ(wheel::describe_error(0x80), "unknown");
}

TEST(PositionUnwrapper, StartsAtZeroAndFollowsBothDirections)
{
  wheel::PositionUnwrapper unwrapper;
  EXPECT_EQ(unwrapper.update(30000), 0);
  EXPECT_EQ(unwrapper.update(30500), 500);
  EXPECT_EQ(unwrapper.update(30100), 100);
  EXPECT_EQ(unwrapper.update(29000), -1000);
}

TEST(PositionUnwrapper, CrossesTheWrapForwardAndBackward)
{
  wheel::PositionUnwrapper unwrapper;
  unwrapper.update(32000);
  // 32000 -> 200 is +968 through the wrap, not -31800.
  EXPECT_EQ(unwrapper.update(200), 968);
  EXPECT_EQ(unwrapper.update(32700), 700);
  EXPECT_EQ(unwrapper.update(31000), -1000);
}

TEST(PositionUnwrapper, AccumulatesManyTurns)
{
  wheel::PositionUnwrapper unwrapper;
  int raw = 0;
  unwrapper.update(raw);
  long long counts = 0;
  // 100 rpm sampled at 50 Hz is 1/30 turn per sample.
  const int step = wheel::kPositionModulus / 30;
  for (int i = 0; i < 3000; ++i) {
    raw = (raw + step) % wheel::kPositionModulus;
    counts = unwrapper.update(raw);
  }
  EXPECT_EQ(counts, 3000LL * step);

  for (int i = 0; i < 6000; ++i) {
    raw = ((raw - step) % wheel::kPositionModulus + wheel::kPositionModulus) %
      wheel::kPositionModulus;
    counts = unwrapper.update(raw);
  }
  EXPECT_EQ(counts, -3000LL * step);

  unwrapper.reset();
  EXPECT_EQ(unwrapper.update(12345), 0);
}

TEST(ClockMapper, RemovesReceiveJitter)
{
  wheel::ClockMapper mapper;
  const double host0 = 1.7e9;
  const double min_delay = 0.003;
  // Frames sent every 20 ms; every third one arrives 8 ms late.
  double previous = 0.0;
  for (int i = 0; i < 500; ++i) {
    const uint32_t esp_ms = 1000u + 20u * static_cast<uint32_t>(i);
    const double sent = host0 + 0.020 * i;
    const double jitter = (i % 3 == 2) ? 0.008 : 0.0;
    const double stamp = mapper.update(esp_ms, sent + min_delay + jitter);

    EXPECT_LE(stamp, sent + min_delay + jitter + 1e-9);
    if (i > 0) {
      EXPECT_NEAR(stamp - previous, 0.020, 1e-4) << "frame " << i;
      EXPECT_NEAR(stamp, sent + min_delay, 2e-4) << "frame " << i;
    }
    previous = stamp;
  }
}

TEST(ClockMapper, FollowsClockDriftInBothDirections)
{
  for (const double drift : {+50e-6, -50e-6}) {
    wheel::ClockMapper mapper;
    const double host0 = 1.7e9;
    double worst = 0.0;
    // 10 minutes; the ESP32 clock runs `drift` fast or slow.
    for (int i = 0; i < 30000; ++i) {
      const uint32_t esp_ms = 20u * static_cast<uint32_t>(i);
      const double sent = host0 + 0.020 * i / (1.0 + drift);
      const double stamp = mapper.update(esp_ms, sent + 0.003);
      worst = std::max(worst, std::abs(stamp - (sent + 0.003)));
    }
    EXPECT_LT(worst, 1e-3) << "drift " << drift;
  }
}

TEST(ClockMapper, RestartsAfterEspRebootOrGap)
{
  wheel::ClockMapper mapper;
  const double host0 = 1.7e9;
  mapper.update(500000u, host0);
  mapper.update(500020u, host0 + 0.020);

  // ESP32 rebooted: its counter starts over.
  EXPECT_DOUBLE_EQ(mapper.update(40u, host0 + 0.9), host0 + 0.9);
  EXPECT_NEAR(mapper.update(60u, host0 + 0.92), host0 + 0.92, 1e-6);

  // Link was down for a while.
  EXPECT_DOUBLE_EQ(mapper.update(5060u, host0 + 6.0), host0 + 6.0);

  // millis() wrapping is not a reboot.
  wheel::ClockMapper wrapping;
  wrapping.update(0xFFFFFFF0u, host0);
  EXPECT_NEAR(wrapping.update(0x00000004u, host0 + 0.020), host0 + 0.020, 1e-6);
  EXPECT_NEAR(wrapping.update(0x00000018u, host0 + 0.040), host0 + 0.040, 1e-6);
}
