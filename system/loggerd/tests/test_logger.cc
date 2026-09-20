#include <unistd.h>

#include "catch2/catch.hpp"
#include "system/loggerd/logger.h"

typedef cereal::Sentinel::SentinelType SentinelType;

void verify_segment(const std::string &route_path, int segment, int max_segment, int required_event_cnt) {
  const std::string segment_path = route_path + "--" + std::to_string(segment);
  SentinelType begin_sentinel = segment == 0 ? SentinelType::START_OF_ROUTE : SentinelType::START_OF_SEGMENT;
  SentinelType end_sentinel = segment == max_segment - 1 ? SentinelType::END_OF_ROUTE : SentinelType::END_OF_SEGMENT;

  REQUIRE(!util::file_exists(segment_path + "/rlog.lock"));
  for (const char *fn : {"/rlog.zst", "/qlog.zst"}) {
    const std::string log_file = segment_path + fn;
    std::string log = util::read_file(log_file);
    REQUIRE(!log.empty());
    std::string decompressed_log = zstd_decompress(log);
    int event_cnt = 0, i = 0;
    kj::ArrayPtr<const capnp::word> words((capnp::word *)decompressed_log.data(), decompressed_log.size() / sizeof(capnp::word));
    while (words.size() > 0) {
      try {
        capnp::FlatArrayMessageReader reader(words);
        auto event = reader.getRoot<cereal::Event>();
        words = kj::arrayPtr(reader.getEnd(), words.end());
        if (i == 0) {
          REQUIRE(event.which() == cereal::Event::INIT_DATA);
        } else if (i == 1) {
          REQUIRE(event.which() == cereal::Event::SENTINEL);
          REQUIRE(event.getSentinel().getType() == begin_sentinel);
          REQUIRE(event.getSentinel().getSignal() == 0);
        } else if (words.size() > 0) {
          REQUIRE(event.which() == cereal::Event::CLOCKS);
          ++event_cnt;
        } else {
          // the last event must be SENTINEL
          REQUIRE(event.which() == cereal::Event::SENTINEL);
          REQUIRE(event.getSentinel().getType() == end_sentinel);
          REQUIRE(event.getSentinel().getSignal() == (end_sentinel == SentinelType::END_OF_ROUTE ? 1 : 0));
        }
        ++i;
      } catch (const kj::Exception &ex) {
        INFO("failed parse " << i << " exception :" << ex.getDescription());
        REQUIRE(0);
        break;
      }
    }
    REQUIRE(event_cnt == required_event_cnt);
  }
}

void write_msg(LoggerState *logger) {
  MessageBuilder msg;
  msg.initEvent().initClocks();
  logger->write(msg.toBytes(), true);
}

TEST_CASE("logger") {
  const int segment_cnt = 100;
  // PER-PROCESS log root. This used to be the host-global "/tmp/test_logger", and the first thing
  // this case does is `rm -rf` it -- so any second pytest run on the same machine (the pre-ship gate
  // plus a review agent's copy of the same suite, which is routine here) deleted this run's segments
  // mid-write and the binary aborted inside ZstdFileWriter on `assert(file_ != nullptr)`. Nothing in
  // pytest's per-test isolation covers it: OPENPILOT_PREFIX and PARAMS_ROOT do not reach a hardcoded
  // absolute path. Measured 2026-09-20: run two-up, 5 of 6 runs failed; run alone, 0 of 3.
  const std::string log_root = "/tmp/test_logger_" + std::to_string(getpid());
  REQUIRE(system(("rm " + log_root + " -rf").c_str()) == 0);
  std::string route_name;
  {
    LoggerState logger(log_root);
    route_name = logger.routeName();
    for (int i = 0; i < segment_cnt; ++i) {
      REQUIRE(logger.next());
      REQUIRE(util::file_exists(logger.segmentPath() + "/rlog.lock"));
      REQUIRE(logger.segment() == i);
      write_msg(&logger);
    }
    logger.setExitSignal(1);
  }
  for (int i = 0; i < segment_cnt; ++i) {
    verify_segment(log_root + "/" + route_name, i, segment_cnt, 1);
  }
  // The root is per-process now, so the `rm -rf` at the top no longer reclaims the previous run's
  // tree: clean up here or /tmp grows one 100-segment directory per test run.
  REQUIRE(system(("rm " + log_root + " -rf").c_str()) == 0);
}
