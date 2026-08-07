#include "spdlog_lyf.h"
#include <chrono>
#include "common.h"

void InitSpdkogLYF(const std::string &log_file, int level) {
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  console_sink->set_level(spdlog::level::warn);
  console_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e]%^[%L]%$: %v");

  auto file_sink =
      std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_file, true);
  file_sink->set_level(spdlog::level::debug);
  // file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e]%^[%L]%$[%s %! %#]: %v");
  file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e]%^[%L]%$: %v");

  std::shared_ptr<spdlog::logger> logger;
  logger.reset(new spdlog::logger("lyf_logger", {console_sink, file_sink}));

  if (level == 2) {
    logger->set_level(spdlog::level::debug);
  } else if (level == 1) {
    logger->set_level(spdlog::level::info);
  } else {
    logger->set_level(spdlog::level::warn);
  }

  spdlog::set_default_logger(logger);
  spdlog::flush_every(std::chrono::seconds(1));

  SPDLOG_WARN("Spdlog init finished...");
}

void SpdlogFlushLYF() { spdlog::get("lyf_logger")->flush(); }
