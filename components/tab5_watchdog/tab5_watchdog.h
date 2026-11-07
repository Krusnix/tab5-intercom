#pragma once

#include "esphome/core/defines.h"

#if defined(USE_ESP_IDF)

#include "esphome/core/component.h"

#include <cstdint>
#include <string>

namespace esphome::tab5_watchdog {

/// Keeps the ESPHome main loop from being frozen out by another task, and
/// says what was going on when the task watchdog restarts the chip anyway
/// (see __init__.py).
class Tab5Watchdog : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  // Early, so the main loop task is known before anything can freeze it.
  float get_setup_priority() const override { return setup_priority::BUS; }

  /// One line for a text sensor (at most 250 characters): the last freeze
  /// since power-up, and the last time the main loop had to be rescued.
  std::string get_report() const;

 protected:
  void log_freeze_();
  void log_rescues_();

  // The freeze before the last restart ("" = none).
  std::string freeze_;
  std::string freeze_busy_;
  // The main loop's code addresses at the freeze, return addresses only.
  uint32_t trace_[16]{};
  uint8_t trace_len_{0};
  uint32_t last_log_ms_{0};
  uint8_t times_logged_{0};
  // Rescues seen so far (the newest is described in rescue_).
  uint32_t rescues_seen_{0};
  bool restore_logged_{true};
  std::string rescue_;
};

}  // namespace esphome::tab5_watchdog

#endif
