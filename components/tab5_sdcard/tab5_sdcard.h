#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ESP_IDF

#include "esphome/core/automation.h"
#include "esphome/core/component.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>

#include <sd_protocol_types.h>
#include <sd_pwr_ctrl.h>

namespace esphome::tab5_sdcard {

/// The Tab5's microSD card (SD host slot 0, next to the Wi-Fi chip on slot 1).
///
/// Threads: everything public runs on the ESPHome main task, except where
/// it says otherwise. Slow work runs on the card's own task ("worker"); its
/// results are read back under lock_, and on_change fires from loop().
class SdCard : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  // Before Wi-Fi connects: setting up the slot changes the SD host's clock,
  // which the Wi-Fi chip shares.
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  void set_mount_point(const std::string &mount_point) { this->mount_point_ = mount_point; }
  void set_ldo_channel(int channel) { this->ldo_channel_ = channel; }
  void set_max_freq_khz(uint32_t khz) { this->max_freq_khz_ = khz; }
  void set_max_files(int max_files) { this->max_files_ = max_files; }

  /// The card is mounted and its files can be used.
  bool is_mounted() const { return this->mounted_.load(); }
  /// "/sdcard"
  const std::string &get_mount_point() const { return this->mount_point_; }
  /// mount_point + "/" + relative path
  std::string path(const std::string &relative) const;

  /// One short sentence: "Ready", "No card found", ...
  std::string get_status() const;
  /// "SDHC 31.9 GB (SD32G), 40 MHz" ("" when not mounted).
  std::string get_card_info() const;
  /// Card size and free space in GB (NAN until known).
  float get_size_gb() const;
  float get_free_gb() const;
  /// The last read/write test: "" (none yet), "Testing...", "OK: ..." or
  /// "Failed: ...".
  std::string get_test_result() const;
  /// The card's task is working on something.
  bool is_busy() const { return this->busy_.load() || this->pending_.load() > 0; }

  /// Mounts the card if it isn't (blocks for up to about a second), then
  /// refreshes the free space and runs the read/write test (4 MB written,
  /// read back and deleted) on the card's task. Not during a call: setting
  /// up the slot changes the SD host's clock, which Wi-Fi shares.
  void check();
  /// Free space again (on the card's task), for example after saving files.
  void refresh_space();

  /// Runs `job` on the card's task (after the jobs before it). From any task.
  /// false if the task isn't running.
  bool run_on_card_task(std::function<void()> &&job);
  /// Asks for on_change to fire on the next loop(). From any task.
  void notify_change() { this->changed_.store(true); }
  /// A file operation failed in a way that points at the card itself (gone,
  /// or not answering): marks it unmounted until check() finds it again.
  /// From any task.
  void report_card_error(const char *what);

  Trigger<> *get_change_trigger() { return &this->change_trigger_; }

 protected:
  bool mount_();
  void unmount_();
  void set_status_(const std::string &status);
  void update_space_();  // card task
  void run_test_();      // card task
  static void worker_trampoline_(void *arg);
  void worker_();

  std::string mount_point_{"/sdcard"};
  int ldo_channel_{4};
  uint32_t max_freq_khz_{40000};
  int max_files_{4};

  // Set up once, never taken down (the SD host is shared with Wi-Fi).
  bool slot_ready_{false};
  sd_pwr_ctrl_handle_t pwr_{nullptr};
  sdmmc_card_t *card_{nullptr};
  void *fs_{nullptr};  // FATFS (ff.h)
  uint8_t pdrv_{0xFF};
  std::atomic<bool> mounted_{false};

  mutable std::mutex lock_;
  std::string status_{"Starting"};
  std::string card_info_;
  std::string test_result_;
  uint64_t total_bytes_{0};
  uint64_t free_bytes_{0};
  bool space_known_{false};

  TaskHandle_t task_{nullptr};
  std::mutex jobs_lock_;
  std::deque<std::function<void()>> jobs_;
  std::atomic<int> pending_{0};
  std::atomic<bool> busy_{false};
  std::atomic<bool> changed_{false};
  Trigger<> change_trigger_;
};

}  // namespace esphome::tab5_sdcard

#endif  // USE_ESP_IDF
