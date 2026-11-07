#pragma once

#include "esphome/core/defines.h"

#ifdef USE_TAB5_SDCARD_MEDIA_SOURCE

#include "esphome/components/audio/audio.h"
#include "esphome/components/media_source/media_source.h"
#include "esphome/core/component.h"

#include "tab5_sdcard.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <atomic>
#include <mutex>
#include <string>

namespace esphome::tab5_sdcard {

/// Plays PCM WAV files from the SD card: "sdcard://folder/file.wav".
///
/// play_uri() hands the file to the source's own task, which reads it and
/// passes the audio on; the task's news (started, finished, failed) turns
/// into media source states in loop(), on the main task.
class SdFileMediaSource : public Component, public media_source::MediaSource {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_card(SdCard *card) { this->card_ = card; }

  bool play_uri(const std::string &uri) override;
  void handle_command(media_source::MediaSourceCommand command) override;
  bool can_handle(const std::string &uri) const override;

 protected:
  static void task_trampoline_(void *arg);
  void run_task_();
  /// Plays one file (on the task). "" when it played to the end or was
  /// stopped, otherwise what went wrong.
  std::string play_file_(const std::string &path);
  void take_news_();

  SdCard *card_{nullptr};
  TaskHandle_t task_{nullptr};
  std::mutex lock_;
  std::string path_;  // under lock_

  std::atomic<bool> active_{false};   // the task has a file
  std::atomic<bool> stop_{false};
  std::atomic<bool> pause_{false};
  std::atomic<bool> started_{false};  // news: the file opened
  std::atomic<uint8_t> ended_{0};     // news: 1 finished or stopped, 2 failed
};

}  // namespace esphome::tab5_sdcard

#endif  // USE_TAB5_SDCARD_MEDIA_SOURCE
