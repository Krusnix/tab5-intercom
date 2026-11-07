#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ESP_IDF

#include "esphome/core/component.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#ifdef USE_TAB5_BROADCAST_SERVER
#include <esp_http_server.h>
#endif

#ifdef USE_SPEAKER
#include "esphome/components/speaker/speaker.h"
#endif

namespace esphome::tab5_broadcast {

#ifdef USE_TAB5_BROADCAST_SERVER
/// Keeps this device's newest recorded clip and serves it as a WAV file
/// (plain http, home network) at /broadcast/<id>.wav.
///
/// Threads: publish_clip() / clear_clip() run on the ESPHome main task; the
/// HTTP server's own task sends the clip. The clip is shared (shared_ptr),
/// so a clip being sent stays alive until it is sent, even if a newer one
/// replaces it meanwhile.
class ClipServer : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  void on_shutdown() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  void set_port(uint16_t port) { this->port_ = port; }

  /// Copies `count` 16-bit mono samples at `sample_rate` into a new clip
  /// (the old one goes) and returns its address on the home network. ""
  /// when there is no memory, no network address yet, or the server isn't
  /// running.
  std::string publish_clip(const int16_t *samples, size_t count, uint32_t sample_rate);
  /// The newest clip's address ("" = none).
  std::string get_clip_url() const;
  /// Takes the clip away (its address answers 404 from then on).
  void clear_clip();
  /// Times the newest clip was sent in full.
  uint32_t get_fetch_count() const { return this->fetches_.load(); }
  bool is_running() const { return this->server_ != nullptr; }

 protected:
  struct Clip {
    uint8_t *data{nullptr};  // 44-byte WAV header + samples, in PSRAM
    size_t size{0};
    uint32_t id{0};
    ~Clip();
  };

  bool start_server_();
  static esp_err_t handle_get_(httpd_req_t *req);

  uint16_t port_{8099};
  httpd_handle_t server_{nullptr};
  uint32_t last_try_{0};
  mutable std::mutex lock_;
  std::shared_ptr<Clip> clip_;
  std::string url_;
  std::atomic<uint32_t> fetches_{0};
};
#endif  // USE_TAB5_BROADCAST_SERVER

#ifdef USE_SPEAKER
/// Passes audio to its output speaker unchanged, but keeps volume and mute
/// to itself: a media player in front of it can't change how loud the
/// hardware speaker plays everything else.
///
/// play() is called from the media source's decode task; the rest from the
/// main task. Its state follows the output speaker's (copied in loop()).
class BroadcastSpeaker : public Component, public speaker::Speaker {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  size_t play(const uint8_t *data, size_t length, TickType_t ticks_to_wait) override;
  size_t play(const uint8_t *data, size_t length) override { return this->play(data, length, 0); }
  void start() override;
  void stop() override;
  void finish() override;
  void set_pause_state(bool pause_state) override { this->output_->set_pause_state(pause_state); }
  bool get_pause_state() const override { return this->output_->get_pause_state(); }
  bool has_buffered_data() const override { return this->output_->has_buffered_data(); }

  // Kept here, never passed on.
  void set_volume(float volume) override { this->volume_ = volume; }
  float get_volume() override { return this->volume_; }
  void set_mute_state(bool mute_state) override { this->mute_state_ = mute_state; }
  bool get_mute_state() override { return this->mute_state_; }

  void set_output_speaker(speaker::Speaker *output) { this->output_ = output; }

  /// Bytes of audio passed to the output since the last reset.
  uint32_t get_played_bytes() const { return this->played_.load(); }
  void reset_played_bytes() { this->played_.store(0); }

 protected:
  void follow_output_state_();

  speaker::Speaker *output_{nullptr};
  std::atomic<uint32_t> played_{0};
};
#endif  // USE_SPEAKER

}  // namespace esphome::tab5_broadcast

#endif  // USE_ESP_IDF
