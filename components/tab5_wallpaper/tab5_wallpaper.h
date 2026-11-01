#pragma once

#include "esphome/core/defines.h"

#if defined(USE_ESP_IDF) && defined(USE_ESP32_VARIANT_ESP32P4)

#include "esphome/components/mipi_dsi/mipi_dsi.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"

#include "driver/jpeg_decode.h"
#include "driver/ppa.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

namespace esphome::tab5_wallpaper {

/// A moving wallpaper behind the lock screen of a plain ESPHome mipi_dsi
/// display (no LVGL).
///
/// The wallpaper is a looping clip: one file of JPEG pictures one after the
/// other (an "MJPEG" file, as ffmpeg writes with -f mjpeg). Each picture is
/// either 720 x 1280 (already turned for the Tab5's panel, the quickest) or
/// 1280 x 720 (turned by the PPA).
///
/// How a picture reaches the screen:
///  - the display lambda draws the lock screen as usual, on black, into the
///    display's own buffer. lock_screen_drawn() copies what it drew into
///    this component's copy of the lock screen and stops the display from
///    sending it to the panel itself;
///  - this component's task reads the file, decodes each picture with the
///    hardware JPEG decoder, and lays the lock screen copy over it with the
///    PPA (black counts as see-through), straight into the panel's frame
///    buffer. A change on the lock screen (the slider moving) goes out at
///    once, without waiting for the next picture.
///
/// Threads: everything public runs on the ESPHome main task (the same task
/// as the display lambda). The task only touches the screen while the lock
/// screen is up, and lock_screen_gone() waits for it to finish, so the
/// display's own drawing and this component never write the panel at the
/// same time.
class Tab5Wallpaper : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  void on_shutdown() override;
  // After the display (it needs the panel set up).
  float get_setup_priority() const override { return setup_priority::LATE; }

  void set_display(mipi_dsi::MipiDsi *display) { this->display_ = display; }
  void set_file(const std::string &path) { this->path_ = path; }
  void set_framerate(uint8_t fps) { this->default_fps_ = fps; }
  void set_max_picture_bytes(uint32_t bytes) { this->max_picture_bytes_ = bytes; }

  // ---- From the display lambda ----

  /// Call at the end of every drawing of the lock screen (also partial
  /// ones). Starts the wallpaper, and while it plays, keeps the display from
  /// sending the lock screen to the panel itself.
  void lock_screen_drawn();
  /// Call when anything else is drawn. Stops the wallpaper; returns once it
  /// no longer writes to the panel.
  void lock_screen_gone();

  // ---- Settings ----

  /// On (default) or off. Turning it off while it plays draws the screen
  /// again (plain black lock screen).
  void set_enabled(bool enabled);
  bool is_enabled() const { return this->enabled_.load(); }

  // ---- Changing the file (for example downloading a new one) ----

  /// Closes the file and keeps it closed until reload() (the last picture
  /// stays). Needed before the file is replaced. Blocks for at most one
  /// picture's reading.
  void close_file();
  /// Opens the file again from the start (a new file, or a new framerate).
  void reload();
  /// Words shown instead of the state until reload() ("" = none), for
  /// example how a download is going.
  void set_note(const std::string &note);

  // ---- State ----

  /// True while pictures go to the screen (the lock screen is up and the
  /// wallpaper plays).
  bool is_playing() const { return this->active_.load() && this->showing_(); }
  /// One short sentence for Home Assistant: "Playing: 160 pictures, 20 a
  /// second", "No wallpaper on the SD card", ...
  std::string get_status() const;
  /// Pictures shown since start-up.
  uint32_t get_frames_shown() const { return this->frames_shown_.load(); }

  /// Fires on the main task when get_status() changed.
  Trigger<> *get_change_trigger() { return &this->change_trigger_; }

 protected:
  // Parts of the task
  static void task_trampoline_(void *arg);
  void run_task_();
  void wait_until_(int64_t until_us);
  void serve_ui_();
  bool prepare_();
  int next_picture_();
  bool open_file_();
  void close_file_locked_();
  bool feed_();
  void picture_done_();
  bool append_(const uint8_t *data, size_t n);
  void segment_done_();
  bool decode_();
  bool compose_(int x, int y, int w, int h);
  void set_status_(const std::string &status);
  bool showing_() const {
    return this->bg_valid_.load() && this->ui_valid_.load() && !this->broken_.load() && this->fb_ != nullptr;
  }
  // Main task
  bool copy_ui_(int x0, int y0, int x1, int y1);
  void request_compose_(int x0, int y0, int x1, int y1);

  mipi_dsi::MipiDsi *display_{nullptr};
  std::string path_{"/sdcard/wallpaper/wallpaper.mjpeg"};
  uint8_t default_fps_{20};
  uint32_t max_picture_bytes_{800 * 1000};

  // Panel (native, unrotated) size and frame buffer
  int panel_w_{720}, panel_h_{1280};
  uint8_t *fb_{nullptr};
  size_t fb_bytes_{0};

  // Hardware
  jpeg_decoder_handle_t decoder_{nullptr};
  jpeg_decode_cfg_t decode_cfg_{};
  ppa_client_handle_t blend_{nullptr};
  ppa_client_handle_t srm_{nullptr};

  // Buffers (PSRAM, made on first use and kept)
  uint8_t *bg_{nullptr};       // the picture, panel size, RGB565
  size_t bg_cap_{0};
  uint8_t *turn_{nullptr};     // a 1280 x 720 picture before turning
  size_t turn_cap_{0};
  uint8_t *ui_{nullptr};       // copy of the lock screen, panel size
  uint8_t *picture_{nullptr};  // one JPEG picture
  size_t picture_cap_{0};
  uint8_t *chunk_{nullptr};    // file reads
  size_t chunk_len_{0}, chunk_pos_{0};

  // File (task, under file_lock_)
  std::mutex file_lock_;
  int fd_{-1};
  bool file_closed_{false};    // close_file() until reload()
  bool want_reopen_{false};
  uint32_t pass_pictures_{0};  // pictures found since the start of the file
  uint32_t file_pictures_{0};  // pictures in the whole file (after one pass)
  uint32_t empty_passes_{0};
  int64_t next_open_us_{0};

  // JPEG splitter (task)
  enum ParseState : uint8_t { P_SOI, P_MARKER, P_LENGTH, P_SEGMENT, P_ENTROPY, P_ENTROPY_FF };
  ParseState pstate_{P_SOI};
  bool prev_ff_{false};
  bool marker_ff_{false};
  uint8_t marker_{0};
  uint8_t len_got_{0};
  uint32_t seg_len_{0}, seg_left_{0};
  size_t seg_start_{0};
  size_t picture_len_{0};
  bool overflow_{false};
  bool picture_ready_{false};
  uint8_t file_fps_{0};        // from a "fps=NN" comment in the pictures

  // Lock screen copy (main task writes, task reads; under ui_mutex_)
  SemaphoreHandle_t ui_mutex_{nullptr};
  StaticSemaphore_t ui_mutex_storage_{};
  int dirty_x0_{0}, dirty_y0_{0}, dirty_x1_{-1}, dirty_y1_{-1};  // to send (inclusive)
  std::atomic<bool> ui_pending_{false};
  std::atomic<bool> ui_valid_{false};
  std::atomic<bool> want_ui_{false};  // task: a picture is ready, copy the lock screen

  // State
  TaskHandle_t task_{nullptr};
  SemaphoreHandle_t task_done_{nullptr};
  std::atomic<bool> running_{false};
  std::atomic<bool> enabled_{true};
  std::atomic<bool> active_{false};
  std::atomic<bool> bg_valid_{false};
  std::atomic<bool> broken_{false};      // the screen can't be written: off until restart
  std::atomic<bool> need_redraw_{false};  // main task: draw the screen again
  bool prepared_{false};
  bool out_of_memory_{false};
  uint32_t bad_pictures_{0};             // in a row
  std::atomic<uint32_t> frames_shown_{0};

  // Timing (task), logged every 10 s while playing
  uint32_t stat_frames_{0}, stat_late_{0};
  int64_t stat_start_us_{0}, stat_read_us_{0}, stat_decode_us_{0}, stat_blend_us_{0};

  // Status (any task, under status_lock_)
  mutable std::mutex status_lock_;
  std::string status_{"Waiting for the lock screen"};
  std::string note_;
  std::atomic<uint32_t> status_version_{0};
  uint32_t status_seen_{0};
  Trigger<> change_trigger_;
};

}  // namespace esphome::tab5_wallpaper

#endif
