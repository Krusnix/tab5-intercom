#include "tab5_wallpaper.h"

#if defined(USE_ESP_IDF) && defined(USE_ESP32_VARIANT_ESP32P4)

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include "esp_heap_caps.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_timer.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace esphome::tab5_wallpaper {

static const char *const TAG = "tab5_wallpaper";

namespace {

// The task: below the audio, network and call video tasks; stack in PSRAM
// (it never writes to flash).
constexpr uint32_t kTaskStackBytes = 8192;
constexpr UBaseType_t kTaskPriority = 3;
// File reads. The SD host is shared with the Wi-Fi chip; 32 KB pieces keep
// each of the card's turns short.
constexpr size_t kChunkBytes = 32 * 1024;
// Pictures in a row that can't be shown before the file counts as broken.
constexpr uint32_t kMaxBadPictures = 8;
// Looking again for a missing or broken file.
constexpr int64_t kRetryUs = 5LL * 1000 * 1000;
// Timing written to the log this often while playing.
constexpr int64_t kStatsUs = 10LL * 1000 * 1000;

// ---- Reaching into ESPHome's mipi_dsi display ----
// The display keeps its draw buffer, its panel and the area its lambda
// changed to itself. Explicit template instantiation may name protected
// members, so these read them anyway; if ESPHome renames one, the build
// fails instead of the screen.
template<typename Tag, typename Tag::type M> struct Reach {
  friend typename Tag::type member(Tag) { return M; }
};
struct DsiBuffer {
  using type = uint8_t *mipi_dsi::MipiDsi::*;
  friend type member(DsiBuffer);
};
struct DsiPanel {
  using type = esp_lcd_panel_handle_t mipi_dsi::MipiDsi::*;
  friend type member(DsiPanel);
};
struct DsiDepth {
  using type = display::ColorBitness mipi_dsi::MipiDsi::*;
  friend type member(DsiDepth);
};
struct DsiXLow {
  using type = uint16_t mipi_dsi::MipiDsi::*;
  friend type member(DsiXLow);
};
struct DsiYLow {
  using type = uint16_t mipi_dsi::MipiDsi::*;
  friend type member(DsiYLow);
};
struct DsiXHigh {
  using type = uint16_t mipi_dsi::MipiDsi::*;
  friend type member(DsiXHigh);
};
struct DsiYHigh {
  using type = uint16_t mipi_dsi::MipiDsi::*;
  friend type member(DsiYHigh);
};
template struct Reach<DsiBuffer, &mipi_dsi::MipiDsi::buffer_>;
template struct Reach<DsiPanel, &mipi_dsi::MipiDsi::handle_>;
template struct Reach<DsiDepth, &mipi_dsi::MipiDsi::color_depth_>;
template struct Reach<DsiXLow, &mipi_dsi::MipiDsi::x_low_>;
template struct Reach<DsiYLow, &mipi_dsi::MipiDsi::y_low_>;
template struct Reach<DsiXHigh, &mipi_dsi::MipiDsi::x_high_>;
template struct Reach<DsiYHigh, &mipi_dsi::MipiDsi::y_high_>;

}  // namespace

// ---------------------------------------------------------------------------
// Set-up

void Tab5Wallpaper::setup() {
  if (this->display_ == nullptr) {
    ESP_LOGE(TAG, "No display");
    this->mark_failed();
    return;
  }
  if (this->display_->*member(DsiDepth{}) != display::COLOR_BITNESS_565) {
    ESP_LOGE(TAG, "The display must use 16-bit colour (color_depth: 16)");
    this->mark_failed();
    return;
  }
  this->panel_w_ = this->display_->get_native_width();
  this->panel_h_ = this->display_->get_native_height();

  // The panel's frame buffer: pictures go straight into it.
  esp_lcd_panel_handle_t panel = this->display_->*member(DsiPanel{});
  void *fb = nullptr;
  if (panel == nullptr || esp_lcd_dpi_panel_get_frame_buffer(panel, 1, &fb) != ESP_OK || fb == nullptr) {
    ESP_LOGE(TAG, "The display's frame buffer isn't there (display not set up?)");
    this->mark_failed();
    return;
  }
  this->fb_ = static_cast<uint8_t *>(fb);
  this->fb_bytes_ = static_cast<size_t>(this->panel_w_) * this->panel_h_ * 2U;
  if ((reinterpret_cast<uintptr_t>(this->fb_) & 63U) != 0 || (this->fb_bytes_ & 63U) != 0) {
    ESP_LOGE(TAG, "The display's frame buffer isn't aligned for the PPA");
    this->fb_ = nullptr;
    this->mark_failed();
    return;
  }

  // Hardware JPEG decoder (shared with call video and the camera; ESP-IDF
  // takes turns picture by picture).
  jpeg_decode_engine_cfg_t engine_config{};
  engine_config.timeout_ms = 200;
  if (jpeg_new_decoder_engine(&engine_config, &this->decoder_) != ESP_OK || this->decoder_ == nullptr) {
    ESP_LOGE(TAG, "JPEG decoder not available");
    this->decoder_ = nullptr;
    this->mark_failed();
    return;
  }
  this->decode_cfg_.output_format = JPEG_DECODE_OUT_FORMAT_RGB565;
  // ESP-IDF's "BGR" is the normal little-endian RGB565 of the display buffer.
  this->decode_cfg_.rgb_order = JPEG_DEC_RGB_ELEMENT_ORDER_BGR;
  this->decode_cfg_.conv_std = JPEG_YUV_RGB_CONV_STD_BT601;

  ppa_client_config_t blend_config{};
  blend_config.oper_type = PPA_OPERATION_BLEND;
  if (ppa_register_client(&blend_config, &this->blend_) != ESP_OK || this->blend_ == nullptr) {
    ESP_LOGE(TAG, "PPA not available");
    this->blend_ = nullptr;
    this->mark_failed();
    return;
  }

  this->ui_mutex_ = xSemaphoreCreateMutexStatic(&this->ui_mutex_storage_);
  this->task_done_ = xSemaphoreCreateBinary();
  this->running_.store(true);
  if (this->task_done_ == nullptr ||
      xTaskCreatePinnedToCoreWithCaps(Tab5Wallpaper::task_trampoline_, "wallpaper", kTaskStackBytes, this,
                                      kTaskPriority, &this->task_, tskNO_AFFINITY,
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
    this->running_.store(false);
    this->task_ = nullptr;
    ESP_LOGE(TAG, "Could not start the wallpaper task");
    this->mark_failed();
    return;
  }
  this->set_status_(this->enabled_.load() ? "Waiting for the lock screen" : "Off");
}

void Tab5Wallpaper::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Tab5 live wallpaper:\n"
                "  File: %s\n"
                "  Pictures a second: %u (unless the file says)\n"
                "  Biggest picture: %u KB",
                this->path_.c_str(), this->default_fps_, (unsigned) (this->max_picture_bytes_ / 1024));
  if (this->is_failed()) {
    ESP_LOGCONFIG(TAG, "  FAILED to start (see the log above)");
  }
}

void Tab5Wallpaper::on_shutdown() {
  if (this->task_ != nullptr && this->running_.exchange(false)) {
    this->active_.store(false);
    xTaskNotifyGive(this->task_);
    xSemaphoreTake(this->task_done_, pdMS_TO_TICKS(500));
  }
}

// ---------------------------------------------------------------------------
// Main task

void Tab5Wallpaper::loop() {
  // The task has a picture: copy the lock screen (once, from the display's
  // buffer, which holds it while the lock screen is up).
  if (this->want_ui_.exchange(false) && this->active_.load() && this->enabled_.load() && !this->ui_valid_.load()) {
    if (this->ui_ == nullptr) {
      this->ui_ = static_cast<uint8_t *>(heap_caps_aligned_calloc(
          128, 1, this->fb_bytes_, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
      if (this->ui_ == nullptr) {
        ESP_LOGE(TAG, "Not enough PSRAM for the lock screen copy (%u KB)", (unsigned) (this->fb_bytes_ / 1024));
        this->set_status_("Not enough memory");
      }
    }
    if (this->ui_ != nullptr && this->copy_ui_(0, 0, this->panel_w_ - 1, this->panel_h_ - 1)) {
      this->ui_valid_.store(true);
      xTaskNotifyGive(this->task_);
    }
  }
  if (this->need_redraw_.exchange(false))
    this->display_->update();
  const uint32_t version = this->status_version_.load();
  if (version != this->status_seen_) {
    this->status_seen_ = version;
    this->change_trigger_.trigger();
  }
}

void Tab5Wallpaper::lock_screen_drawn() {
  if (this->is_failed() || !this->enabled_.load() || this->broken_.load())
    return;
  if (!this->active_.load()) {
    this->active_.store(true);
    xTaskNotifyGive(this->task_);
  }
  uint16_t &x_low = this->display_->*member(DsiXLow{});
  uint16_t &y_low = this->display_->*member(DsiYLow{});
  uint16_t &x_high = this->display_->*member(DsiXHigh{});
  uint16_t &y_high = this->display_->*member(DsiYHigh{});
  if (this->ui_ != nullptr) {
    if (!this->ui_valid_.load()) {
      // First drawing of this lock screen: take all of it.
      if (this->copy_ui_(0, 0, this->panel_w_ - 1, this->panel_h_ - 1))
        this->ui_valid_.store(true);
    } else if (x_low <= x_high && y_low <= y_high) {
      this->copy_ui_(x_low, y_low, std::min<int>(x_high, this->panel_w_ - 1), std::min<int>(y_high, this->panel_h_ - 1));
    }
  }
  if (this->showing_()) {
    // The task puts it on the screen, over the wallpaper; the display must
    // not send it on black.
    x_low = this->panel_w_;
    y_low = this->panel_h_;
    x_high = 0;
    y_high = 0;
  }
}

void Tab5Wallpaper::lock_screen_gone() {
  if (!this->active_.load()) {
    this->ui_valid_.store(false);
    return;
  }
  // Waits for a picture the task may be putting on the screen right now.
  xSemaphoreTake(this->ui_mutex_, portMAX_DELAY);
  this->active_.store(false);
  this->ui_valid_.store(false);
  this->dirty_x1_ = -1;
  this->dirty_y1_ = -1;
  xSemaphoreGive(this->ui_mutex_);
}

bool Tab5Wallpaper::copy_ui_(int x0, int y0, int x1, int y1) {
  uint8_t *src = this->display_->*member(DsiBuffer{});
  if (src == nullptr || this->ui_ == nullptr)
    return false;
  x0 = std::max(0, x0);
  y0 = std::max(0, y0);
  x1 = std::min(x1, this->panel_w_ - 1);
  y1 = std::min(y1, this->panel_h_ - 1);
  if (x0 > x1 || y0 > y1)
    return false;
  const size_t stride = static_cast<size_t>(this->panel_w_) * 2U;
  xSemaphoreTake(this->ui_mutex_, portMAX_DELAY);
  if (x0 == 0 && x1 == this->panel_w_ - 1) {
    memcpy(this->ui_ + y0 * stride, src + y0 * stride, (y1 - y0 + 1) * stride);
  } else {
    const size_t n = static_cast<size_t>(x1 - x0 + 1) * 2U;
    for (int y = y0; y <= y1; y++)
      memcpy(this->ui_ + y * stride + x0 * 2, src + y * stride + x0 * 2, n);
  }
  // Send it at the next chance (with the area changed since the last time).
  if (this->dirty_x1_ < this->dirty_x0_ || this->dirty_y1_ < this->dirty_y0_) {
    this->dirty_x0_ = x0;
    this->dirty_y0_ = y0;
    this->dirty_x1_ = x1;
    this->dirty_y1_ = y1;
  } else {
    this->dirty_x0_ = std::min(this->dirty_x0_, x0);
    this->dirty_y0_ = std::min(this->dirty_y0_, y0);
    this->dirty_x1_ = std::max(this->dirty_x1_, x1);
    this->dirty_y1_ = std::max(this->dirty_y1_, y1);
  }
  this->ui_pending_.store(true);
  xSemaphoreGive(this->ui_mutex_);
  if (this->task_ != nullptr)
    xTaskNotifyGive(this->task_);
  return true;
}

void Tab5Wallpaper::set_enabled(bool enabled) {
  if (this->enabled_.exchange(enabled) == enabled)
    return;
  if (this->is_failed())
    return;
  if (!enabled) {
    const bool was_showing = this->is_playing();
    this->lock_screen_gone();
    this->set_status_("Off");
    // Its last picture is on the screen: draw the lock screen again (the
    // display sends all of it now).
    if (was_showing)
      this->need_redraw_.store(true);
  } else {
    this->set_status_("Waiting for the lock screen");
    // If the lock screen is up, drawing it again starts the wallpaper.
    this->need_redraw_.store(true);
  }
}

void Tab5Wallpaper::close_file() {
  {
    std::lock_guard<std::mutex> guard(this->file_lock_);
    this->file_closed_ = true;
    this->close_file_locked_();
  }
  this->set_status_("File closed (swapping it)");
}

void Tab5Wallpaper::reload() {
  {
    std::lock_guard<std::mutex> guard(this->file_lock_);
    this->close_file_locked_();
    this->file_closed_ = false;
    this->next_open_us_ = 0;
  }
  {
    std::lock_guard<std::mutex> guard(this->status_lock_);
    this->note_.clear();
  }
  this->set_status_(this->enabled_.load() ? "Waiting for the lock screen" : "Off");
  if (this->task_ != nullptr)
    xTaskNotifyGive(this->task_);
}

void Tab5Wallpaper::set_note(const std::string &note) {
  std::lock_guard<std::mutex> guard(this->status_lock_);
  if (this->note_ != note) {
    this->note_ = note;
    this->status_version_.fetch_add(1);
  }
}

std::string Tab5Wallpaper::get_status() const {
  std::lock_guard<std::mutex> guard(this->status_lock_);
  return this->note_.empty() ? this->status_ : this->note_;
}

void Tab5Wallpaper::set_status_(const std::string &status) {
  std::lock_guard<std::mutex> guard(this->status_lock_);
  if (this->status_ != status) {
    this->status_ = status;
    this->status_version_.fetch_add(1);
  }
}

// ---------------------------------------------------------------------------
// The task

void Tab5Wallpaper::task_trampoline_(void *arg) {
  static_cast<Tab5Wallpaper *>(arg)->run_task_();
  vTaskDelete(nullptr);
}

void Tab5Wallpaper::run_task_() {
  int64_t due = 0;
  bool was_active = false;
  while (this->running_.load()) {
    if (!this->active_.load()) {
      if (was_active) {
        was_active = false;
        if (this->bg_valid_.load() && !this->broken_.load() && this->enabled_.load())
          this->set_status_(this->file_pictures_ > 0
                                ? "Ready: " + std::to_string(this->file_pictures_) + " pictures"
                                : std::string("Ready"));
      }
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      due = 0;
      continue;
    }
    if (!was_active) {
      was_active = true;
      this->stat_start_us_ = esp_timer_get_time();
      this->stat_frames_ = this->stat_late_ = 0;
      this->stat_read_us_ = this->stat_decode_us_ = this->stat_blend_us_ = 0;
    }
    if (this->broken_.load() || this->out_of_memory_) {
      // Nothing to do until the lock screen goes (lock screen changes
      // still go out if a picture was shown before).
      this->wait_until_(INT64_MAX);
      continue;
    }
    if (!this->prepared_ && !this->prepare_())
      continue;  // no file yet: prepare_() waited a while

    const int64_t t0 = esp_timer_get_time();
    const int got = this->next_picture_();
    if (got <= 0) {
      // No file (or closed for swapping, or broken): the last picture stays,
      // lock screen changes still go out. Look again in a while.
      this->wait_until_(esp_timer_get_time() + (got == 0 ? 250000 : kRetryUs));
      due = 0;
      continue;
    }
    const int64_t t1 = esp_timer_get_time();
    this->stat_read_us_ += t1 - t0;
    const int fps = this->file_fps_ != 0 ? this->file_fps_ : this->default_fps_;
    const int64_t period = 1000000LL / std::max(1, fps);
    if (due == 0 || t1 - due > 3 * period) {
      due = t1;  // starting, or far behind: don't race to catch up
    } else if (t1 > due) {
      this->stat_late_++;
    }
    this->wait_until_(due);
    due += period;
    if (!this->active_.load())
      continue;

    if (!this->decode_()) {
      if (++this->bad_pictures_ >= kMaxBadPictures) {
        ESP_LOGW(TAG, "%u pictures in a row couldn't be shown: trying the file again in a while",
                 (unsigned) this->bad_pictures_);
        {
          std::lock_guard<std::mutex> guard(this->file_lock_);
          this->close_file_locked_();
          this->next_open_us_ = esp_timer_get_time() + kRetryUs;
        }
        this->bad_pictures_ = 0;
      }
      continue;
    }
    this->bad_pictures_ = 0;
    if (!this->bg_valid_.exchange(true))
      ESP_LOGI(TAG, "First picture ready");

    // The whole screen: this picture with the lock screen over it.
    const int64_t t2 = esp_timer_get_time();
    bool shown = false;
    xSemaphoreTake(this->ui_mutex_, portMAX_DELAY);
    if (this->active_.load() && this->ui_valid_.load() && !this->broken_.load()) {
      shown = this->compose_(0, 0, this->panel_w_, this->panel_h_);
      this->dirty_x1_ = -1;
      this->dirty_y1_ = -1;
      this->ui_pending_.store(false);
    }
    xSemaphoreGive(this->ui_mutex_);
    if (!this->ui_valid_.load())
      this->want_ui_.store(true);  // the main task copies the lock screen, then wakes us
    if (shown) {
      this->stat_blend_us_ += esp_timer_get_time() - t2;
      this->stat_frames_++;
      this->frames_shown_.fetch_add(1);
      const int fps_now = this->file_fps_ != 0 ? this->file_fps_ : this->default_fps_;
      this->set_status_(this->file_pictures_ > 0 ? "Playing: " + std::to_string(this->file_pictures_) +
                                                       " pictures, " + std::to_string(fps_now) + " a second"
                                                 : "Playing: " + std::to_string(fps_now) + " pictures a second");
    }

    const int64_t now = esp_timer_get_time();
    if (now - this->stat_start_us_ >= kStatsUs && this->stat_frames_ > 0) {
      const float secs = (now - this->stat_start_us_) / 1e6f;
      const float n = this->stat_frames_;
      ESP_LOGD(TAG, "%.1f pictures/s (wanted %d): read %.1f ms, decode %.1f ms, blend %.1f ms, late %u",
               n / secs, fps, this->stat_read_us_ / n / 1000.0f, this->stat_decode_us_ / n / 1000.0f,
               this->stat_blend_us_ / n / 1000.0f, (unsigned) this->stat_late_);
      this->stat_start_us_ = now;
      this->stat_frames_ = this->stat_late_ = 0;
      this->stat_read_us_ = this->stat_decode_us_ = this->stat_blend_us_ = 0;
    }
  }
  {
    std::lock_guard<std::mutex> guard(this->file_lock_);
    this->close_file_locked_();
  }
  xSemaphoreGive(this->task_done_);
}

// Sleeps until `until_us`, sending lock screen changes as they come.
void Tab5Wallpaper::wait_until_(int64_t until_us) {
  for (;;) {
    this->serve_ui_();
    if (!this->active_.load() || !this->running_.load())
      return;
    const int64_t now = esp_timer_get_time();
    if (now >= until_us)
      return;
    TickType_t ticks = portMAX_DELAY;
    if (until_us != INT64_MAX) {
      ticks = pdMS_TO_TICKS((until_us - now + 999) / 1000);
      if (ticks == 0)
        ticks = 1;
    }
    ulTaskNotifyTake(pdTRUE, ticks);
  }
}

void Tab5Wallpaper::serve_ui_() {
  if (!this->ui_pending_.load())
    return;
  xSemaphoreTake(this->ui_mutex_, portMAX_DELAY);
  if (this->ui_pending_.exchange(false) && this->active_.load() && this->ui_valid_.load() &&
      this->bg_valid_.load() && !this->broken_.load() && this->dirty_x1_ >= this->dirty_x0_ &&
      this->dirty_y1_ >= this->dirty_y0_) {
    this->compose_(this->dirty_x0_, this->dirty_y0_, this->dirty_x1_ - this->dirty_x0_ + 1,
                   this->dirty_y1_ - this->dirty_y0_ + 1);
    this->dirty_x1_ = -1;
    this->dirty_y1_ = -1;
  }
  xSemaphoreGive(this->ui_mutex_);
}

// Buffers, made the first time the wallpaper plays (and kept).
bool Tab5Wallpaper::prepare_() {
  if (this->out_of_memory_)
    return false;
  // Only once there is a file: no file, no memory used.
  struct stat st {};
  if (stat(this->path_.c_str(), &st) != 0 || st.st_size <= 0) {
    this->set_status_("No wallpaper on the SD card");
    this->wait_until_(esp_timer_get_time() + kRetryUs);
    return false;
  }
  const size_t free_before = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  jpeg_decode_memory_alloc_cfg_t out_cfg{};
  out_cfg.buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER;
  jpeg_decode_memory_alloc_cfg_t in_cfg{};
  in_cfg.buffer_direction = JPEG_DEC_ALLOC_INPUT_BUFFER;
  if (this->bg_ == nullptr)
    this->bg_ = static_cast<uint8_t *>(jpeg_alloc_decoder_mem(this->fb_bytes_, &out_cfg, &this->bg_cap_));
  if (this->picture_ == nullptr)
    this->picture_ = static_cast<uint8_t *>(jpeg_alloc_decoder_mem(this->max_picture_bytes_, &in_cfg, &this->picture_cap_));
  if (this->chunk_ == nullptr)
    this->chunk_ = static_cast<uint8_t *>(
        heap_caps_aligned_alloc(128, kChunkBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT));
  if (this->bg_ == nullptr || this->picture_ == nullptr || this->chunk_ == nullptr) {
    ESP_LOGE(TAG, "Not enough PSRAM for the wallpaper (%u KB free, largest piece %u KB)",
             (unsigned) (heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
             (unsigned) (heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM) / 1024));
    this->out_of_memory_ = true;
    this->set_status_("Not enough memory");
    return false;
  }
  ESP_LOGI(TAG, "Buffers ready: %u KB of PSRAM (%u KB left)",
           (unsigned) ((free_before - heap_caps_get_free_size(MALLOC_CAP_SPIRAM)) / 1024),
           (unsigned) (heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
  this->prepared_ = true;
  return true;
}

// ---------------------------------------------------------------------------
// Reading the file

bool Tab5Wallpaper::open_file_() {
  const int64_t now = esp_timer_get_time();
  if (now < this->next_open_us_)
    return false;
  this->fd_ = open(this->path_.c_str(), O_RDONLY);
  if (this->fd_ < 0) {
    this->next_open_us_ = now + kRetryUs;
    this->set_status_(errno == ENOENT ? "No wallpaper on the SD card"
                                      : std::string("Can't open the wallpaper (") + strerror(errno) + ")");
    return false;
  }
  this->chunk_len_ = this->chunk_pos_ = 0;
  this->pstate_ = P_SOI;
  this->prev_ff_ = false;
  this->overflow_ = false;
  this->picture_ready_ = false;
  this->pass_pictures_ = 0;
  this->file_pictures_ = 0;
  this->empty_passes_ = 0;
  this->file_fps_ = 0;
  ESP_LOGI(TAG, "Playing %s", this->path_.c_str());
  return true;
}

void Tab5Wallpaper::close_file_locked_() {
  if (this->fd_ >= 0) {
    close(this->fd_);
    this->fd_ = -1;
  }
  this->chunk_len_ = this->chunk_pos_ = 0;
  this->pstate_ = P_SOI;
  this->prev_ff_ = false;
  this->picture_ready_ = false;
}

// 1: a picture is in picture_ (picture_len_ bytes). 0: the file is closed
// for swapping. -1: no file, or it can't be read.
int Tab5Wallpaper::next_picture_() {
  std::lock_guard<std::mutex> guard(this->file_lock_);
  if (this->file_closed_)
    return 0;
  if (this->fd_ < 0 && !this->open_file_())
    return -1;
  // A whole pass costs at most the file's size; a picture is far smaller.
  for (int reads = 0; reads < 4096; reads++) {
    if (this->feed_())
      return 1;
    // This piece is used up: read the next one.
    const ssize_t n = read(this->fd_, this->chunk_, kChunkBytes);
    if (n < 0) {
      ESP_LOGW(TAG, "Reading the wallpaper failed: %s", strerror(errno));
      this->set_status_(std::string("Can't read the wallpaper (") + strerror(errno) + ")");
      this->close_file_locked_();
      this->next_open_us_ = esp_timer_get_time() + kRetryUs;
      return -1;
    }
    if (n == 0) {
      // End of the file: round again from the start.
      if (this->pass_pictures_ == 0) {
        if (++this->empty_passes_ >= 2) {
          ESP_LOGW(TAG, "No JPEG pictures found in %s", this->path_.c_str());
          this->set_status_("No pictures in the wallpaper file");
          this->close_file_locked_();
          this->next_open_us_ = esp_timer_get_time() + kRetryUs * 6;
          return -1;
        }
      } else {
        this->empty_passes_ = 0;
        if (this->file_pictures_ == 0)
          ESP_LOGI(TAG, "%u pictures in the file", (unsigned) this->pass_pictures_);
        this->file_pictures_ = this->pass_pictures_;
      }
      this->pass_pictures_ = 0;
      lseek(this->fd_, 0, SEEK_SET);
      this->pstate_ = P_SOI;
      this->prev_ff_ = false;
      this->overflow_ = false;
      this->chunk_len_ = this->chunk_pos_ = 0;
      continue;
    }
    this->chunk_len_ = static_cast<size_t>(n);
    this->chunk_pos_ = 0;
  }
  return -1;
}

bool Tab5Wallpaper::append_(const uint8_t *data, size_t n) {
  if (this->overflow_)
    return false;
  if (this->picture_len_ + n > this->picture_cap_) {
    this->overflow_ = true;
    return false;
  }
  memcpy(this->picture_ + this->picture_len_, data, n);
  this->picture_len_ += n;
  return true;
}

// Splits the file into JPEG pictures: follows the markers and segment
// lengths, and in the compressed data looks for the end marker (FF D9; a
// real FF there is always followed by 00 or a restart marker). True when a
// whole picture is in picture_.
bool Tab5Wallpaper::feed_() {
  while (this->chunk_pos_ < this->chunk_len_) {
    const uint8_t *p = this->chunk_ + this->chunk_pos_;
    const size_t avail = this->chunk_len_ - this->chunk_pos_;
    switch (this->pstate_) {
      case P_SOI: {
        const uint8_t b = *p;
        this->chunk_pos_++;
        if (this->prev_ff_ && b == 0xD8) {
          this->picture_len_ = 0;
          this->overflow_ = false;
          const uint8_t soi[2] = {0xFF, 0xD8};
          this->append_(soi, 2);
          this->pstate_ = P_MARKER;
          this->marker_ff_ = false;
          this->prev_ff_ = false;
        } else {
          this->prev_ff_ = b == 0xFF;
        }
        break;
      }
      case P_MARKER: {
        const uint8_t b = *p;
        this->chunk_pos_++;
        if (!this->marker_ff_) {
          if (b != 0xFF) {  // not a JPEG after all: look for the next start
            this->pstate_ = P_SOI;
            this->prev_ff_ = false;
            break;
          }
          this->append_(p, 1);
          this->marker_ff_ = true;
          break;
        }
        if (b == 0xFF) {  // fill byte
          this->append_(p, 1);
          break;
        }
        this->append_(p, 1);
        this->marker_ff_ = false;
        this->marker_ = b;
        if (b == 0xD9) {
          this->picture_done_();
          if (this->picture_ready_)
            return true;
          break;
        }
        if (b == 0x01 || (b >= 0xD0 && b <= 0xD7))
          break;  // markers without a length
        this->pstate_ = P_LENGTH;
        this->len_got_ = 0;
        this->seg_len_ = 0;
        break;
      }
      case P_LENGTH: {
        const uint8_t b = *p;
        this->chunk_pos_++;
        this->append_(p, 1);
        this->seg_len_ = (this->seg_len_ << 8) | b;
        if (++this->len_got_ == 2) {
          if (this->seg_len_ < 2) {
            this->pstate_ = P_SOI;
            this->prev_ff_ = false;
            break;
          }
          this->seg_left_ = this->seg_len_ - 2;
          this->seg_start_ = this->picture_len_;
          this->pstate_ = P_SEGMENT;
          if (this->seg_left_ == 0)
            this->segment_done_();
        }
        break;
      }
      case P_SEGMENT: {
        const size_t n = std::min<size_t>(avail, this->seg_left_);
        this->append_(p, n);
        this->chunk_pos_ += n;
        this->seg_left_ -= n;
        if (this->seg_left_ == 0)
          this->segment_done_();
        break;
      }
      case P_ENTROPY: {
        const uint8_t *ff = static_cast<const uint8_t *>(memchr(p, 0xFF, avail));
        const size_t n = ff != nullptr ? static_cast<size_t>(ff - p) + 1 : avail;
        this->append_(p, n);
        this->chunk_pos_ += n;
        if (ff != nullptr)
          this->pstate_ = P_ENTROPY_FF;
        break;
      }
      case P_ENTROPY_FF: {
        const uint8_t b = *p;
        this->chunk_pos_++;
        this->append_(p, 1);
        if (b == 0x00 || (b >= 0xD0 && b <= 0xD7)) {
          this->pstate_ = P_ENTROPY;
        } else if (b == 0xFF) {
          // fill byte: still waiting for the marker
        } else if (b == 0xD9) {
          this->picture_done_();
          if (this->picture_ready_)
            return true;
        } else {
          // Another segment (for example the next scan's tables).
          this->marker_ = b;
          this->pstate_ = P_LENGTH;
          this->len_got_ = 0;
          this->seg_len_ = 0;
        }
        break;
      }
    }
  }
  return false;
}

void Tab5Wallpaper::segment_done_() {
  if (this->marker_ == 0xFE && !this->overflow_) {
    // A comment: "fps=20" sets the speed.
    const size_t n = this->picture_len_ - this->seg_start_;
    char text[64];
    const size_t m = std::min(n, sizeof(text) - 1);
    memcpy(text, this->picture_ + this->seg_start_, m);
    text[m] = 0;
    const char *f = strstr(text, "fps=");
    if (f != nullptr) {
      const int fps = atoi(f + 4);
      if (fps >= 1 && fps <= 60 && fps != this->file_fps_) {
        this->file_fps_ = static_cast<uint8_t>(fps);
        ESP_LOGI(TAG, "The file asks for %d pictures a second", fps);
      }
    }
  }
  this->pstate_ = this->marker_ == 0xDA ? P_ENTROPY : P_MARKER;
  this->marker_ff_ = false;
}

void Tab5Wallpaper::picture_done_() {
  this->pstate_ = P_SOI;
  this->prev_ff_ = false;
  if (this->overflow_) {
    this->overflow_ = false;
    this->picture_ready_ = false;
    ESP_LOGW(TAG, "A picture is bigger than %u KB: skipped", (unsigned) (this->picture_cap_ / 1024));
    this->set_status_("Pictures too big (over " + std::to_string(this->picture_cap_ / 1024) + " KB)");
    return;
  }
  this->picture_ready_ = true;
  this->pass_pictures_++;
}

// ---------------------------------------------------------------------------
// Decoding and showing

bool Tab5Wallpaper::decode_() {
  if (!this->picture_ready_)
    return false;
  this->picture_ready_ = false;
  const int64_t t0 = esp_timer_get_time();
  jpeg_decode_picture_info_t info{};
  if (jpeg_decoder_get_info(this->picture_, this->picture_len_, &info) != ESP_OK) {
    this->set_status_("Pictures can't be read (baseline JPEG only)");
    return false;
  }
  const int w = info.width, h = info.height;
  bool turn = false;
  uint8_t *out = this->bg_;
  size_t out_cap = this->bg_cap_;
  if (w == this->panel_w_ && h == this->panel_h_) {
    // Already turned for the panel: straight in.
  } else if (w == this->panel_h_ && h == this->panel_w_) {
    // Screen-shaped (1280 x 720): decoded, then turned by the PPA.
    turn = true;
    if (this->turn_ == nullptr) {
      jpeg_decode_memory_alloc_cfg_t out_cfg{};
      out_cfg.buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER;
      this->turn_ = static_cast<uint8_t *>(jpeg_alloc_decoder_mem(this->fb_bytes_, &out_cfg, &this->turn_cap_));
      ppa_client_config_t srm_config{};
      srm_config.oper_type = PPA_OPERATION_SRM;
      if (this->srm_ == nullptr && ppa_register_client(&srm_config, &this->srm_) != ESP_OK)
        this->srm_ = nullptr;
      if (this->turn_ == nullptr || this->srm_ == nullptr) {
        this->set_status_("Not enough memory to turn 1280x720 pictures");
        return false;
      }
    }
    out = this->turn_;
    out_cap = this->turn_cap_;
  } else {
    char words[96];
    snprintf(words, sizeof(words), "Pictures are %dx%d (must be %dx%d or %dx%d)", w, h, this->panel_h_,
             this->panel_w_, this->panel_w_, this->panel_h_);
    this->set_status_(words);
    return false;
  }
  uint32_t out_size = 0;
  const esp_err_t err = jpeg_decoder_process(this->decoder_, &this->decode_cfg_, this->picture_,
                                             this->picture_len_, out, out_cap, &out_size);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Picture didn't decode: %s", esp_err_to_name(err));
    return false;
  }
  if (turn) {
    // Same turn as the display's own (rotation 270 = the PPA's 90,
    // counter-clockwise).
    ppa_srm_oper_config_t srm{};
    srm.in.buffer = this->turn_;
    srm.in.pic_w = w;
    srm.in.pic_h = h;
    srm.in.block_w = w;
    srm.in.block_h = h;
    srm.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
    srm.out.buffer = this->bg_;
    srm.out.buffer_size = this->bg_cap_;
    srm.out.pic_w = this->panel_w_;
    srm.out.pic_h = this->panel_h_;
    srm.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
    srm.rotation_angle = PPA_SRM_ROTATION_ANGLE_90;
    srm.scale_x = 1.0f;
    srm.scale_y = 1.0f;
    srm.mode = PPA_TRANS_MODE_BLOCKING;
    const esp_err_t terr = ppa_do_scale_rotate_mirror(this->srm_, &srm);
    if (terr != ESP_OK) {
      ESP_LOGW(TAG, "Turning the picture failed: %s", esp_err_to_name(terr));
      return false;
    }
  }
  this->stat_decode_us_ += esp_timer_get_time() - t0;
  return true;
}

// Puts one area (panel coordinates) on the screen: the picture with the
// lock screen over it, black being see-through. Call with ui_mutex_ held.
bool Tab5Wallpaper::compose_(int x, int y, int w, int h) {
  ppa_blend_oper_config_t c{};
  c.in_bg.buffer = this->bg_;
  c.in_bg.pic_w = this->panel_w_;
  c.in_bg.pic_h = this->panel_h_;
  c.in_bg.block_w = w;
  c.in_bg.block_h = h;
  c.in_bg.block_offset_x = x;
  c.in_bg.block_offset_y = y;
  c.in_bg.blend_cm = PPA_BLEND_COLOR_MODE_RGB565;
  c.in_fg.buffer = this->ui_;
  c.in_fg.pic_w = this->panel_w_;
  c.in_fg.pic_h = this->panel_h_;
  c.in_fg.block_w = w;
  c.in_fg.block_h = h;
  c.in_fg.block_offset_x = x;
  c.in_fg.block_offset_y = y;
  c.in_fg.blend_cm = PPA_BLEND_COLOR_MODE_RGB565;
  c.out.buffer = this->fb_;
  c.out.buffer_size = this->fb_bytes_;
  c.out.pic_w = this->panel_w_;
  c.out.pic_h = this->panel_h_;
  c.out.block_offset_x = x;
  c.out.block_offset_y = y;
  c.out.blend_cm = PPA_BLEND_COLOR_MODE_RGB565;
  // Both layers fully opaque; the lock screen's black is the hole.
  c.bg_alpha_update_mode = PPA_ALPHA_FIX_VALUE;
  c.bg_alpha_fix_val = 255;
  c.fg_alpha_update_mode = PPA_ALPHA_FIX_VALUE;
  c.fg_alpha_fix_val = 255;
  c.bg_ck_en = false;
  c.fg_ck_en = true;
  c.fg_ck_rgb_low_thres.r = 0;
  c.fg_ck_rgb_low_thres.g = 0;
  c.fg_ck_rgb_low_thres.b = 0;
  // Only pure black (the next RGB565 step up is 4 or more in RGB888).
  c.fg_ck_rgb_high_thres.r = 3;
  c.fg_ck_rgb_high_thres.g = 3;
  c.fg_ck_rgb_high_thres.b = 3;
  c.ck_reverse_bg2fg = false;
  c.mode = PPA_TRANS_MODE_BLOCKING;
  const esp_err_t err = ppa_do_blend(this->blend_, &c);
  if (err != ESP_OK) {
    // The lock screen would never reach the screen: give the screen back
    // to the display for good.
    ESP_LOGE(TAG, "Blending failed (%s): wallpaper off until restart", esp_err_to_name(err));
    this->broken_.store(true);
    this->set_status_("Error: can't draw on the screen");
    this->need_redraw_.store(true);
    return false;
  }
  return true;
}

}  // namespace esphome::tab5_wallpaper

#endif
