#include "tab5_video.h"

#if defined(USE_ESP_IDF) && defined(USE_ESPHOME_VOIP_STACK_VIDEO_JPEG)

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include "esp_heap_caps.h"
#include "esp_rom_crc.h"

#include <lwip/netdb.h>
#include <lwip/sockets.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <cstring>

namespace esphome::tab5_video {

static const char *const TAG = "tab5_video";

namespace {

// Biggest single JPEG picture accepted. A 640x360 call picture is about
// 20-40 KB; a browser's 1280x720 picture can reach a few hundred KB.
constexpr size_t kMaxPictureBytes = 384 * 1024;
// Decode task: below the audio and video network tasks, stack in PSRAM to
// keep the scarce internal memory free. It never touches flash.
constexpr uint32_t kTaskStackBytes = 6144;
constexpr UBaseType_t kTaskPriority = 5;
// PPA scale factors have 1/16 steps.
constexpr int kScaleUnits = 16;
// Biggest own camera picture kept for the self view.
constexpr size_t kMaxSelfPictureBytes = 256 * 1024;
// Still pictures: biggest JPEG accepted, and the fetch task (below the
// decode task; stack in PSRAM, it never touches flash).
constexpr size_t kMaxStillBytes = 1024 * 1024;
constexpr uint32_t kFetchStackBytes = 8192;
constexpr UBaseType_t kFetchPriority = 3;
constexpr int kConnectTimeoutMs = 5000;
constexpr int kReadTimeoutMs = 10000;

enum PictureError : uint8_t {
  PIC_OK = 0,
  PIC_BAD_URL,
  PIC_NO_HOST,
  PIC_NO_CONNECT,
  PIC_HTTP_STATUS,
  PIC_BAD_REPLY,
  PIC_READ_FAILED,
  PIC_TOO_MANY_BYTES,
  PIC_NOT_JPEG,
  PIC_TOO_BIG,
  PIC_CANT_DECODE,
};

inline uint32_t align_up(uint32_t value, uint32_t alignment) {
  return (value + alignment - 1U) & ~(alignment - 1U);
}

}  // namespace

void Tab5VideoView::setup() {
  if (this->display_ == nullptr || this->area_w_ <= 0 || this->area_h_ <= 0) {
    ESP_LOGE(TAG, "No display or empty video area");
    this->mark_failed();
    return;
  }

  // Hardware JPEG decoder. The camera's encoder shares the one JPEG engine;
  // ESP-IDF takes turns between them picture by picture.
  jpeg_decode_engine_cfg_t engine_config{};
  engine_config.timeout_ms = 100;
  if (jpeg_new_decoder_engine(&engine_config, &this->decoder_) != ESP_OK ||
      this->decoder_ == nullptr) {
    ESP_LOGE(TAG, "JPEG decoder not available");
    this->decoder_ = nullptr;
    this->mark_failed();
    return;
  }
  this->decode_cfg_.output_format = JPEG_DECODE_OUT_FORMAT_RGB565;
  // "BGR" is ESP-IDF's name for normal little-endian RGB565, the format the
  // ESPHome mipi_dsi panel buffer uses.
  this->decode_cfg_.rgb_order =
      this->swap_rb_ ? JPEG_DEC_RGB_ELEMENT_ORDER_RGB : JPEG_DEC_RGB_ELEMENT_ORDER_BGR;
  this->decode_cfg_.conv_std = JPEG_YUV_RGB_CONV_STD_BT601;

  ppa_client_config_t ppa_config{};
  ppa_config.oper_type = PPA_OPERATION_SRM;
  if (ppa_register_client(&ppa_config, &this->ppa_) != ESP_OK || this->ppa_ == nullptr) {
    ESP_LOGE(TAG, "PPA not available");
    this->ppa_ = nullptr;
    this->mark_failed();
    return;
  }

  // All big buffers live in PSRAM and are made once, so a call never has to
  // find megabytes of free memory.
  this->au_buf_ = static_cast<uint8_t *>(
      heap_caps_aligned_alloc(64, kMaxPictureBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  this->au_cap_ = this->au_buf_ == nullptr ? 0 : kMaxPictureBytes;

  jpeg_decode_memory_alloc_cfg_t mem_cfg{};
  mem_cfg.buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER;
  const size_t surface_bytes =
      static_cast<size_t>(align_up(this->max_decode_w_, 16)) * align_up(this->max_decode_h_, 16) * 2U;
  this->surface_ = static_cast<uint8_t *>(jpeg_alloc_decoder_mem(surface_bytes, &mem_cfg, &this->surface_cap_));
  if (this->surface_ == nullptr)
    this->surface_cap_ = 0;

  // The panel's DMA reads this buffer: align it to the P4's largest cache
  // line (128 bytes, PSRAM L2).
  this->out_align_ = 128;
  const size_t out_bytes = align_up(static_cast<uint32_t>(this->area_w_) * this->area_h_ * 2U, this->out_align_);
  this->out_buf_ = static_cast<uint8_t *>(heap_caps_aligned_calloc(
      this->out_align_, 1, out_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT));
  this->out_cap_ = this->out_buf_ == nullptr ? 0 : out_bytes;

  if (this->au_buf_ == nullptr || this->surface_ == nullptr || this->out_buf_ == nullptr) {
    ESP_LOGE(TAG, "Not enough PSRAM for video buffers");
    this->mark_failed();
    return;
  }

  if (this->self_enabled_) {
    this->self_au_buf_ = static_cast<uint8_t *>(
        heap_caps_aligned_alloc(64, kMaxSelfPictureBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    this->self_au_cap_ = this->self_au_buf_ == nullptr ? 0 : kMaxSelfPictureBytes;
    const size_t self_bytes =
        static_cast<size_t>(align_up(this->self_max_w_, 16)) * align_up(this->self_max_h_, 16) * 2U;
    this->self_surface_ =
        static_cast<uint8_t *>(jpeg_alloc_decoder_mem(self_bytes, &mem_cfg, &this->self_surface_cap_));
    if (this->self_surface_ == nullptr)
      this->self_surface_cap_ = 0;
    // The thumbnail is never bigger than the picture itself.
    const size_t thumb_bytes =
        align_up(static_cast<uint32_t>(this->self_max_w_) * this->self_max_h_ * 2U, this->out_align_);
    this->thumb_ = static_cast<uint8_t *>(heap_caps_aligned_calloc(
        this->out_align_, 1, thumb_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT));
    this->thumb_native_ = static_cast<uint8_t *>(heap_caps_aligned_calloc(
        this->out_align_, 1, thumb_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT));
    this->thumb_cap_ = thumb_bytes;
    if (this->self_au_buf_ == nullptr || this->self_surface_ == nullptr || this->thumb_ == nullptr ||
        this->thumb_native_ == nullptr) {
      // Calls and the other side's picture still work without it.
      ESP_LOGW(TAG, "Not enough PSRAM for the self view: turned off");
      this->self_enabled_ = false;
    }
  }

  this->task_done_ = xSemaphoreCreateBinary();
  this->running_.store(true, std::memory_order_release);
  if (this->task_done_ == nullptr ||
      xTaskCreatePinnedToCoreWithCaps(Tab5VideoView::decode_task_trampoline_, "tab5_video", kTaskStackBytes,
                                      this, kTaskPriority, &this->task_, tskNO_AFFINITY,
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
    this->running_.store(false, std::memory_order_release);
    this->task_ = nullptr;
    ESP_LOGE(TAG, "Could not start the video decode task");
    this->mark_failed();
    return;
  }
  // The other side can stop its camera without ending the video stream:
  // when no picture came for a while, give the screen back.
  this->set_interval("no_video", 500, [this]() { this->check_stale_(); });
  this->disable_loop();
}

void Tab5VideoView::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Tab5 video view:\n"
                "  Area: %dx%d at %d,%d (display rotation %d)\n"
                "  Asks for: %ux%u at %u fps (accepts up to %ux%u)\n"
                "  Fill by cutting up to %.0f%%\n"
                "  Picture off after %u ms without video",
                this->area_w_, this->area_h_, this->area_x_, this->area_y_, this->display_rotation_,
                this->rx_width_, this->rx_height_, this->framerate_, this->max_decode_w_, this->max_decode_h_,
                this->max_crop_ * 100.0f, (unsigned) this->no_video_timeout_ms_);
  if (this->self_enabled_) {
    ESP_LOGCONFIG(TAG, "  Self view: up to %ux%u at %u fps%s", this->self_max_w_, this->self_max_h_, this->self_fps_,
                  this->self_mirror_ ? ", mirrored" : "");
  }
  if (this->is_failed()) {
    ESP_LOGCONFIG(TAG, "  FAILED to start (see the log above)");
  }
}

void Tab5VideoView::on_shutdown() {
  this->active_.store(false, std::memory_order_release);
  this->prepared_.store(false, std::memory_order_release);
  this->picture_on_.store(false, std::memory_order_release);
  if (this->task_ != nullptr && this->running_.exchange(false, std::memory_order_acq_rel)) {
    xTaskNotifyGive(this->task_);
    xSemaphoreTake(this->task_done_, pdMS_TO_TICKS(500));
  }
  // The fetch task sees running_ off after its current fetch, if any.
  if (this->fetch_handle_ != nullptr)
    xTaskNotifyGive(this->fetch_handle_);
}

voip_stack::VideoCapability Tab5VideoView::get_receive_video_capability() const {
  voip_stack::VideoCapability capability;
  capability.payload_type = 26;
  capability.encoding = "JPEG";
  capability.width = this->rx_width_;
  capability.height = this->rx_height_;
  capability.max_fps = this->framerate_;
  return capability;
}

bool Tab5VideoView::start_video(const voip_stack::VideoCapability &capability) {
  if (this->is_failed() || !capability.valid() || !capability.is_jpeg() ||
      !this->running_.load(std::memory_order_acquire)) {
    ESP_LOGW(TAG, "Video refused (failed=%s, jpeg=%s)", YESNO(this->is_failed()), YESNO(capability.is_jpeg()));
    return false;
  }
  if (this->prepared_.exchange(true, std::memory_order_acq_rel)) {
    ESP_LOGW(TAG, "Video refused: one is already set up");
    return false;
  }
  // A call's video takes the screen from a still picture.
  this->hide_picture();
  this->generation_.fetch_add(1, std::memory_order_acq_rel);
  this->negotiated_fps_ =
      capability.max_fps == 0 ? this->framerate_ : std::min<uint8_t>(this->framerate_, capability.max_fps);
  this->cadence_.reset(this->negotiated_fps_);
  this->frames_shown_.store(0, std::memory_order_relaxed);
  this->frames_decoded_.store(0, std::memory_order_relaxed);
  this->decode_errors_.store(0, std::memory_order_relaxed);
  this->active_.store(false, std::memory_order_release);
  this->visible_.store(false, std::memory_order_release);
  this->logged_geometry_ = false;
  ESP_LOGI(TAG, "Video ready to receive (up to %u fps)", this->negotiated_fps_);
  return true;
}

bool Tab5VideoView::set_video_active(bool active) {
  if (active) {
    if (!this->prepared_.load(std::memory_order_acquire) || !this->running_.load(std::memory_order_acquire))
      return false;
    this->cadence_.reset(this->negotiated_fps_);
    this->active_.store(true, std::memory_order_release);
    return true;
  }
  // A picture already decoded for the old stream is dropped by loop()
  // because its generation no longer matches.
  this->active_.store(false, std::memory_order_release);
  this->generation_.fetch_add(1, std::memory_order_acq_rel);
  if (this->visible_.exchange(false, std::memory_order_acq_rel)) {
    this->ended_pending_.store(true, std::memory_order_release);
    this->enable_loop_soon_any_context();
  }
  return true;
}

void Tab5VideoView::stop_video() {
  this->set_video_active(false);
  this->prepared_.store(false, std::memory_order_release);
  ESP_LOGI(TAG, "Video stopped (%u pictures shown, %u could not be decoded)",
           (unsigned) this->frames_shown_.load(std::memory_order_relaxed),
           (unsigned) this->decode_errors_.load(std::memory_order_relaxed));
}

bool Tab5VideoView::consume_video_access_unit(const voip_stack::EncodedVideoAccessUnit &access_unit) {
  // Runs on voip_stack's video RTP task: copy and return quickly.
  if (!this->active_.load(std::memory_order_acquire) || access_unit.data == nullptr ||
      access_unit.size == 0 || access_unit.size > this->au_cap_) {
    return false;
  }
  if (!this->cadence_.accept(access_unit.timestamp_90khz))
    return false;
  uint8_t expected = 0;
  if (!this->au_state_.compare_exchange_strong(expected, 2, std::memory_order_acq_rel))
    return false;  // the previous picture is still waiting: drop this one
  std::memcpy(this->au_buf_, access_unit.data, access_unit.size);
  this->au_size_ = access_unit.size;
  this->au_generation_ = this->generation_.load(std::memory_order_acquire);
  this->au_state_.store(1, std::memory_order_release);
  if (this->task_ != nullptr)
    xTaskNotifyGive(this->task_);
  return true;
}

void Tab5VideoView::decode_task_trampoline_(void *arg) {
  static_cast<Tab5VideoView *>(arg)->decode_task_();
}

void Tab5VideoView::decode_task_() {
  while (this->running_.load(std::memory_order_acquire)) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (!this->running_.load(std::memory_order_acquire))
      break;
    // The other side's picture first, then this device's own, then a
    // still picture.
    this->decode_remote_();
    this->decode_self_();
    this->decode_picture_();
  }
  xSemaphoreGive(this->task_done_);
  vTaskSuspend(nullptr);
}

void Tab5VideoView::decode_remote_() {
  static uint32_t last_error_log = 0;
  // The picture waits in its slot until loop() has taken the last one.
  if (this->au_state_.load(std::memory_order_acquire) != 1 ||
      this->surface_state_.load(std::memory_order_acquire) != 0)
    return;
  uint8_t expected = 1;
  if (!this->au_state_.compare_exchange_strong(expected, 2, std::memory_order_acq_rel))
    return;

  const uint32_t generation = this->au_generation_;
  if (!this->active_.load(std::memory_order_acquire) ||
      generation != this->generation_.load(std::memory_order_acquire)) {
    this->au_state_.store(0, std::memory_order_release);
    return;
  }

  jpeg_decode_picture_info_t info{};
  bool ok = jpeg_decoder_get_info(this->au_buf_, static_cast<uint32_t>(this->au_size_), &info) == ESP_OK &&
            info.sample_method != JPEG_DOWN_SAMPLING_GRAY && info.width > 0 && info.height > 0 &&
            info.width <= this->max_decode_w_ && info.height <= this->max_decode_h_;
  uint32_t store_w = 0, store_h = 0, need = 0;
  if (ok) {
    // The decoder writes whole JPEG blocks.
    store_w = align_up(info.width, info.sample_method == JPEG_DOWN_SAMPLING_YUV444 ? 8 : 16);
    store_h = align_up(info.height, info.sample_method == JPEG_DOWN_SAMPLING_YUV420 ? 16 : 8);
    need = store_w * store_h * 2U;
    ok = need <= this->surface_cap_;
  }
  esp_err_t err = ESP_ERR_INVALID_SIZE;
  uint32_t decoded = 0;
  if (ok) {
    this->surface_state_.store(1, std::memory_order_release);
    err = jpeg_decoder_process(this->decoder_, &this->decode_cfg_, this->au_buf_,
                               static_cast<uint32_t>(this->au_size_), this->surface_, need, &decoded);
  }
  this->au_state_.store(0, std::memory_order_release);

  if (ok && err == ESP_OK && decoded > 0 && this->active_.load(std::memory_order_acquire) &&
      generation == this->generation_.load(std::memory_order_acquire)) {
    this->surf_w_ = static_cast<uint16_t>(info.width);
    this->surf_h_ = static_cast<uint16_t>(info.height);
    this->surf_store_w_ = static_cast<uint16_t>(store_w);
    this->surf_store_h_ = static_cast<uint16_t>(store_h);
    this->surface_generation_ = generation;
    this->surface_is_picture_ = false;
    this->frames_decoded_.fetch_add(1, std::memory_order_relaxed);
    this->surface_state_.store(2, std::memory_order_release);
    this->enable_loop_soon_any_context();
  } else {
    this->surface_state_.store(0, std::memory_order_release);
    if (!ok || err != ESP_OK) {
      this->decode_errors_.fetch_add(1, std::memory_order_relaxed);
      const uint32_t now = millis();
      if (now - last_error_log > 5000) {
        last_error_log = now;
        ESP_LOGW(TAG, "Could not decode a picture (%ux%u, %u bytes): %s", (unsigned) info.width,
                 (unsigned) info.height, (unsigned) this->au_size_, ok ? esp_err_to_name(err) : "unsupported size");
      }
    }
  }
}

void Tab5VideoView::decode_self_() {
  static uint32_t last_error_log = 0;
  if (!this->self_enabled_ || this->self_au_state_.load(std::memory_order_acquire) != 1 ||
      this->self_surface_state_.load(std::memory_order_acquire) != 0)
    return;
  uint8_t expected = 1;
  if (!this->self_au_state_.compare_exchange_strong(expected, 2, std::memory_order_acq_rel))
    return;

  jpeg_decode_picture_info_t info{};
  bool ok = jpeg_decoder_get_info(this->self_au_buf_, static_cast<uint32_t>(this->self_au_size_), &info) == ESP_OK &&
            info.sample_method != JPEG_DOWN_SAMPLING_GRAY && info.width > 0 && info.height > 0;
  uint32_t store_w = 0, store_h = 0, need = 0;
  if (ok) {
    store_w = align_up(info.width, info.sample_method == JPEG_DOWN_SAMPLING_YUV444 ? 8 : 16);
    store_h = align_up(info.height, info.sample_method == JPEG_DOWN_SAMPLING_YUV420 ? 16 : 8);
    need = store_w * store_h * 2U;
    ok = need <= this->self_surface_cap_ && static_cast<size_t>(info.width) * info.height * 2U <= this->thumb_cap_;
  }
  esp_err_t err = ESP_ERR_INVALID_SIZE;
  uint32_t decoded = 0;
  if (ok) {
    this->self_surface_state_.store(1, std::memory_order_release);
    err = jpeg_decoder_process(this->decoder_, &this->decode_cfg_, this->self_au_buf_,
                               static_cast<uint32_t>(this->self_au_size_), this->self_surface_, need, &decoded);
  }
  this->self_au_state_.store(0, std::memory_order_release);
  if (ok && err == ESP_OK && decoded > 0 && this->self_visible_.load(std::memory_order_acquire)) {
    this->self_w_px_ = static_cast<uint16_t>(info.width);
    this->self_h_px_ = static_cast<uint16_t>(info.height);
    this->self_store_w_ = static_cast<uint16_t>(store_w);
    this->self_store_h_ = static_cast<uint16_t>(store_h);
    this->self_surface_state_.store(2, std::memory_order_release);
    this->enable_loop_soon_any_context();
    return;
  }
  this->self_surface_state_.store(0, std::memory_order_release);
  if (!ok || err != ESP_OK) {
    const uint32_t now = millis();
    if (now - last_error_log > 5000) {
      last_error_log = now;
      ESP_LOGW(TAG, "Could not decode own camera picture (%ux%u, %u bytes): %s", (unsigned) info.width,
               (unsigned) info.height, (unsigned) this->self_au_size_, ok ? esp_err_to_name(err) : "too big");
    }
  }
}

void Tab5VideoView::loop() {
  // Clear the wake-up first, so one that arrives while this runs is kept.
  this->disable_loop();

  if (this->ended_pending_.exchange(false, std::memory_order_acq_rel)) {
    this->end_presentation_();
    this->video_ended_trigger_.trigger();
  }
  bool picture_note = false;
  if (this->picture_ended_pending_.exchange(false, std::memory_order_acq_rel)) {
    this->end_presentation_();
    picture_note = true;
  }

  if (this->surface_state_.load(std::memory_order_acquire) == 2) {
    const uint32_t generation = this->generation_.load(std::memory_order_acquire);
    const bool paused = this->paused_.load(std::memory_order_acquire);
    bool first = false;
    if (this->surface_is_picture_) {
      // A still picture: only while still wanted and no call video is set up.
      if (this->picture_on_.load(std::memory_order_acquire) && !this->prepared_.load(std::memory_order_acquire) &&
          this->surface_generation_ == generation && !paused && this->present_()) {
        this->shown_crc_.store(this->surface_crc_, std::memory_order_release);
        this->picture_changed_ms_ = millis();
        this->pic_shown_w_ = this->surf_w_;
        this->pic_shown_h_ = this->surf_h_;
        if (!this->picture_visible_.exchange(true, std::memory_order_acq_rel))
          ESP_LOGI(TAG, "Showing the still picture (%ux%u)", this->surf_w_, this->surf_h_);
        picture_note = true;
      }
    } else {
      const bool current = this->active_.load(std::memory_order_acquire) && this->surface_generation_ == generation;
      if (current && !paused && this->present_()) {
        this->frames_shown_.fetch_add(1, std::memory_order_relaxed);
        this->last_shown_ms_ = millis();
        first = !this->visible_.exchange(true, std::memory_order_acq_rel);
      }
    }
    this->surface_state_.store(0, std::memory_order_release);
    // A picture that arrived meanwhile can be decoded now.
    if (this->task_ != nullptr)
      xTaskNotifyGive(this->task_);
    if (first) {
      ESP_LOGI(TAG, "Showing the other side's video");
      this->first_frame_trigger_.trigger();
    }
  }
  if (this->picture_note_pending_.exchange(false, std::memory_order_acq_rel))
    picture_note = true;
  if (picture_note)
    this->picture_trigger_.trigger();

  // A new own camera picture: scale it once, then draw it.
  if (this->self_surface_state_.load(std::memory_order_acquire) == 2) {
    if (this->self_visible_.load(std::memory_order_acquire) && this->render_thumb_())
      this->self_redraw_ = true;
    this->self_surface_state_.store(0, std::memory_order_release);
    if (this->task_ != nullptr)
      xTaskNotifyGive(this->task_);
  }
  if (this->self_redraw_) {
    this->self_redraw_ = false;
    this->draw_self_();
  }
}

void Tab5VideoView::set_self_view(int x, int y, int w, int h) {
  if (!this->self_enabled_)
    return;
  if (w <= 0 || h <= 0) {
    this->self_rx_ = this->self_ry_ = this->self_rw_ = this->self_rh_ = 0;
    this->self_visible_.store(false, std::memory_order_release);
    this->thumb_valid_ = false;
    this->self_redraw_ = false;
    return;
  }
  if (w != this->self_rw_ || h != this->self_rh_)
    this->thumb_valid_ = false;  // made for another size: wait for the next picture
  if (!this->self_visible_.load(std::memory_order_acquire))
    this->self_cadence_.reset(this->self_fps_);
  this->self_rx_ = x;
  this->self_ry_ = y;
  this->self_rw_ = w;
  this->self_rh_ = h;
  this->self_visible_.store(true, std::memory_order_release);
  // The screen update that called this may cover the picture: draw it again
  // once that update is done.
  this->self_redraw_ = true;
  this->enable_loop();
}

void Tab5VideoView::offer_self_frame(const voip_stack::EncodedVideoAccessUnit &access_unit) {
  // Runs on the camera's task: copy and return quickly.
  if (!this->self_enabled_ || !this->self_visible_.load(std::memory_order_acquire) || access_unit.data == nullptr ||
      access_unit.size == 0 || access_unit.size > this->self_au_cap_)
    return;
  if (!this->self_cadence_.accept(access_unit.timestamp_90khz))
    return;
  uint8_t expected = 0;
  if (!this->self_au_state_.compare_exchange_strong(expected, 2, std::memory_order_acq_rel))
    return;  // still busy with the last one: skip this picture
  std::memcpy(this->self_au_buf_, access_unit.data, access_unit.size);
  this->self_au_size_ = access_unit.size;
  this->self_au_state_.store(1, std::memory_order_release);
  if (this->task_ != nullptr)
    xTaskNotifyGive(this->task_);
}

bool Tab5VideoView::render_thumb_() {
  const int cw = this->self_w_px_, ch = this->self_h_px_;
  const int rw = this->self_rw_, rh = this->self_rh_;
  if (cw <= 0 || ch <= 0 || rw <= 0 || rh <= 0)
    return false;
  // Fit the whole picture in the rectangle (never bigger than it is).
  const int units = std::min({kScaleUnits, rw * kScaleUnits / cw, rh * kScaleUnits / ch});
  if (units < 1)
    return false;
  const int tw = cw * units / kScaleUnits, th = ch * units / kScaleUnits;
  if (tw <= 0 || th <= 0 || static_cast<size_t>(tw) * th * 2U > this->thumb_cap_)
    return false;
  // Scale and mirror without turning: with no rotation the mirror is plainly
  // left-right. The turn for the panel is a second step (draw_self_ and
  // composite_self_).
  ppa_srm_oper_config_t srm{};
  srm.in.buffer = this->self_surface_;
  srm.in.pic_w = this->self_store_w_;
  srm.in.pic_h = this->self_store_h_;
  srm.in.block_w = cw;
  srm.in.block_h = ch;
  srm.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
  srm.out.buffer = this->thumb_;
  srm.out.buffer_size = this->thumb_cap_;
  srm.out.pic_w = tw;
  srm.out.pic_h = th;
  srm.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
  srm.rotation_angle = PPA_SRM_ROTATION_ANGLE_0;
  srm.scale_x = static_cast<float>(units) / kScaleUnits;
  srm.scale_y = srm.scale_x;
  srm.mirror_x = this->self_mirror_;
  srm.mode = PPA_TRANS_MODE_BLOCKING;
  const esp_err_t err = ppa_do_scale_rotate_mirror(this->ppa_, &srm);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Self view scaling failed: %s", esp_err_to_name(err));
    this->thumb_valid_ = false;
    return false;
  }
  this->thumb_w_ = tw;
  this->thumb_h_ = th;
  this->thumb_valid_ = true;
  return true;
}

bool Tab5VideoView::thumb_place_(int *lx, int *ly) const {
  if (!this->thumb_valid_ || this->self_rw_ <= 0 || this->thumb_w_ > this->self_rw_ ||
      this->thumb_h_ > this->self_rh_)
    return false;
  *lx = this->self_rx_ + (this->self_rw_ - this->thumb_w_) / 2;
  *ly = this->self_ry_ + (this->self_rh_ - this->thumb_h_) / 2;
  return true;
}

bool Tab5VideoView::draw_self_() {
  int lx, ly;
  if (!this->self_visible_.load(std::memory_order_acquire) || !this->thumb_place_(&lx, &ly))
    return false;
  int nx, ny, nw, nh;
  ppa_srm_rotation_angle_t angle;
  this->map_to_native_(lx, ly, this->thumb_w_, this->thumb_h_, &nx, &ny, &nw, &nh, &angle);
  if (nx < 0 || ny < 0 || nx + nw > this->display_->get_native_width() ||
      ny + nh > this->display_->get_native_height())
    return false;
  ppa_srm_oper_config_t srm{};
  srm.in.buffer = this->thumb_;
  srm.in.pic_w = this->thumb_w_;
  srm.in.pic_h = this->thumb_h_;
  srm.in.block_w = this->thumb_w_;
  srm.in.block_h = this->thumb_h_;
  srm.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
  srm.out.buffer = this->thumb_native_;
  srm.out.buffer_size = this->thumb_cap_;
  srm.out.pic_w = nw;
  srm.out.pic_h = nh;
  srm.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
  srm.rotation_angle = angle;
  srm.scale_x = 1.0f;
  srm.scale_y = 1.0f;
  srm.mode = PPA_TRANS_MODE_BLOCKING;
  if (ppa_do_scale_rotate_mirror(this->ppa_, &srm) != ESP_OK)
    return false;
  this->display_->draw_pixels_at(nx, ny, nw, nh, this->thumb_native_, display::COLOR_ORDER_RGB,
                                 display::COLOR_BITNESS_565, false, 0, 0, 0);
  return true;
}

void Tab5VideoView::composite_self_(int pic_nx, int pic_ny, int pic_nw, int pic_nh) {
  // Put the own picture into the other side's picture before it goes to the
  // panel, so the corner never flickers.
  int lx, ly;
  if (!this->self_visible_.load(std::memory_order_acquire) || !this->thumb_place_(&lx, &ly))
    return;
  int nx, ny, nw, nh;
  ppa_srm_rotation_angle_t angle;
  this->map_to_native_(lx, ly, this->thumb_w_, this->thumb_h_, &nx, &ny, &nw, &nh, &angle);
  const int ox = nx - pic_nx, oy = ny - pic_ny;
  if (ox < 0 || oy < 0 || ox + nw > pic_nw || oy + nh > pic_nh)
    return;  // not inside the picture: loop() draws it on its own
  ppa_srm_oper_config_t srm{};
  srm.in.buffer = this->thumb_;
  srm.in.pic_w = this->thumb_w_;
  srm.in.pic_h = this->thumb_h_;
  srm.in.block_w = this->thumb_w_;
  srm.in.block_h = this->thumb_h_;
  srm.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
  srm.out.buffer = this->out_buf_;
  srm.out.buffer_size = this->out_cap_;
  srm.out.pic_w = pic_nw;
  srm.out.pic_h = pic_nh;
  srm.out.block_offset_x = ox;
  srm.out.block_offset_y = oy;
  srm.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
  srm.rotation_angle = angle;
  srm.scale_x = 1.0f;
  srm.scale_y = 1.0f;
  srm.mode = PPA_TRANS_MODE_BLOCKING;
  if (ppa_do_scale_rotate_mirror(this->ppa_, &srm) != ESP_OK)
    this->self_redraw_ = true;
}

void Tab5VideoView::check_stale_() {
  if (this->no_video_timeout_ms_ == 0 || !this->visible_.load(std::memory_order_acquire) ||
      this->paused_.load(std::memory_order_acquire) || millis() - this->last_shown_ms_ < this->no_video_timeout_ms_)
    return;
  if (!this->visible_.exchange(false, std::memory_order_acq_rel))
    return;
  ESP_LOGI(TAG, "No video from the other side for %u ms: picture off", (unsigned) this->no_video_timeout_ms_);
  this->end_presentation_();
  this->video_ended_trigger_.trigger();
}

void Tab5VideoView::set_paused(bool paused) {
  if (paused == this->paused_.exchange(paused, std::memory_order_acq_rel))
    return;
  // Whatever covered the area is gone or about to arrive: draw afresh.
  this->end_presentation_();
}

void Tab5VideoView::end_presentation_() {
  this->shown_lx_ = -1;
  this->shown_ly_ = -1;
  this->shown_lw_ = 0;
  this->shown_lh_ = 0;
  this->area_clean_ = false;
}

void Tab5VideoView::map_to_native_(int lx, int ly, int lw, int lh, int *nx, int *ny, int *nw, int *nh,
                                   ppa_srm_rotation_angle_t *angle) const {
  // Same mapping as the display's own software rotation (mipi_dsi
  // draw_pixel_at). PPA angles turn counter-clockwise.
  const int native_w = this->display_->get_native_width();
  const int native_h = this->display_->get_native_height();
  switch (this->display_rotation_) {
    case 90:
      *nx = native_w - ly - lh;
      *ny = lx;
      *nw = lh;
      *nh = lw;
      *angle = PPA_SRM_ROTATION_ANGLE_270;
      break;
    case 180:
      *nx = native_w - lx - lw;
      *ny = native_h - ly - lh;
      *nw = lw;
      *nh = lh;
      *angle = PPA_SRM_ROTATION_ANGLE_180;
      break;
    case 270:
      *nx = ly;
      *ny = native_h - lx - lw;
      *nw = lh;
      *nh = lw;
      *angle = PPA_SRM_ROTATION_ANGLE_90;
      break;
    default:
      *nx = lx;
      *ny = ly;
      *nw = lw;
      *nh = lh;
      *angle = PPA_SRM_ROTATION_ANGLE_0;
      break;
  }
}

bool Tab5VideoView::clear_area_() {
  int nx, ny, nw, nh;
  ppa_srm_rotation_angle_t angle;
  this->map_to_native_(this->area_x_, this->area_y_, this->area_w_, this->area_h_, &nx, &ny, &nw, &nh, &angle);
  const size_t bytes = static_cast<size_t>(nw) * nh * 2U;
  if (nx < 0 || ny < 0 || bytes > this->out_cap_)
    return false;
  std::memset(this->out_buf_, 0, bytes);
  this->display_->draw_pixels_at(nx, ny, nw, nh, this->out_buf_, display::COLOR_ORDER_RGB, display::COLOR_BITNESS_565,
                                 false, 0, 0, 0);
  this->area_clean_ = true;
  return true;
}

bool Tab5VideoView::present_() {
  const int cw = this->surf_w_, ch = this->surf_h_;
  const int aw = this->area_w_, ah = this->area_h_;
  if (cw <= 0 || ch <= 0)
    return false;

  // Fill the area if that cuts off no more than max_crop of the picture;
  // otherwise fit the whole picture with black bars.
  const int fill_units =
      std::max((aw * kScaleUnits + cw - 1) / cw, (ah * kScaleUnits + ch - 1) / ch);
  const int fit_units = std::min(aw * kScaleUnits / cw, ah * kScaleUnits / ch);
  const float kept_w = std::min(1.0f, static_cast<float>(aw) * kScaleUnits / fill_units / cw);
  const float kept_h = std::min(1.0f, static_cast<float>(ah) * kScaleUnits / fill_units / ch);
  const bool fill = (1.0f - kept_w) <= this->max_crop_ && (1.0f - kept_h) <= this->max_crop_;
  const int units = std::max(1, fill ? fill_units : fit_units);

  const int bw = std::min(cw, aw * kScaleUnits / units);
  const int bh = std::min(ch, ah * kScaleUnits / units);
  const int ow = bw * units / kScaleUnits;
  const int oh = bh * units / kScaleUnits;
  if (bw <= 0 || bh <= 0 || ow <= 0 || oh <= 0)
    return false;
  const int lx = this->area_x_ + (aw - ow) / 2;
  const int ly = this->area_y_ + (ah - oh) / 2;

  // New size or place (or first picture): black out the area first so no
  // old screen content stays around the picture.
  if (lx != this->shown_lx_ || ly != this->shown_ly_ || ow != this->shown_lw_ || oh != this->shown_lh_) {
    if ((ow < aw || oh < ah) && !this->clear_area_())
      return false;
    this->shown_lx_ = lx;
    this->shown_ly_ = ly;
    this->shown_lw_ = ow;
    this->shown_lh_ = oh;
    if (!this->logged_geometry_) {
      this->logged_geometry_ = true;
      ESP_LOGI(TAG, "Picture %dx%d shown as %dx%d at %d,%d (%s)", cw, ch, ow, oh, lx, ly,
               fill ? "filled" : "fitted");
    }
  }

  int nx, ny, nw, nh;
  ppa_srm_rotation_angle_t angle;
  this->map_to_native_(lx, ly, ow, oh, &nx, &ny, &nw, &nh, &angle);
  if (nx < 0 || ny < 0 || nx + nw > this->display_->get_native_width() ||
      ny + nh > this->display_->get_native_height() || static_cast<size_t>(nw) * nh * 2U > this->out_cap_)
    return false;

  ppa_srm_oper_config_t srm{};
  srm.in.buffer = this->surface_;
  srm.in.pic_w = this->surf_store_w_;
  srm.in.pic_h = this->surf_store_h_;
  srm.in.block_w = bw;
  srm.in.block_h = bh;
  srm.in.block_offset_x = (cw - bw) / 2;
  srm.in.block_offset_y = (ch - bh) / 2;
  srm.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
  srm.out.buffer = this->out_buf_;
  srm.out.buffer_size = this->out_cap_;
  srm.out.pic_w = nw;
  srm.out.pic_h = nh;
  srm.out.block_offset_x = 0;
  srm.out.block_offset_y = 0;
  srm.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
  srm.rotation_angle = angle;
  srm.scale_x = static_cast<float>(units) / kScaleUnits;
  srm.scale_y = srm.scale_x;
  srm.mode = PPA_TRANS_MODE_BLOCKING;
  const esp_err_t err = ppa_do_scale_rotate_mirror(this->ppa_, &srm);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "PPA scale/rotate failed: %s", esp_err_to_name(err));
    return false;
  }
  this->composite_self_(nx, ny, nw, nh);
  this->display_->draw_pixels_at(nx, ny, nw, nh, this->out_buf_, display::COLOR_ORDER_RGB, display::COLOR_BITNESS_565,
                                 false, 0, 0, 0);
  return true;
}

// ---------------- Still pictures ----------------

bool Tab5VideoView::start_picture_task_() {
  if (this->fetch_handle_ != nullptr)
    return true;
  if (this->pic_mutex_ == nullptr)
    this->pic_mutex_ = xSemaphoreCreateMutexStatic(&this->pic_mutex_storage_);
  if (this->pic_buf_ == nullptr) {
    this->pic_buf_ = static_cast<uint8_t *>(
        heap_caps_aligned_alloc(64, kMaxStillBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    this->pic_cap_ = this->pic_buf_ == nullptr ? 0 : kMaxStillBytes;
  }
  if (this->pic_mutex_ == nullptr || this->pic_buf_ == nullptr) {
    ESP_LOGE(TAG, "Not enough memory for still pictures");
    return false;
  }
  if (xTaskCreatePinnedToCoreWithCaps(Tab5VideoView::fetch_task_trampoline_, "tab5_picture", kFetchStackBytes, this,
                                      kFetchPriority, &this->fetch_handle_, tskNO_AFFINITY,
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
    this->fetch_handle_ = nullptr;
    ESP_LOGE(TAG, "Could not start the picture fetch task");
    return false;
  }
  return true;
}

bool Tab5VideoView::show_picture(const std::string &url) {
  if (this->is_failed() || url.empty() || !this->running_.load(std::memory_order_acquire))
    return false;
  if (this->prepared_.load(std::memory_order_acquire))
    return false;  // a call's video owns the area
  if (!this->start_picture_task_())
    return false;
  if (!this->picture_on_.load(std::memory_order_acquire)) {
    this->generation_.fetch_add(1, std::memory_order_acq_rel);
    this->shown_crc_.store(0, std::memory_order_release);
    this->picture_visible_.store(false, std::memory_order_release);
    this->pic_err_.store(PIC_OK, std::memory_order_release);
    this->pic_err_detail_.store(0, std::memory_order_release);
    this->picture_changed_ms_ = 0;
    this->pic_shown_w_ = this->pic_shown_h_ = 0;
    this->logged_geometry_ = false;
    this->end_presentation_();
    this->picture_on_.store(true, std::memory_order_release);
    ESP_LOGI(TAG, "Still picture on");
  }
  xSemaphoreTake(this->pic_mutex_, portMAX_DELAY);
  this->pic_url_ = url;
  xSemaphoreGive(this->pic_mutex_);
  xTaskNotifyGive(this->fetch_handle_);
  return true;
}

void Tab5VideoView::hide_picture() {
  if (!this->picture_on_.exchange(false, std::memory_order_acq_rel))
    return;
  this->generation_.fetch_add(1, std::memory_order_acq_rel);
  this->shown_crc_.store(0, std::memory_order_release);
  if (this->pic_mutex_ != nullptr) {
    xSemaphoreTake(this->pic_mutex_, portMAX_DELAY);
    this->pic_url_.clear();
    xSemaphoreGive(this->pic_mutex_);
  }
  this->picture_visible_.store(false, std::memory_order_release);
  // Whatever was shown, the screen must be drawn again.
  this->picture_ended_pending_.store(true, std::memory_order_release);
  this->enable_loop_soon_any_context();
  ESP_LOGI(TAG, "Still picture off");
}

std::string Tab5VideoView::picture_error() const {
  const int32_t d = this->pic_err_detail_.load(std::memory_order_acquire);
  char buf[96];
  switch (this->pic_err_.load(std::memory_order_acquire)) {
    case PIC_OK:
      return "";
    case PIC_BAD_URL:
      return "Picture address must start with http://";
    case PIC_NO_HOST:
      return "Can't find Home Assistant's address";
    case PIC_NO_CONNECT:
      return "Can't reach Home Assistant";
    case PIC_HTTP_STATUS:
      if (d == 401 || d == 403)
        snprintf(buf, sizeof(buf), "Home Assistant refused the picture (%d)", (int) d);
      else if (d == 404)
        snprintf(buf, sizeof(buf), "No such camera in Home Assistant (404)");
      else if (d >= 500)
        snprintf(buf, sizeof(buf), "Camera has no picture right now (%d)", (int) d);
      else
        snprintf(buf, sizeof(buf), "Home Assistant answered %d", (int) d);
      return buf;
    case PIC_BAD_REPLY:
      return "Strange answer from Home Assistant";
    case PIC_READ_FAILED:
      return "Picture download broke off";
    case PIC_TOO_MANY_BYTES:
      return "Picture file too big";
    case PIC_NOT_JPEG:
      return "Not a JPEG picture";
    case PIC_TOO_BIG:
      snprintf(buf, sizeof(buf), "Picture too big to show (%dx%d)", (int) ((uint32_t) d >> 16), (int) (d & 0xFFFF));
      return buf;
    case PIC_CANT_DECODE:
    default:
      return "Can't decode the picture";
  }
}

void Tab5VideoView::set_picture_error_(uint8_t code, int32_t detail) {
  const uint8_t old = this->pic_err_.exchange(code, std::memory_order_acq_rel);
  const int32_t old_detail = this->pic_err_detail_.exchange(detail, std::memory_order_acq_rel);
  if (old != code || old_detail != detail) {
    if (code != PIC_OK) {
      const std::string text = this->picture_error();
      ESP_LOGW(TAG, "Still picture: %s", text.c_str());
    }
    this->picture_note_pending_.store(true, std::memory_order_release);
    this->enable_loop_soon_any_context();
  }
}

void Tab5VideoView::fetch_task_trampoline_(void *arg) { static_cast<Tab5VideoView *>(arg)->fetch_task_(); }

void Tab5VideoView::fetch_task_() {
  while (this->running_.load(std::memory_order_acquire)) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (!this->running_.load(std::memory_order_acquire))
      break;
    if (!this->picture_on_.load(std::memory_order_acquire))
      continue;
    const uint32_t generation = this->generation_.load(std::memory_order_acquire);
    xSemaphoreTake(this->pic_mutex_, portMAX_DELAY);
    const std::string url = this->pic_url_;
    xSemaphoreGive(this->pic_mutex_);
    if (!url.empty())
      this->fetch_picture_(url, generation);
  }
  vTaskSuspend(nullptr);
}

namespace {

// Reads until "\r\n\r\n" or the buffer is full. Returns the bytes read,
// and where the body starts (0 = end of the headers not found).
int read_headers(int sock, char *buf, int cap, int *body_at) {
  int got = 0;
  *body_at = 0;
  while (got < cap) {
    const int r = lwip_recv(sock, buf + got, cap - got, 0);
    if (r <= 0)
      return got;
    got += r;
    for (int i = std::max(0, got - r - 3); i + 3 < got; i++) {
      if (buf[i] == '\r' && buf[i + 1] == '\n' && buf[i + 2] == '\r' && buf[i + 3] == '\n') {
        *body_at = i + 4;
        return got;
      }
    }
  }
  return got;
}

// Value of header "name" (lower case, with the colon) in the header block,
// or "" if missing.
std::string header_value(const char *headers, int len, const char *name) {
  const size_t name_len = strlen(name);
  int line = 0;
  while (line < len) {
    int end = line;
    while (end < len && headers[end] != '\n')
      end++;
    if (static_cast<size_t>(end - line) > name_len) {
      bool match = true;
      for (size_t k = 0; k < name_len && match; k++)
        match = std::tolower(static_cast<unsigned char>(headers[line + k])) == name[k];
      if (match) {
        int v = line + static_cast<int>(name_len);
        while (v < end && (headers[v] == ' ' || headers[v] == '\t'))
          v++;
        int e = end;
        while (e > v && (headers[e - 1] == '\r' || headers[e - 1] == ' '))
          e--;
        return std::string(headers + v, e - v);
      }
    }
    line = end + 1;
  }
  return "";
}

}  // namespace

void Tab5VideoView::fetch_picture_(const std::string &url, uint32_t generation) {
  // Wait (briefly) for the decoder to take the last picture.
  for (int i = 0; i < 40 && this->pic_state_.load(std::memory_order_acquire) != 0; i++)
    vTaskDelay(pdMS_TO_TICKS(50));
  uint8_t expected = 0;
  if (!this->pic_state_.compare_exchange_strong(expected, 2, std::memory_order_acq_rel))
    return;  // still busy: the next request tries again

  // http://host[:port]/path?query (no https: this stays on the home network)
  std::string host, path = "/";
  int port = 80;
  if (url.compare(0, 7, "http://") != 0) {
    this->pic_state_.store(0, std::memory_order_release);
    this->set_picture_error_(PIC_BAD_URL, 0);
    return;
  }
  {
    const size_t start = 7;
    size_t slash = url.find('/', start);
    std::string hostport = url.substr(start, slash == std::string::npos ? std::string::npos : slash - start);
    if (slash != std::string::npos)
      path = url.substr(slash);
    const size_t colon = hostport.rfind(':');
    if (colon != std::string::npos) {
      port = atoi(hostport.c_str() + colon + 1);
      hostport.resize(colon);
    }
    host = hostport;
  }
  if (host.empty() || port <= 0 || port > 65535) {
    this->pic_state_.store(0, std::memory_order_release);
    this->set_picture_error_(PIC_BAD_URL, 0);
    return;
  }

  uint8_t err = PIC_OK;
  int32_t detail = 0;
  size_t got = 0;
  int sock = -1;

  struct addrinfo hints {};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  struct addrinfo *res = nullptr;
  char port_str[8];
  snprintf(port_str, sizeof(port_str), "%d", port);
  if (lwip_getaddrinfo(host.c_str(), port_str, &hints, &res) != 0 || res == nullptr) {
    err = PIC_NO_HOST;
  } else {
    sock = lwip_socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (sock < 0) {
      err = PIC_NO_CONNECT;
    } else {
      // Connect with a time limit, then plain blocking reads with one.
      int flags = lwip_fcntl(sock, F_GETFL, 0);
      lwip_fcntl(sock, F_SETFL, flags | O_NONBLOCK);
      int rc = lwip_connect(sock, res->ai_addr, res->ai_addrlen);
      if (rc != 0 && errno == EINPROGRESS) {
        fd_set wfds;
        FD_ZERO(&wfds);
        FD_SET(sock, &wfds);
        struct timeval tv {};
        tv.tv_sec = kConnectTimeoutMs / 1000;
        tv.tv_usec = (kConnectTimeoutMs % 1000) * 1000;
        rc = lwip_select(sock + 1, nullptr, &wfds, nullptr, &tv) == 1 ? 0 : -1;
        if (rc == 0) {
          int so_error = 0;
          socklen_t len = sizeof(so_error);
          lwip_getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_error, &len);
          rc = so_error == 0 ? 0 : -1;
        }
      }
      lwip_fcntl(sock, F_SETFL, flags);
      if (rc != 0)
        err = PIC_NO_CONNECT;
    }
  }
  if (res != nullptr)
    lwip_freeaddrinfo(res);

  if (err == PIC_OK) {
    struct timeval tv {};
    tv.tv_sec = kReadTimeoutMs / 1000;
    lwip_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    lwip_setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    // HTTP/1.0: the answer comes in one piece (no chunks) and the
    // connection closes after it.
    std::string request = "GET " + path + " HTTP/1.0\r\nHost: " + host;
    if (port != 80)
      request += ":" + std::to_string(port);
    request += "\r\nAccept: image/jpeg\r\nConnection: close\r\nUser-Agent: tab5_video\r\n\r\n";
    if (lwip_send(sock, request.data(), request.size(), 0) != static_cast<int>(request.size())) {
      err = PIC_NO_CONNECT;
    }
  }

  if (err == PIC_OK) {
    char head[2048];
    int body_at = 0;
    const int head_len = read_headers(sock, head, sizeof(head), &body_at);
    // Status line: "HTTP/1.1 200 OK"
    int status = 0;
    if (body_at > 12 && std::memcmp(head, "HTTP/", 5) == 0) {
      const char *space = static_cast<const char *>(std::memchr(head, ' ', body_at));
      if (space != nullptr)
        status = atoi(space + 1);
    }
    if (body_at == 0 || status <= 0) {
      err = head_len <= 0 ? PIC_READ_FAILED : PIC_BAD_REPLY;
    } else if (status != 200) {
      err = PIC_HTTP_STATUS;
      detail = status;
    } else if (!header_value(head, body_at, "transfer-encoding:").empty()) {
      err = PIC_BAD_REPLY;
    } else {
      const std::string cl = header_value(head, body_at, "content-length:");
      const long expected_len = cl.empty() ? -1 : atol(cl.c_str());
      if (expected_len > static_cast<long>(this->pic_cap_)) {
        err = PIC_TOO_MANY_BYTES;
      } else {
        got = static_cast<size_t>(head_len - body_at);
        std::memcpy(this->pic_buf_, head + body_at, got);
        while (got < this->pic_cap_ && (expected_len < 0 || got < static_cast<size_t>(expected_len))) {
          const int r = lwip_recv(sock, this->pic_buf_ + got, this->pic_cap_ - got, 0);
          if (r <= 0)
            break;
          got += r;
        }
        if (expected_len >= 0 && got < static_cast<size_t>(expected_len)) {
          err = PIC_READ_FAILED;
        } else if (expected_len < 0 && got >= this->pic_cap_) {
          err = PIC_TOO_MANY_BYTES;
        } else if (got < 4 || this->pic_buf_[0] != 0xFF || this->pic_buf_[1] != 0xD8) {
          err = PIC_NOT_JPEG;
        }
      }
    }
  }
  if (sock >= 0)
    lwip_close(sock);

  if (err != PIC_OK) {
    this->pic_state_.store(0, std::memory_order_release);
    this->set_picture_error_(err, detail);
    return;
  }
  if (!this->picture_on_.load(std::memory_order_acquire) ||
      generation != this->generation_.load(std::memory_order_acquire)) {
    this->pic_state_.store(0, std::memory_order_release);  // turned off meanwhile
    return;
  }
  const uint32_t crc = esp_rom_crc32_le(0, this->pic_buf_, got) | 1U;  // never 0 (0 = nothing shown)
  if (crc == this->shown_crc_.load(std::memory_order_acquire)) {
    // Same picture as on the screen: nothing to do.
    this->pic_state_.store(0, std::memory_order_release);
    this->set_picture_error_(PIC_OK, 0);
    return;
  }
  this->pic_size_ = got;
  this->pic_generation_ = generation;
  this->pic_crc_ = crc;
  this->pic_state_.store(1, std::memory_order_release);
  this->set_picture_error_(PIC_OK, 0);
  if (this->task_ != nullptr)
    xTaskNotifyGive(this->task_);
}

void Tab5VideoView::decode_picture_() {
  if (this->pic_state_.load(std::memory_order_acquire) != 1 ||
      this->surface_state_.load(std::memory_order_acquire) != 0)
    return;
  uint8_t expected = 1;
  if (!this->pic_state_.compare_exchange_strong(expected, 2, std::memory_order_acq_rel))
    return;
  const uint32_t generation = this->pic_generation_;
  auto wanted = [&]() {
    return this->picture_on_.load(std::memory_order_acquire) && !this->prepared_.load(std::memory_order_acquire) &&
           generation == this->generation_.load(std::memory_order_acquire);
  };
  if (!wanted()) {
    this->pic_state_.store(0, std::memory_order_release);
    return;
  }

  jpeg_decode_picture_info_t info{};
  if (jpeg_decoder_get_info(this->pic_buf_, static_cast<uint32_t>(this->pic_size_), &info) != ESP_OK ||
      info.sample_method == JPEG_DOWN_SAMPLING_GRAY || info.width == 0 || info.height == 0) {
    this->pic_state_.store(0, std::memory_order_release);
    this->set_picture_error_(PIC_CANT_DECODE, 0);
    return;
  }
  const uint32_t store_w = align_up(info.width, info.sample_method == JPEG_DOWN_SAMPLING_YUV444 ? 8 : 16);
  const uint32_t store_h = align_up(info.height, info.sample_method == JPEG_DOWN_SAMPLING_YUV420 ? 16 : 8);
  const uint32_t need = store_w * store_h * 2U;
  if (info.width > this->max_decode_w_ || info.height > this->max_decode_h_ || need > this->surface_cap_) {
    this->pic_state_.store(0, std::memory_order_release);
    this->set_picture_error_(PIC_TOO_BIG, static_cast<int32_t>((info.width << 16) | (info.height & 0xFFFF)));
    return;
  }
  this->surface_state_.store(1, std::memory_order_release);
  uint32_t decoded = 0;
  const esp_err_t err = jpeg_decoder_process(this->decoder_, &this->decode_cfg_, this->pic_buf_,
                                             static_cast<uint32_t>(this->pic_size_), this->surface_, need, &decoded);
  const uint32_t crc = this->pic_crc_;
  this->pic_state_.store(0, std::memory_order_release);
  if (err == ESP_OK && decoded > 0 && wanted()) {
    this->surf_w_ = static_cast<uint16_t>(info.width);
    this->surf_h_ = static_cast<uint16_t>(info.height);
    this->surf_store_w_ = static_cast<uint16_t>(store_w);
    this->surf_store_h_ = static_cast<uint16_t>(store_h);
    this->surface_generation_ = generation;
    this->surface_is_picture_ = true;
    this->surface_crc_ = crc;
    this->surface_state_.store(2, std::memory_order_release);
    this->enable_loop_soon_any_context();
    return;
  }
  this->surface_state_.store(0, std::memory_order_release);
  if (err != ESP_OK)
    this->set_picture_error_(PIC_CANT_DECODE, err);
}

// ---------------- Camera gate ----------------

Tab5CameraGate::Tab5CameraGate() { this->mutex_ = xSemaphoreCreateMutexStatic(&this->mutex_storage_); }

voip_stack::VideoCapability Tab5CameraGate::get_video_capability() const {
  return this->source_ == nullptr ? voip_stack::VideoCapability{} : this->source_->get_video_capability();
}

bool Tab5CameraGate::prepare_video(const voip_stack::VideoCapability &capability) {
  // Same checks as the camera source, without asking it (it refuses while
  // it runs, and it is not running yet here).
  const voip_stack::VideoCapability own = this->get_video_capability();
  return this->source_ != nullptr && capability.valid() && capability.is_jpeg() && capability.width == own.width &&
         capability.height == own.height;
}

bool Tab5CameraGate::start_video(voip_stack::EncodedVideoAccessUnitCallback callback, void *ctx,
                                 const voip_stack::VideoCapability &capability) {
  if (callback == nullptr || !this->prepare_video(capability))
    return false;
  xSemaphoreTake(this->mutex_, portMAX_DELAY);
  if (this->source_running_) {
    this->source_->stop_video();
    this->source_running_ = false;
  }
  this->callback_ = callback;
  this->callback_ctx_ = ctx;
  this->capability_ = capability;
  // Every call's video starts with the camera off.
  this->enabled_.store(false, std::memory_order_release);
  this->session_.store(true, std::memory_order_release);
  xSemaphoreGive(this->mutex_);
  ESP_LOGI(TAG, "Video can be sent on this call (camera off until switched on)");
  return true;
}

void Tab5CameraGate::stop_video() {
  xSemaphoreTake(this->mutex_, portMAX_DELAY);
  const bool was = this->session_.exchange(false, std::memory_order_acq_rel);
  this->enabled_.store(false, std::memory_order_release);
  if (this->source_running_) {
    this->source_->stop_video();
    this->source_running_ = false;
  }
  this->callback_ = nullptr;
  this->callback_ctx_ = nullptr;
  xSemaphoreGive(this->mutex_);
  if (was)
    ESP_LOGI(TAG, "Call video over: camera off");
}

bool Tab5CameraGate::set_enabled(bool enabled) {
  if (this->source_ == nullptr)
    return false;
  xSemaphoreTake(this->mutex_, portMAX_DELAY);
  bool ok = true;
  if (!enabled) {
    if (this->source_running_) {
      this->source_->stop_video();
      this->source_running_ = false;
    }
    this->enabled_.store(false, std::memory_order_release);
  } else if (!this->session_.load(std::memory_order_acquire) || this->callback_ == nullptr) {
    ok = false;  // no call with video: nothing to send to
  } else {
    if (!this->source_running_)
      this->source_running_ = this->source_->start_video(Tab5CameraGate::forward_, this, this->capability_);
    ok = this->source_running_;
    this->enabled_.store(ok, std::memory_order_release);
  }
  xSemaphoreGive(this->mutex_);
  ESP_LOGI(TAG, "Camera %s%s", enabled ? "on" : "off", ok ? "" : " refused (no video on this call)");
  return ok;
}

void Tab5CameraGate::forward_(void *ctx, const voip_stack::EncodedVideoAccessUnit &access_unit) {
  // Camera task. The camera source stops calling this before stop_video()
  // or set_enabled(false) clear the callback, so no lock is needed here.
  auto *gate = static_cast<Tab5CameraGate *>(ctx);
  const auto callback = gate->callback_;
  if (callback != nullptr)
    callback(gate->callback_ctx_, access_unit);
  if (gate->self_view_ != nullptr)
    gate->self_view_->offer_self_frame(access_unit);
}

void Tab5CameraGate::request_key_frame() {
  // JPEG pictures stand alone; pass it on anyway.
  if (this->source_ != nullptr && this->enabled_.load(std::memory_order_acquire))
    this->source_->request_key_frame();
}

}  // namespace esphome::tab5_video

#endif
