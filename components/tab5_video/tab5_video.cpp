#include "tab5_video.h"

#if defined(USE_ESP_IDF) && defined(USE_ESPHOME_VOIP_STACK_VIDEO_JPEG)

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include "esp_heap_caps.h"

#include <algorithm>
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
  if (this->is_failed())
    ESP_LOGCONFIG(TAG, "  FAILED to start (see the log above)");
}

void Tab5VideoView::on_shutdown() {
  this->active_.store(false, std::memory_order_release);
  this->prepared_.store(false, std::memory_order_release);
  if (this->task_ != nullptr && this->running_.exchange(false, std::memory_order_acq_rel)) {
    xTaskNotifyGive(this->task_);
    xSemaphoreTake(this->task_done_, pdMS_TO_TICKS(500));
  }
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
  uint32_t last_error_log = 0;
  while (this->running_.load(std::memory_order_acquire)) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (!this->running_.load(std::memory_order_acquire))
      break;
    // The picture waits in its slot until loop() has taken the last one.
    if (this->au_state_.load(std::memory_order_acquire) != 1 ||
        this->surface_state_.load(std::memory_order_acquire) != 0)
      continue;
    uint8_t expected = 1;
    if (!this->au_state_.compare_exchange_strong(expected, 2, std::memory_order_acq_rel))
      continue;

    const uint32_t generation = this->au_generation_;
    if (!this->active_.load(std::memory_order_acquire) ||
        generation != this->generation_.load(std::memory_order_acquire)) {
      this->au_state_.store(0, std::memory_order_release);
      continue;
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
  xSemaphoreGive(this->task_done_);
  vTaskSuspend(nullptr);
}

void Tab5VideoView::loop() {
  // Clear the wake-up first, so one that arrives while this runs is kept.
  this->disable_loop();

  if (this->ended_pending_.exchange(false, std::memory_order_acq_rel)) {
    this->end_presentation_();
    this->video_ended_trigger_.trigger();
  }

  if (this->surface_state_.load(std::memory_order_acquire) == 2) {
    const bool current = this->active_.load(std::memory_order_acquire) &&
                         this->surface_generation_ == this->generation_.load(std::memory_order_acquire);
    bool first = false;
    if (current && !this->paused_.load(std::memory_order_acquire) && this->present_()) {
      this->frames_shown_.fetch_add(1, std::memory_order_relaxed);
      this->last_shown_ms_ = millis();
      first = !this->visible_.exchange(true, std::memory_order_acq_rel);
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
  this->display_->draw_pixels_at(nx, ny, nw, nh, this->out_buf_, display::COLOR_ORDER_RGB, display::COLOR_BITNESS_565,
                                 false, 0, 0, 0);
  return true;
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
      this->source_running_ = this->source_->start_video(this->callback_, this->callback_ctx_, this->capability_);
    ok = this->source_running_;
    this->enabled_.store(ok, std::memory_order_release);
  }
  xSemaphoreGive(this->mutex_);
  ESP_LOGI(TAG, "Camera %s%s", enabled ? "on" : "off", ok ? "" : " refused (no video on this call)");
  return ok;
}

void Tab5CameraGate::request_key_frame() {
  // JPEG pictures stand alone; pass it on anyway.
  if (this->source_ != nullptr && this->enabled_.load(std::memory_order_acquire))
    this->source_->request_key_frame();
}

}  // namespace esphome::tab5_video

#endif
