#pragma once

#include "esphome/core/defines.h"

#if defined(USE_ESP_IDF) && defined(USE_ESPHOME_VOIP_STACK_VIDEO_JPEG)

#include "esphome/components/display/display.h"
#include "esphome/components/voip_stack/video.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"

#include "driver/jpeg_decode.h"
#include "driver/ppa.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

namespace esphome::tab5_video {

/// Receives the other side's JPEG pictures from voip_stack and shows them
/// in one rectangle of a plain ESPHome display.
///
/// Threads:
///  - voip_stack's video RTP task calls consume_video_access_unit(): the
///    picture is copied into one slot (a busy slot drops the picture; JPEG
///    pictures stand alone, so nothing else is lost).
///  - this component's decode task turns the slot into RGB565 with the
///    hardware JPEG decoder.
///  - loop() (the ESPHome main task) scales and rotates it with the PPA and
///    writes it to the panel. All panel writes happen on the main task, the
///    same task that runs the display lambda, so the two never overlap.
///  - still pictures (show_picture()) are fetched over plain HTTP by a
///    second small task, then take the same decode and display path while
///    no call video is set up.
class Tab5VideoView : public Component, public voip_stack::EncodedVideoSink {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  void on_shutdown() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

  void set_display(display::Display *display) { this->display_ = display; }
  void set_display_rotation(int rotation) { this->display_rotation_ = rotation; }
  void set_area(int x, int y, int w, int h) {
    this->area_x_ = x;
    this->area_y_ = y;
    this->area_w_ = w;
    this->area_h_ = h;
  }
  void set_receive_size(uint16_t w, uint16_t h) {
    this->rx_width_ = w;
    this->rx_height_ = h;
  }
  void set_framerate(uint8_t fps) { this->framerate_ = fps; }
  void set_max_decode(uint16_t w, uint16_t h) {
    this->max_decode_w_ = w;
    this->max_decode_h_ = h;
  }
  void set_max_crop(float share) { this->max_crop_ = share; }
  void set_swap_red_blue(bool swap) { this->swap_rb_ = swap; }
  void set_no_video_timeout(uint32_t ms) { this->no_video_timeout_ms_ = ms; }
  /// Self view: this device's own camera pictures, drawn small. Turned on
  /// by the camera: block (it sends each picture to offer_self_frame()).
  void set_self_view_source_size(uint16_t w, uint16_t h) {
    this->self_enabled_ = true;
    this->self_max_w_ = w;
    this->self_max_h_ = h;
  }
  void set_self_view_mirror(bool mirror) { this->self_mirror_ = mirror; }
  void set_self_view_framerate(uint8_t fps) { this->self_fps_ = fps; }

  Trigger<> *get_first_frame_trigger() { return &this->first_frame_trigger_; }
  Trigger<> *get_video_ended_trigger() { return &this->video_ended_trigger_; }
  /// Fires on the main loop when the still picture changes on the screen,
  /// when fetching it starts or stops failing, and when it goes away.
  Trigger<> *get_picture_trigger() { return &this->picture_trigger_; }

  // voip_stack::EncodedVideoSink
  voip_stack::VideoCapability get_receive_video_capability() const override;
  bool start_video(const voip_stack::VideoCapability &capability) override;
  bool set_video_active(bool active) override;
  void stop_video() override;
  bool consume_video_access_unit(
      const voip_stack::EncodedVideoAccessUnit &access_unit) override;

  /// True while the other side's picture is on the screen.
  bool has_frame() const { return this->visible_.load(std::memory_order_acquire); }
  /// True while a video stream from the other side is set up.
  bool is_receiving() const { return this->active_.load(std::memory_order_acquire); }
  /// Pictures put on the screen since the current video started.
  uint32_t frames_shown() const { return this->frames_shown_.load(std::memory_order_relaxed); }
  /// Pictures that could not be decoded since the current video started.
  uint32_t decode_errors() const { return this->decode_errors_.load(std::memory_order_relaxed); }
  /// Stop putting pictures on the screen (something else covers the area).
  /// The next picture after un-pausing clears the area first.
  void set_paused(bool paused);
  bool is_paused() const { return this->paused_.load(std::memory_order_acquire); }

  /// Where to draw this device's own picture (screen coordinates); a width
  /// or height of 0 hides it. Call it from the display lambda on every
  /// drawing: the picture is drawn again right after the screen update, so
  /// the lambda can draw anything under it. The picture keeps its shape and
  /// is centred in the rectangle.
  void set_self_view(int x, int y, int w, int h);
  /// The rectangle last given to set_self_view().
  void get_self_view(int *x, int *y, int *w, int *h) const {
    *x = this->self_rx_;
    *y = this->self_ry_;
    *w = this->self_rw_;
    *h = this->self_rh_;
  }
  /// Called by Tab5CameraGate, on the camera's task, with each picture sent.
  void offer_self_frame(const voip_stack::EncodedVideoAccessUnit &access_unit);

  // ---- Still pictures (for example a doorbell camera) ----
  /// Fetch one JPEG picture from an http:// address and show it in the area,
  /// as long as no call video is set up. Call it again to look for a newer
  /// picture: the screen only changes when the picture did. Main loop only.
  /// Returns false (nothing done) while a call's video owns the area.
  bool show_picture(const std::string &url);
  /// Take the still picture off and stop fetching. A call's video starting
  /// does this too.
  void hide_picture();
  /// True from show_picture() until hide_picture().
  bool picture_mode() const { return this->picture_on_.load(std::memory_order_acquire); }
  /// True while a still picture is on the screen.
  bool has_picture() const { return this->picture_visible_.load(std::memory_order_acquire); }
  /// What went wrong with the last fetch, or "" when it worked.
  std::string picture_error() const;
  /// millis() when the still picture on the screen last changed, 0 = none.
  uint32_t picture_changed_ms() const { return this->picture_changed_ms_; }
  /// Size of the last still picture shown.
  int picture_width() const { return this->pic_shown_w_; }
  int picture_height() const { return this->pic_shown_h_; }

 protected:
  static void decode_task_trampoline_(void *arg);
  void decode_task_();
  void decode_remote_();
  void decode_self_();
  bool present_();
  bool render_thumb_();
  bool draw_self_();
  void composite_self_(int pic_nx, int pic_ny, int pic_nw, int pic_nh);
  bool thumb_place_(int *lx, int *ly) const;
  bool clear_area_();
  void check_stale_();
  bool start_picture_task_();
  static void fetch_task_trampoline_(void *arg);
  void fetch_task_();
  void fetch_picture_(const std::string &url, uint32_t generation);
  void decode_picture_();
  void set_picture_error_(uint8_t code, int32_t detail);
  void end_presentation_();
  void map_to_native_(int lx, int ly, int lw, int lh, int *nx, int *ny, int *nw,
                      int *nh, ppa_srm_rotation_angle_t *angle) const;

  display::Display *display_{nullptr};
  int display_rotation_{0};
  int area_x_{0}, area_y_{0}, area_w_{0}, area_h_{0};
  uint16_t rx_width_{640}, rx_height_{360};
  uint8_t framerate_{10};
  uint16_t max_decode_w_{1280}, max_decode_h_{720};
  float max_crop_{0.15f};
  bool swap_rb_{false};
  uint32_t no_video_timeout_ms_{3000};
  uint32_t last_shown_ms_{0};

  jpeg_decoder_handle_t decoder_{nullptr};
  jpeg_decode_cfg_t decode_cfg_{};
  ppa_client_handle_t ppa_{nullptr};

  // Encoded picture slot: 0 free, 1 full, 2 owned by a writer/reader.
  uint8_t *au_buf_{nullptr};
  size_t au_cap_{0};
  size_t au_size_{0};
  uint32_t au_generation_{0};
  std::atomic<uint8_t> au_state_{0};

  // Decoded picture: 0 free, 1 decoding, 2 ready for loop().
  uint8_t *surface_{nullptr};
  size_t surface_cap_{0};
  uint16_t surf_w_{0}, surf_h_{0};          // picture size
  uint16_t surf_store_w_{0}, surf_store_h_{0};  // stored size (JPEG blocks)
  uint32_t surface_generation_{0};
  std::atomic<uint8_t> surface_state_{0};

  // Scaled and rotated picture handed to the panel.
  uint8_t *out_buf_{nullptr};
  size_t out_cap_{0};
  size_t out_align_{64};

  // Self view. Rectangle and thumbnail belong to the main loop; the camera
  // task only reads self_visible_.
  bool self_enabled_{false};
  bool self_mirror_{true};
  uint8_t self_fps_{10};
  uint16_t self_max_w_{640}, self_max_h_{360};
  int self_rx_{0}, self_ry_{0}, self_rw_{0}, self_rh_{0};
  std::atomic<bool> self_visible_{false};
  bool self_redraw_{false};
  voip_stack::RtpFrameCadence90k self_cadence_;
  uint8_t *self_au_buf_{nullptr};
  size_t self_au_cap_{0};
  size_t self_au_size_{0};
  std::atomic<uint8_t> self_au_state_{0};  // 0 free, 1 full, 2 owned
  uint8_t *self_surface_{nullptr};
  size_t self_surface_cap_{0};
  uint16_t self_w_px_{0}, self_h_px_{0}, self_store_w_{0}, self_store_h_{0};
  std::atomic<uint8_t> self_surface_state_{0};  // 0 free, 1 decoding, 2 ready
  // Scaled and mirrored picture (screen orientation), and the same turned
  // for the panel.
  uint8_t *thumb_{nullptr};
  uint8_t *thumb_native_{nullptr};
  size_t thumb_cap_{0};
  int thumb_w_{0}, thumb_h_{0};
  bool thumb_valid_{false};

  // What is on the screen now (to know when the area must be cleared).
  int shown_lx_{-1}, shown_ly_{-1}, shown_lw_{0}, shown_lh_{0};
  bool area_clean_{false};

  TaskHandle_t task_{nullptr};
  SemaphoreHandle_t task_done_{nullptr};
  std::atomic<bool> running_{false};
  std::atomic<bool> prepared_{false};
  std::atomic<bool> active_{false};
  std::atomic<bool> visible_{false};
  std::atomic<bool> paused_{false};
  std::atomic<bool> first_frame_pending_{false};
  std::atomic<bool> ended_pending_{false};
  std::atomic<uint32_t> generation_{0};
  std::atomic<uint32_t> frames_shown_{0};
  std::atomic<uint32_t> frames_decoded_{0};
  std::atomic<uint32_t> decode_errors_{0};
  voip_stack::RtpFrameCadence90k cadence_;
  uint8_t negotiated_fps_{10};
  bool logged_geometry_{false};

  // Still pictures. The address is guarded by pic_mutex_; the fetched
  // picture slot works like au_state_ (0 free, 1 full, 2 owned).
  SemaphoreHandle_t pic_mutex_{nullptr};
  StaticSemaphore_t pic_mutex_storage_{};
  std::string pic_url_;
  TaskHandle_t fetch_handle_{nullptr};
  uint8_t *pic_buf_{nullptr};
  size_t pic_cap_{0};
  size_t pic_size_{0};
  uint32_t pic_generation_{0};
  uint32_t pic_crc_{0};
  std::atomic<uint8_t> pic_state_{0};
  std::atomic<bool> picture_on_{false};
  std::atomic<bool> picture_visible_{false};
  std::atomic<bool> picture_ended_pending_{false};
  std::atomic<bool> picture_note_pending_{false};
  // Checksum of the picture on the screen (0 = none), so an unchanged
  // picture is not decoded and drawn again.
  std::atomic<uint32_t> shown_crc_{0};
  std::atomic<uint8_t> pic_err_{0};
  std::atomic<int32_t> pic_err_detail_{0};
  uint32_t picture_changed_ms_{0};
  int pic_shown_w_{0}, pic_shown_h_{0};
  // What the decoded surface holds: a still picture (and its checksum) or
  // a call picture.
  bool surface_is_picture_{false};
  uint32_t surface_crc_{0};

  Trigger<> first_frame_trigger_;
  Trigger<> video_ended_trigger_;
  Trigger<> picture_trigger_;
};

/// Stands between the camera's JPEG source and voip_stack, so the camera can
/// be switched on and off during a call without SIP changes.
///
/// voip_stack only lets a call change video direction if video was agreed
/// when the call started, and it agrees "send" only while sending is wanted.
/// So voip_stack is left wanting to send, and this gate decides whether the
/// camera actually runs: it starts closed for every call, opens only when
/// asked during that call, and closes again when the call's video ends.
/// While closed the camera is stopped and no pictures leave the device.
class Tab5CameraGate : public voip_stack::EncodedVideoSource {
 public:
  Tab5CameraGate();
  void set_source(voip_stack::EncodedVideoSource *source) { this->source_ = source; }
  /// Also hand every picture sent to this view's self view.
  void set_self_view(Tab5VideoView *view) { this->self_view_ = view; }

  /// Open (true) or close (false) the gate. Opening only works while a
  /// call's video is set up; returns false otherwise.
  bool set_enabled(bool enabled);
  /// True while pictures are being sent.
  bool is_enabled() const { return this->enabled_.load(std::memory_order_acquire); }
  /// True while a call's video is set up (the camera could be switched on).
  bool is_available() const { return this->session_.load(std::memory_order_acquire); }

  // voip_stack::EncodedVideoSource
  voip_stack::VideoCapability get_video_capability() const override;
  bool prepare_video(const voip_stack::VideoCapability &capability) override;
  bool start_video(voip_stack::EncodedVideoAccessUnitCallback callback, void *ctx,
                   const voip_stack::VideoCapability &capability) override;
  void stop_video() override;
  void request_key_frame() override;

 protected:
  static void forward_(void *ctx, const voip_stack::EncodedVideoAccessUnit &access_unit);

  voip_stack::EncodedVideoSource *source_{nullptr};
  Tab5VideoView *self_view_{nullptr};
  SemaphoreHandle_t mutex_{nullptr};
  StaticSemaphore_t mutex_storage_{};
  voip_stack::EncodedVideoAccessUnitCallback callback_{nullptr};
  void *callback_ctx_{nullptr};
  voip_stack::VideoCapability capability_{};
  bool source_running_{false};
  std::atomic<bool> session_{false};
  std::atomic<bool> enabled_{false};
};

}  // namespace esphome::tab5_video

#endif
