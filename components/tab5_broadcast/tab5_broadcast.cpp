#include "tab5_broadcast.h"

#ifdef USE_ESP_IDF

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#ifdef USE_TAB5_BROADCAST_SERVER
#include "esphome/components/network/util.h"

#include <esp_heap_caps.h>
#include <lwip/sockets.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#endif

namespace esphome::tab5_broadcast {

static const char *const TAG = "tab5_broadcast";

#ifdef USE_TAB5_BROADCAST_SERVER

static constexpr size_t WAV_HEADER_SIZE = 44;
static constexpr const char *const CLIP_PATH = "/broadcast/";
// The server's own control port (each ESP-IDF HTTP server needs a free one;
// ESPHome's web server uses the default 32768).
static constexpr uint16_t CTRL_PORT = 32791;

static void put16(uint8_t *p, uint16_t v) {
  p[0] = v & 0xFF;
  p[1] = v >> 8;
}
static void put32(uint8_t *p, uint32_t v) {
  for (int i = 0; i < 4; i++)
    p[i] = (v >> (8 * i)) & 0xFF;
}

ClipServer::Clip::~Clip() {
  if (this->data != nullptr)
    heap_caps_free(this->data);
}

void ClipServer::setup() {
  if (!this->start_server_()) {
    this->status_set_warning();
  }
}

void ClipServer::loop() {
  if (this->server_ != nullptr) {
    this->disable_loop();
    return;
  }
  // The server didn't start (network not ready yet?): try again every 10 s.
  const uint32_t now = millis();
  if (now - this->last_try_ >= 10000 && this->start_server_()) {
    this->status_clear_warning();
  }
}

bool ClipServer::start_server_() {
  this->last_try_ = millis();
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = this->port_;
  config.ctrl_port = CTRL_PORT;
  config.max_open_sockets = 2;
  config.max_uri_handlers = 1;
  config.lru_purge_enable = true;
  config.uri_match_fn = httpd_uri_match_wildcard;
  // Low priority: sending a clip must never get in the way of call audio.
  config.task_priority = 2;
  config.stack_size = 4096;
  config.task_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
  // The other Tab5 reads the clip about as fast as it plays it.
  config.send_wait_timeout = 10;
  config.recv_wait_timeout = 5;

  httpd_handle_t server = nullptr;
  esp_err_t err = httpd_start(&server, &config);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Clip server didn't start on port %u: %s", this->port_, esp_err_to_name(err));
    return false;
  }
  httpd_uri_t clip_uri = {};
  clip_uri.uri = "/broadcast/*";
  clip_uri.method = HTTP_GET;
  clip_uri.handler = ClipServer::handle_get_;
  clip_uri.user_ctx = this;
  err = httpd_register_uri_handler(server, &clip_uri);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Clip server: %s", esp_err_to_name(err));
    httpd_stop(server);
    return false;
  }
  this->server_ = server;
  ESP_LOGI(TAG, "Clip server on port %u", this->port_);
  return true;
}

void ClipServer::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Tab5 broadcast clip server:\n"
                "  Port: %u\n"
                "  Running: %s",
                this->port_, YESNO(this->server_ != nullptr));
}

void ClipServer::on_shutdown() {
  if (this->server_ != nullptr) {
    httpd_stop(this->server_);
    this->server_ = nullptr;
  }
}

std::string ClipServer::publish_clip(const int16_t *samples, size_t count, uint32_t sample_rate) {
  if (this->server_ == nullptr || samples == nullptr || count == 0 || sample_rate == 0)
    return "";

  // This device's address on the home network.
  char ip[network::IP_ADDRESS_BUFFER_SIZE] = {0};
  for (const auto &addr : network::get_ip_addresses()) {
    if (addr.is_set() && addr.is_ip4()) {
      addr.str_to(ip);
      break;
    }
  }
  if (ip[0] == '\0') {
    ESP_LOGW(TAG, "No network address for the clip");
    return "";
  }

  const size_t data_bytes = count * sizeof(int16_t);
  auto clip = std::make_shared<Clip>();
  clip->data = static_cast<uint8_t *>(heap_caps_malloc(WAV_HEADER_SIZE + data_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (clip->data == nullptr) {
    ESP_LOGW(TAG, "No memory for a %u-byte clip", (unsigned) data_bytes);
    return "";
  }
  clip->size = WAV_HEADER_SIZE + data_bytes;
  do {
    clip->id = random_uint32();
  } while (clip->id == 0);

  // WAV header: PCM, 16-bit, mono
  uint8_t *h = clip->data;
  memcpy(h, "RIFF", 4);
  put32(h + 4, 36 + data_bytes);
  memcpy(h + 8, "WAVEfmt ", 8);
  put32(h + 16, 16);
  put16(h + 20, 1);
  put16(h + 22, 1);
  put32(h + 24, sample_rate);
  put32(h + 28, sample_rate * 2);
  put16(h + 32, 2);
  put16(h + 34, 16);
  memcpy(h + 36, "data", 4);
  put32(h + 40, data_bytes);

  // The samples, with a 10 ms fade in and out (no click at either end).
  int16_t *out = reinterpret_cast<int16_t *>(h + WAV_HEADER_SIZE);
  memcpy(out, samples, data_bytes);
  const size_t fade = std::min<size_t>(sample_rate / 100, count / 2);
  for (size_t i = 0; i < fade; i++) {
    const int32_t g = (int32_t) ((i << 15) / fade);
    out[i] = (int16_t) ((out[i] * g) >> 15);
    out[count - 1 - i] = (int16_t) ((out[count - 1 - i] * g) >> 15);
  }

  char url[96];
  snprintf(url, sizeof(url), "http://%s:%u%s%08" PRIx32 ".wav", ip, this->port_, CLIP_PATH, clip->id);
  {
    std::lock_guard<std::mutex> guard(this->lock_);
    this->clip_ = clip;
    this->url_ = url;
  }
  this->fetches_.store(0);
  ESP_LOGI(TAG, "New clip: %.1f s at %u Hz, %s", count / (float) sample_rate, (unsigned) sample_rate, url);
  return url;
}

std::string ClipServer::get_clip_url() const {
  std::lock_guard<std::mutex> guard(this->lock_);
  return this->url_;
}

void ClipServer::clear_clip() {
  std::lock_guard<std::mutex> guard(this->lock_);
  this->clip_.reset();
  this->url_.clear();
}

// Runs on the HTTP server's task.
esp_err_t ClipServer::handle_get_(httpd_req_t *req) {
  auto *self = static_cast<ClipServer *>(req->user_ctx);

  // Who asks (for the log)
  char peer[INET_ADDRSTRLEN] = "?";
  struct sockaddr_in addr = {};
  socklen_t addr_len = sizeof(addr);
  if (lwip_getpeername(httpd_req_to_sockfd(req), (struct sockaddr *) &addr, &addr_len) == 0 &&
      addr.sin_family == AF_INET) {
    inet_ntop(AF_INET, &addr.sin_addr, peer, sizeof(peer));
  }

  // /broadcast/<id>.wav (anything after the id is ignored)
  const char *id_text = req->uri + strlen(CLIP_PATH);
  char *end = nullptr;
  const uint32_t want = strtoul(id_text, &end, 16);

  std::shared_ptr<Clip> clip;
  {
    std::lock_guard<std::mutex> guard(self->lock_);
    clip = self->clip_;
  }
  if (clip == nullptr || end == id_text || clip->id != want) {
    ESP_LOGW(TAG, "%s asked for a clip that is gone: %s", peer, req->uri);
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "This broadcast is no longer here");
    return ESP_OK;
  }

  httpd_resp_set_type(req, "audio/wav");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  ESP_LOGI(TAG, "Sending the clip to %s", peer);
  const esp_err_t err = httpd_resp_send(req, reinterpret_cast<const char *>(clip->data), clip->size);
  if (err == ESP_OK) {
    self->fetches_++;
  } else {
    ESP_LOGW(TAG, "Sending the clip to %s stopped: %s", peer, esp_err_to_name(err));
  }
  return err;
}

#endif  // USE_TAB5_BROADCAST_SERVER

#ifdef USE_SPEAKER

void BroadcastSpeaker::setup() {
  // Report played audio upward (the media player uses it for progress).
  this->output_->add_audio_output_callback(
      [this](uint32_t frames, int64_t timestamp) { this->audio_output_callback_.call(frames, timestamp); });
}

void BroadcastSpeaker::loop() { this->follow_output_state_(); }

void BroadcastSpeaker::follow_output_state_() {
  // Only is_stopped() and is_running() are asked of a speaker, so starting
  // and stopping can both show as "starting".
  if (this->output_->is_stopped()) {
    this->state_ = speaker::STATE_STOPPED;
  } else if (this->output_->is_running()) {
    this->state_ = speaker::STATE_RUNNING;
  } else {
    this->state_ = speaker::STATE_STARTING;
  }
}

void BroadcastSpeaker::dump_config() { ESP_LOGCONFIG(TAG, "Tab5 broadcast speaker (keeps its own volume)"); }

void BroadcastSpeaker::start() {
  this->output_->set_audio_stream_info(this->audio_stream_info_);
  this->output_->start();
  if (this->state_ == speaker::STATE_STOPPED)
    this->state_ = speaker::STATE_STARTING;
}

void BroadcastSpeaker::stop() { this->output_->stop(); }

void BroadcastSpeaker::finish() { this->output_->finish(); }

size_t BroadcastSpeaker::play(const uint8_t *data, size_t length, TickType_t ticks_to_wait) {
  // The output starts by itself on its first play(); it has to know the
  // stream before that.
  if (this->output_->is_stopped()) {
    this->output_->set_audio_stream_info(this->audio_stream_info_);
  } else if (this->output_->get_audio_stream_info() != this->audio_stream_info_) {
    // Still running an earlier, different stream: let it stop first.
    this->output_->stop();
    vTaskDelay(ticks_to_wait > 0 ? ticks_to_wait : 1);
    return 0;
  }
  const size_t written = this->output_->play(data, length, ticks_to_wait);
  this->played_ += written;
  return written;
}

#endif  // USE_SPEAKER

}  // namespace esphome::tab5_broadcast

#endif  // USE_ESP_IDF
