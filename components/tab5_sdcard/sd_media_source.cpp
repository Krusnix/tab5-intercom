#include "sd_media_source.h"

#ifdef USE_TAB5_SDCARD_MEDIA_SOURCE

#include "esphome/core/log.h"

#include <esp_heap_caps.h>

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstring>

namespace esphome::tab5_sdcard {

static const char *const TAG = "tab5_sdcard.player";

static constexpr char URI_PREFIX[] = "sdcard://";
// Read in 8 KB pieces: a quarter of a second of 16 kHz mono.
static constexpr size_t READ_CHUNK = 8192;
static constexpr uint32_t WRITE_TIMEOUT_MS = 50;
static constexpr uint32_t PAUSE_POLL_MS = 20;
// Gives up when the player takes no audio for this long.
static constexpr uint32_t STUCK_MS = 5000;
static constexpr uint32_t TASK_STACK = 4096;
static constexpr UBaseType_t TASK_PRIORITY = 2;

static uint16_t le16(const uint8_t *p) { return p[0] | (p[1] << 8); }
static uint32_t le32(const uint8_t *p) {
  return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t) p[3] << 24);
}

void SdFileMediaSource::setup() {
  if (xTaskCreatePinnedToCoreWithCaps(SdFileMediaSource::task_trampoline_, "sd_player", TASK_STACK, this,
                                      TASK_PRIORITY, &this->task_, tskNO_AFFINITY,
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
    this->task_ = nullptr;
    ESP_LOGE(TAG, "Could not start the player's task");
    this->mark_failed();
  }
}

void SdFileMediaSource::loop() { this->take_news_(); }

void SdFileMediaSource::dump_config() { ESP_LOGCONFIG(TAG, "SD card player (sdcard:// WAV files)"); }

bool SdFileMediaSource::can_handle(const std::string &uri) const { return uri.rfind(URI_PREFIX, 0) == 0; }

// Main loop
void SdFileMediaSource::take_news_() {
  if (this->started_.exchange(false) && this->get_state() == media_source::MediaSourceState::IDLE)
    this->set_state_(media_source::MediaSourceState::PLAYING);
  const uint8_t ended = this->ended_.exchange(0);
  if (ended == 2)
    this->set_state_(media_source::MediaSourceState::ERROR);
  if (ended != 0)
    this->set_state_(media_source::MediaSourceState::IDLE);
}

// Main loop
bool SdFileMediaSource::play_uri(const std::string &uri) {
  if (this->task_ == nullptr || this->card_ == nullptr || !this->has_listener() || !this->can_handle(uri))
    return false;
  if (this->get_state() != media_source::MediaSourceState::IDLE || this->active_.load()) {
    ESP_LOGW(TAG, "Busy, can't play %s", uri.c_str());
    return false;
  }
  if (!this->card_->is_mounted()) {
    ESP_LOGW(TAG, "No SD card for %s", uri.c_str());
    return false;
  }
  {
    std::lock_guard<std::mutex> guard(this->lock_);
    this->path_ = this->card_->path(uri.substr(sizeof(URI_PREFIX) - 1));
  }
  this->stop_.store(false);
  this->pause_.store(false);
  this->started_.store(false);
  this->ended_.store(0);
  this->active_.store(true);
  xTaskNotifyGive(this->task_);
  return true;
}

// Main loop
void SdFileMediaSource::handle_command(media_source::MediaSourceCommand command) {
  switch (command) {
    case media_source::MediaSourceCommand::STOP:
      if (this->active_.load()) {
        this->stop_.store(true);
        this->pause_.store(false);
        // The task stops between pieces (at most a write's wait): wait for it
        // briefly, so the next file can start right away.
        for (int i = 0; i < 40 && this->active_.load(); i++)
          vTaskDelay(pdMS_TO_TICKS(5));
      }
      this->take_news_();
      if (!this->active_.load())
        this->set_state_(media_source::MediaSourceState::IDLE);  // also after an error
      break;
    case media_source::MediaSourceCommand::PAUSE:
      if (this->get_state() != media_source::MediaSourceState::PLAYING)
        break;
      this->set_state_(media_source::MediaSourceState::PAUSED);
      this->pause_.store(true);
      break;
    case media_source::MediaSourceCommand::PLAY:
      if (this->get_state() != media_source::MediaSourceState::PAUSED)
        break;
      this->set_state_(media_source::MediaSourceState::PLAYING);
      this->pause_.store(false);
      break;
    default:
      break;
  }
}

void SdFileMediaSource::task_trampoline_(void *arg) { static_cast<SdFileMediaSource *>(arg)->run_task_(); }

void SdFileMediaSource::run_task_() {
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (!this->active_.load())
      continue;
    std::string path;
    {
      std::lock_guard<std::mutex> guard(this->lock_);
      path = this->path_;
    }
    const std::string error = this->play_file_(path);
    if (!error.empty())
      ESP_LOGW(TAG, "Can't play %s: %s", path.c_str(), error.c_str());
    this->ended_.store(error.empty() ? 1 : 2);
    this->active_.store(false);
  }
}

std::string SdFileMediaSource::play_file_(const std::string &path) {
  auto *buf = static_cast<uint8_t *>(heap_caps_aligned_alloc(128, READ_CHUNK, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (buf == nullptr)
    return "out of memory";
  const int fd = open(path.c_str(), O_RDONLY);
  if (fd < 0) {
    const int e = errno;
    heap_caps_free(buf);
    return e == ENOENT ? "no such file" : strerror(e);
  }

  // The WAV header: "RIFF" size "WAVE", then chunks ("fmt ", "data", ...).
  std::string error;
  ssize_t n = read(fd, buf, READ_CHUNK);
  uint32_t data_at = 0, data_len = 0, rate = 0;
  uint16_t format = 0, channels = 0, bits = 0;
  if (n < 0) {
    error = "can't read the card";
  } else if (n < 44 || memcmp(buf, "RIFF", 4) != 0 || memcmp(buf + 8, "WAVE", 4) != 0) {
    error = "not a WAV file";
  } else {
    size_t pos = 12;
    while (pos + 8 <= static_cast<size_t>(n)) {
      const uint32_t size = le32(buf + pos + 4);
      if (memcmp(buf + pos, "fmt ", 4) == 0 && size >= 16 && pos + 24 <= static_cast<size_t>(n)) {
        format = le16(buf + pos + 8);
        channels = le16(buf + pos + 10);
        rate = le32(buf + pos + 12);
        bits = le16(buf + pos + 22);
      } else if (memcmp(buf + pos, "data", 4) == 0) {
        data_at = pos + 8;
        data_len = size;
        break;
      }
      pos += 8 + static_cast<size_t>(size) + (size & 1);
    }
    if (data_at == 0) {
      error = "no audio in the file";
    } else if (format != 1 || bits != 16 || channels < 1 || channels > 2 || rate < 8000 || rate > 48000) {
      error = "not 16-bit PCM";
    }
  }

  if (error.empty()) {
    const audio::AudioStreamInfo info(bits, channels, rate);
    const size_t frame = channels * 2;
    // A recording that was cut short can say 0 (or too much): then up to
    // the end of the file.
    size_t left = (data_len == 0 || data_len == 0xFFFFFFFF) ? SIZE_MAX : data_len - data_len % frame;
    if (lseek(fd, data_at, SEEK_SET) < 0) {
      error = "can't read the card";
    } else {
      this->started_.store(true);
      ESP_LOGD(TAG, "Playing %s (%u Hz, %u channel%s)", path.c_str(), (unsigned) rate, channels,
               channels == 1 ? "" : "s");
    }
    while (error.empty() && left > 0 && !this->stop_.load()) {
      n = read(fd, buf, std::min(READ_CHUNK, left));
      if (n < 0) {
        error = "can't read the card";
        break;
      }
      n -= n % frame;
      if (n == 0)
        break;  // the end of the file
      left -= n;
      size_t done = 0;
      uint32_t idle_ms = 0;
      while (done < static_cast<size_t>(n) && !this->stop_.load()) {
        if (this->pause_.load()) {
          vTaskDelay(pdMS_TO_TICKS(PAUSE_POLL_MS));
          continue;
        }
        const size_t w = this->write_output(buf + done, n - done, WRITE_TIMEOUT_MS, info);
        done += w;
        idle_ms = w > 0 ? 0 : idle_ms + WRITE_TIMEOUT_MS;
        if (idle_ms >= STUCK_MS) {
          error = "the player took no audio";
          break;
        }
      }
    }
  }
  close(fd);
  heap_caps_free(buf);
  return error;
}

}  // namespace esphome::tab5_sdcard

#endif  // USE_TAB5_SDCARD_MEDIA_SOURCE
