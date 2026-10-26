#include "tab5_sdcard.h"

#ifdef USE_ESP_IDF

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <driver/sdmmc_host.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <esp_vfs_fat.h>
#include <sd_pwr_ctrl_by_on_chip_ldo.h>
#include <sdmmc_cmd.h>

#include "diskio_impl.h"
#include "diskio_sdmmc.h"
#include "ff.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace esphome::tab5_sdcard {

static const char *const TAG = "tab5_sdcard";

// The card's own task: low priority (below audio, video and network), stack
// in PSRAM (it never writes to flash).
static constexpr uint32_t TASK_STACK = 6144;
static constexpr UBaseType_t TASK_PRIORITY = 2;
// Read/write test: 4 MB in 64 KB pieces.
static constexpr size_t TEST_CHUNK = 64 * 1024;
static constexpr size_t TEST_BYTES = 4 * 1024 * 1024;
static constexpr const char *const TEST_FILE = "tab5-test.tmp";

std::string SdCard::path(const std::string &relative) const {
  if (relative.empty())
    return this->mount_point_;
  if (relative[0] == '/')
    return this->mount_point_ + relative;
  return this->mount_point_ + "/" + relative;
}

void SdCard::setup() {
  if (xTaskCreatePinnedToCoreWithCaps(SdCard::worker_trampoline_, "tab5_sdcard", TASK_STACK, this, TASK_PRIORITY,
                                      &this->task_, tskNO_AFFINITY, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
    this->task_ = nullptr;
    ESP_LOGE(TAG, "Could not start the card's task");
  }
  if (this->mount_()) {
    this->refresh_space();
  } else {
    this->status_set_warning();
  }
  this->notify_change();
}

void SdCard::loop() {
  if (this->changed_.exchange(false))
    this->change_trigger_.trigger();
}

void SdCard::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Tab5 SD card:\n"
                "  Mount point: %s\n"
                "  Slot: 0 (4-bit), LDO channel: %d, up to %u kHz\n"
                "  Status: %s",
                this->mount_point_.c_str(), this->ldo_channel_, (unsigned) this->max_freq_khz_,
                this->get_status().c_str());
  const std::string info = this->get_card_info();
  if (!info.empty()) {
    ESP_LOGCONFIG(TAG, "  Card: %s", info.c_str());
  }
}

void SdCard::set_status_(const std::string &status) {
  {
    std::lock_guard<std::mutex> guard(this->lock_);
    if (this->status_ == status)
      return;
    this->status_ = status;
  }
  this->notify_change();
}

std::string SdCard::get_status() const {
  std::lock_guard<std::mutex> guard(this->lock_);
  return this->status_;
}

std::string SdCard::get_card_info() const {
  std::lock_guard<std::mutex> guard(this->lock_);
  return this->card_info_;
}

float SdCard::get_size_gb() const {
  std::lock_guard<std::mutex> guard(this->lock_);
  return this->space_known_ ? this->total_bytes_ / 1e9f : NAN;
}

float SdCard::get_free_gb() const {
  std::lock_guard<std::mutex> guard(this->lock_);
  return this->space_known_ ? this->free_bytes_ / 1e9f : NAN;
}

std::string SdCard::get_test_result() const {
  std::lock_guard<std::mutex> guard(this->lock_);
  return this->test_result_;
}

// ---------------- Mounting ----------------

bool SdCard::mount_() {
  if (this->mounted_.load())
    return true;
  esp_err_t err;

  if (!this->slot_ready_) {
    // The slot's pins are powered by the on-chip LDO; the card driver sets
    // it to 3.3 V.
    if (this->ldo_channel_ > 0 && this->pwr_ == nullptr) {
      sd_pwr_ctrl_ldo_config_t ldo = {};
      ldo.ldo_chan_id = this->ldo_channel_;
      err = sd_pwr_ctrl_new_on_chip_ldo(&ldo, &this->pwr_);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "Can't power the card slot (LDO %d): %s", this->ldo_channel_, esp_err_to_name(err));
        this->pwr_ = nullptr;
        this->set_status_("Can't power the card slot");
        return false;
      }
    }
    // The Wi-Fi chip already started the SD host (this only says so), and
    // slot 1 is its own. Neither is ever shut down from here.
    err = sdmmc_host_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
      ESP_LOGE(TAG, "SD host: %s", esp_err_to_name(err));
      this->set_status_("SD host error");
      return false;
    }
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();  // P4: CLK 43, CMD 44, D0-D3 39-42
    slot.width = 4;
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    err = sdmmc_host_init_slot(SDMMC_HOST_SLOT_0, &slot);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Card slot: %s", esp_err_to_name(err));
      this->set_status_("Card slot error");
      return false;
    }
    this->slot_ready_ = true;
  }

  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  host.slot = SDMMC_HOST_SLOT_0;
  host.max_freq_khz = (int) this->max_freq_khz_;
  host.pwr_ctrl_handle = this->pwr_;

  if (this->card_ == nullptr) {
    this->card_ = static_cast<sdmmc_card_t *>(heap_caps_calloc(1, sizeof(sdmmc_card_t), MALLOC_CAP_8BIT));
    if (this->card_ == nullptr) {
      this->set_status_("Out of memory");
      return false;
    }
  }
  const int64_t t0 = esp_timer_get_time();
  err = sdmmc_card_init(&host, this->card_);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "No card found (%s)", esp_err_to_name(err));
    this->set_status_(err == ESP_ERR_TIMEOUT || err == ESP_ERR_NOT_FOUND || err == ESP_ERR_INVALID_RESPONSE
                          ? "No card found"
                          : std::string("Card not answering (") + esp_err_to_name(err) + ")");
    return false;
  }
  if (this->card_->is_sdio) {
    this->set_status_("That is not a memory card");
    return false;
  }

  // FAT on the card, as files under the mount point.
  BYTE pdrv = 0xFF;
  if (ff_diskio_get_drive(&pdrv) != ESP_OK || pdrv == 0xFF) {
    this->set_status_("Too many drives");
    return false;
  }
  ff_diskio_register_sdmmc(pdrv, this->card_);
  ff_sdmmc_set_disk_status_check(pdrv, false);
  char drv[3] = {(char) ('0' + pdrv), ':', 0};
  esp_vfs_fat_conf_t conf = {};
  conf.base_path = this->mount_point_.c_str();
  conf.fat_drive = drv;
  conf.max_files = (size_t) this->max_files_;
  FATFS *fs = nullptr;
  err = esp_vfs_fat_register_cfg(&conf, &fs);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG, "Can't register the card's files: %s", esp_err_to_name(err));
    ff_diskio_unregister(pdrv);
    this->set_status_("Can't use the card's files");
    return false;
  }
  const FRESULT fr = f_mount(fs, drv, 1);
  if (fr != FR_OK) {
    ESP_LOGW(TAG, "Can't read the card's files (FatFs error %d)", (int) fr);
    f_mount(nullptr, drv, 0);
    esp_vfs_fat_unregister_path(this->mount_point_.c_str());
    ff_diskio_unregister(pdrv);
    this->set_status_(fr == FR_NO_FILESYSTEM ? "Not FAT32: format the card as FAT32" : "Can't read the card");
    return false;
  }
  this->fs_ = fs;
  this->pdrv_ = pdrv;

  // What it is: "SDHC 31.9 GB (SD32G), 40 MHz"
  const sdmmc_card_t *c = this->card_;
  const uint64_t bytes = (uint64_t) c->csd.capacity * c->csd.sector_size;
  const char *kind = c->is_mmc ? "MMC" : (c->ocr & SD_OCR_SDHC_CAP) ? (bytes > 34359738368ULL ? "SDXC" : "SDHC") : "SDSC";
  char name[sizeof(c->cid.name) + 1] = {0};
  memcpy(name, c->cid.name, sizeof(c->cid.name));
  for (char &ch : name) {
    if (ch != 0 && (ch < 0x20 || ch > 0x7E))
      ch = '?';
  }
  char info[96];
  snprintf(info, sizeof(info), "%s %.1f GB%s%s%s, %d MHz", kind, bytes / 1e9, name[0] ? " (" : "", name,
           name[0] ? ")" : "", c->real_freq_khz / 1000);
  {
    std::lock_guard<std::mutex> guard(this->lock_);
    this->card_info_ = info;
    this->space_known_ = false;
  }
  this->mounted_.store(true);
  this->set_status_("Ready");
  this->status_clear_warning();
  ESP_LOGI(TAG, "Card ready at %s: %s (%.0f ms)", this->mount_point_.c_str(), info,
           (esp_timer_get_time() - t0) / 1000.0);
  return true;
}

// Takes the files away (the card itself and the slot stay set up).
void SdCard::unmount_() {
  if (!this->mounted_.exchange(false))
    return;
  char drv[3] = {(char) ('0' + this->pdrv_), ':', 0};
  f_mount(nullptr, drv, 0);
  esp_vfs_fat_unregister_path(this->mount_point_.c_str());
  ff_diskio_unregister(this->pdrv_);
  this->fs_ = nullptr;
  this->pdrv_ = 0xFF;
  std::lock_guard<std::mutex> guard(this->lock_);
  this->card_info_.clear();
  this->space_known_ = false;
}

void SdCard::report_card_error(const char *what) {
  if (!this->mounted_.load())
    return;
  ESP_LOGW(TAG, "Card error (%s): is the card still in?", what);
  // Unmounting goes through the card's task, after whatever uses the files.
  this->run_on_card_task([this]() {
    this->unmount_();
    this->set_status_("Card error: take it out and put it back, then Check SD Card");
  });
}

void SdCard::check() {
  if (!this->mounted_.load()) {
    if (this->is_busy()) {
      // Nothing may use the files while mounting: wait for the task.
      this->set_status_("Busy, try again in a moment");
      return;
    }
    if (!this->mount_()) {
      this->status_set_warning();
      this->notify_change();
      return;
    }
  }
  this->refresh_space();
  {
    std::lock_guard<std::mutex> guard(this->lock_);
    this->test_result_ = "Testing...";
  }
  this->notify_change();
  this->run_on_card_task([this]() { this->run_test_(); });
}

void SdCard::refresh_space() {
  this->run_on_card_task([this]() { this->update_space_(); });
}

// ---------------- The card's task ----------------

bool SdCard::run_on_card_task(std::function<void()> &&job) {
  if (this->task_ == nullptr)
    return false;
  {
    std::lock_guard<std::mutex> guard(this->jobs_lock_);
    this->jobs_.push_back(std::move(job));
  }
  this->pending_++;
  xTaskNotifyGive(this->task_);
  return true;
}

void SdCard::worker_trampoline_(void *arg) { static_cast<SdCard *>(arg)->worker_(); }

void SdCard::worker_() {
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    while (true) {
      std::function<void()> job;
      {
        std::lock_guard<std::mutex> guard(this->jobs_lock_);
        if (this->jobs_.empty())
          break;
        job = std::move(this->jobs_.front());
        this->jobs_.pop_front();
      }
      this->busy_.store(true);
      this->pending_--;
      job();
      this->busy_.store(false);
    }
  }
}

void SdCard::update_space_() {
  if (!this->mounted_.load())
    return;
  uint64_t total = 0, free_bytes = 0;
  const esp_err_t err = esp_vfs_fat_info(this->mount_point_.c_str(), &total, &free_bytes);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Can't read the free space: %s", esp_err_to_name(err));
    this->report_card_error("free space");
    return;
  }
  {
    std::lock_guard<std::mutex> guard(this->lock_);
    this->total_bytes_ = total;
    this->free_bytes_ = free_bytes;
    this->space_known_ = true;
  }
  ESP_LOGD(TAG, "%.2f GB free of %.2f GB", free_bytes / 1e9, total / 1e9);
  this->notify_change();
}

// Writes 4 MB, reads it back and checks it, then deletes it.
void SdCard::run_test_() {
  auto done = [this](const std::string &result) {
    {
      std::lock_guard<std::mutex> guard(this->lock_);
      this->test_result_ = result;
    }
    ESP_LOGI(TAG, "Card test: %s", result.c_str());
    this->notify_change();
  };
  if (!this->mounted_.load()) {
    done("Failed: no card");
    return;
  }
  // Cache-line aligned PSRAM: the SD host moves it without copying.
  auto *buf = static_cast<uint8_t *>(heap_caps_aligned_alloc(128, TEST_CHUNK, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (buf == nullptr) {
    done("Failed: out of memory");
    return;
  }
  const std::string file = this->path(TEST_FILE);
  std::string result;
  bool card_gone = false;  // an I/O error: the card is gone or not answering
  int fd = open(file.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0) {
    result = std::string("Failed: can't make a file (") + strerror(errno) + ")";
    card_gone = errno == EIO;
  } else {
    // Write
    const int64_t t0 = esp_timer_get_time();
    bool ok = true;
    for (size_t off = 0; off < TEST_BYTES && ok; off += TEST_CHUNK) {
      for (size_t i = 0; i < TEST_CHUNK; i += 4) {
        const uint32_t v = (uint32_t) (off + i) * 2654435761u;
        memcpy(buf + i, &v, 4);
      }
      ok = write(fd, buf, TEST_CHUNK) == (ssize_t) TEST_CHUNK;
    }
    ok = ok && fsync(fd) == 0;
    const int err_w = errno;
    close(fd);
    const int64_t t1 = esp_timer_get_time();
    if (!ok) {
      result = std::string("Failed: writing stopped (") + strerror(err_w) + ")";
      card_gone = err_w == EIO;
    } else {
      // Read back and check
      fd = open(file.c_str(), O_RDONLY);
      bool read_ok = fd >= 0;
      int err_r = read_ok ? 0 : errno;
      bool same = true;
      for (size_t off = 0; off < TEST_BYTES && read_ok && same; off += TEST_CHUNK) {
        if (read(fd, buf, TEST_CHUNK) != (ssize_t) TEST_CHUNK) {
          read_ok = false;
          err_r = errno;
          break;
        }
        for (size_t i = 0; i < TEST_CHUNK; i += 4) {
          const uint32_t v = (uint32_t) (off + i) * 2654435761u;
          if (memcmp(buf + i, &v, 4) != 0) {
            same = false;
            break;
          }
        }
      }
      if (fd >= 0)
        close(fd);
      const int64_t t2 = esp_timer_get_time();
      if (!read_ok) {
        result = "Failed: couldn't read it back";
        card_gone = err_r == EIO;
      } else if (!same) {
        result = "Failed: what came back was different";
      } else {
        char b[96];
        snprintf(b, sizeof(b), "OK: writes %.1f MB/s, reads %.1f MB/s", TEST_BYTES / (double) (t1 - t0),
                 TEST_BYTES / (double) (t2 - t1));
        result = b;
      }
    }
    unlink(file.c_str());
  }
  heap_caps_free(buf);
  done(result);
  if (card_gone) {
    this->report_card_error("read/write test");
  } else {
    this->update_space_();
  }
}

}  // namespace esphome::tab5_sdcard

#endif  // USE_ESP_IDF
