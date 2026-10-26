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

#include <dirent.h>
#include <fcntl.h>
#include <lwip/netdb.h>
#include <lwip/sockets.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
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
// Saving downloads: written to the card in pieces of up to 32 KB.
static constexpr size_t SAVE_CHUNK = 32 * 1024;
static constexpr int CONNECT_TIMEOUT_MS = 3000;
static constexpr int READ_TIMEOUT_S = 8;
// Most files one folder listing keeps.
static constexpr size_t MAX_LISTED = 200;

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
    this->scan_();
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
  this->run_on_card_task([this]() { this->scan_(); });
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

// ---------------- Folders ----------------

void SdCard::scan_folder(const std::string &folder) {
  {
    std::lock_guard<std::mutex> guard(this->lock_);
    this->scan_folder_ = folder;
  }
  this->run_on_card_task([this]() { this->scan_(); });
}

std::vector<std::string> SdCard::get_files() const {
  std::lock_guard<std::mutex> guard(this->lock_);
  return this->files_;
}

void SdCard::remove_file(const std::string &relative) {
  this->run_on_card_task([this, relative]() {
    if (this->mounted_.load() && unlink(this->path(relative).c_str()) != 0 && errno != ENOENT) {
      ESP_LOGW(TAG, "Can't delete %s: %s", relative.c_str(), strerror(errno));
      if (errno == EIO)
        this->report_card_error("delete");
    }
    this->scan_();
  });
}

void SdCard::rename_file(const std::string &from, const std::string &to) {
  this->run_on_card_task([this, from, to]() {
    if (this->mounted_.load()) {
      const std::string a = this->path(from), b = this->path(to);
      unlink(b.c_str());
      if (rename(a.c_str(), b.c_str()) != 0) {
        ESP_LOGW(TAG, "Can't rename %s: %s", from.c_str(), strerror(errno));
        if (errno == EIO)
          this->report_card_error("rename");
      }
    }
    this->scan_();
  });
}

void SdCard::scan_() {
  std::string folder;
  {
    std::lock_guard<std::mutex> guard(this->lock_);
    folder = this->scan_folder_;
  }
  if (folder.empty())
    return;
  std::vector<std::string> names;
  if (this->mounted_.load()) {
    const std::string dir = this->path(folder);
    mkdir(dir.c_str(), 0775);  // fine if it's there
    DIR *d = opendir(dir.c_str());
    if (d == nullptr) {
      ESP_LOGW(TAG, "Can't list %s: %s", folder.c_str(), strerror(errno));
    } else {
      while (struct dirent *e = readdir(d)) {
        if (e->d_type == DT_DIR || e->d_name[0] == '.')
          continue;
        const size_t len = strlen(e->d_name);
        if (len > 5 && strcmp(e->d_name + len - 5, ".part") == 0)
          continue;  // a save that didn't finish
        names.emplace_back(e->d_name);
      }
      closedir(d);
    }
    std::sort(names.begin(), names.end(), std::greater<>());
    if (names.size() > MAX_LISTED)
      names.resize(MAX_LISTED);
  }
  {
    std::lock_guard<std::mutex> guard(this->lock_);
    this->files_ = std::move(names);
  }
  this->files_version_++;
  this->notify_change();
}

bool SdCard::make_dirs_(const std::string &relative_file) {
  size_t at = 0;
  while ((at = relative_file.find('/', at + 1)) != std::string::npos) {
    const std::string dir = this->path(relative_file.substr(0, at));
    if (mkdir(dir.c_str(), 0775) != 0 && errno != EEXIST) {
      struct stat st {};
      if (stat(dir.c_str(), &st) != 0 || !S_ISDIR(st.st_mode))
        return false;
    }
  }
  return true;
}

// ---------------- Saving a download ----------------

std::string SdCard::get_save_error() const {
  std::lock_guard<std::mutex> guard(this->lock_);
  return this->save_error_;
}

bool SdCard::start_save(const std::string &url, const std::string &relative, uint32_t max_bytes) {
  if (!this->mounted_.load() || this->task_ == nullptr || relative.empty())
    return false;
  uint8_t state = this->save_state_.load();
  if (state == SAVE_RUNNING || !this->save_state_.compare_exchange_strong(state, SAVE_RUNNING))
    return false;
  this->save_bytes_.store(0);
  {
    std::lock_guard<std::mutex> guard(this->lock_);
    this->save_error_.clear();
  }
  if (!this->run_on_card_task([this, url, relative, max_bytes]() { this->save_url_(url, relative, max_bytes); })) {
    this->save_state_.store(SAVE_FAILED);
    return false;
  }
  return true;
}

namespace {

// Reads until "\r\n\r\n" or the buffer is full. Returns the bytes read and
// where the body starts (0 = the end of the headers wasn't found).
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

// The value of header `name` (lower case, with the colon), or "".
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

// Connects to host:port within CONNECT_TIMEOUT_MS. -1 on failure.
int connect_to(const std::string &host, int port) {
  struct addrinfo hints {};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  struct addrinfo *res = nullptr;
  char port_str[8];
  snprintf(port_str, sizeof(port_str), "%d", port);
  if (lwip_getaddrinfo(host.c_str(), port_str, &hints, &res) != 0 || res == nullptr)
    return -1;
  int sock = lwip_socket(res->ai_family, res->ai_socktype, res->ai_protocol);
  if (sock >= 0) {
    const int flags = lwip_fcntl(sock, F_GETFL, 0);
    lwip_fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    int rc = lwip_connect(sock, res->ai_addr, res->ai_addrlen);
    if (rc != 0 && errno == EINPROGRESS) {
      fd_set wfds;
      FD_ZERO(&wfds);
      FD_SET(sock, &wfds);
      struct timeval tv {};
      tv.tv_sec = CONNECT_TIMEOUT_MS / 1000;
      tv.tv_usec = (CONNECT_TIMEOUT_MS % 1000) * 1000;
      rc = lwip_select(sock + 1, nullptr, &wfds, nullptr, &tv) == 1 ? 0 : -1;
      if (rc == 0) {
        int so_error = 0;
        socklen_t len = sizeof(so_error);
        lwip_getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_error, &len);
        rc = so_error == 0 ? 0 : -1;
      }
    }
    lwip_fcntl(sock, F_SETFL, flags);
    if (rc != 0) {
      lwip_close(sock);
      sock = -1;
    }
  }
  lwip_freeaddrinfo(res);
  return sock;
}

}  // namespace

void SdCard::save_url_(const std::string &url, const std::string &relative, uint32_t max_bytes) {
  const int64_t t0 = esp_timer_get_time();
  std::string error;
  bool card_gone = false;
  int sock = -1, fd = -1;
  uint8_t *buf = nullptr;
  const std::string final_path = this->path(relative);
  const std::string part_path = final_path + ".part";
  size_t got = 0;

  // http://host[:port]/path (no https: this stays on the home network)
  std::string host, req_path = "/";
  int port = 80;
  if (url.compare(0, 7, "http://") == 0) {
    const size_t slash = url.find('/', 7);
    host = url.substr(7, slash == std::string::npos ? std::string::npos : slash - 7);
    if (slash != std::string::npos)
      req_path = url.substr(slash);
    const size_t colon = host.rfind(':');
    if (colon != std::string::npos) {
      port = atoi(host.c_str() + colon + 1);
      host.resize(colon);
    }
  }

  if (host.empty() || port <= 0 || port > 65535) {
    error = "bad address";
  } else if (!this->mounted_.load()) {
    error = "no card";
  } else if ((sock = connect_to(host, port)) < 0) {
    error = "can't reach " + host;
  } else {
    struct timeval tv {};
    tv.tv_sec = READ_TIMEOUT_S;
    lwip_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    lwip_setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    // HTTP/1.0: the answer comes in one piece and the connection closes.
    std::string request = "GET " + req_path + " HTTP/1.0\r\nHost: " + host;
    if (port != 80)
      request += ":" + std::to_string(port);
    request += "\r\nConnection: close\r\nUser-Agent: tab5_sdcard\r\n\r\n";
    if (lwip_send(sock, request.data(), request.size(), 0) != static_cast<int>(request.size()))
      error = "can't ask " + host;
  }

  // Cache-line aligned PSRAM: the SD host moves it without copying.
  if (error.empty()) {
    buf = static_cast<uint8_t *>(heap_caps_aligned_alloc(128, SAVE_CHUNK, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (buf == nullptr)
      error = "out of memory";
  }

  long expected = -1;
  size_t fill = 0;
  if (error.empty()) {
    char head[1024];
    int body_at = 0;
    const int head_len = read_headers(sock, head, sizeof(head), &body_at);
    int status = 0;
    if (body_at > 12 && memcmp(head, "HTTP/", 5) == 0) {
      const char *space = static_cast<const char *>(memchr(head, ' ', body_at));
      if (space != nullptr)
        status = atoi(space + 1);
    }
    if (body_at == 0 || status <= 0) {
      error = head_len <= 0 ? "no answer" : "bad answer";
    } else if (status != 200) {
      error = "the other side said " + std::to_string(status);
    } else if (!header_value(head, body_at, "transfer-encoding:").empty()) {
      error = "bad answer (chunked)";
    } else {
      const std::string cl = header_value(head, body_at, "content-length:");
      expected = cl.empty() ? -1 : atol(cl.c_str());
      if (expected > static_cast<long>(max_bytes)) {
        error = "too big";
      } else {
        fill = static_cast<size_t>(head_len - body_at);
        memcpy(buf, head + body_at, fill);
      }
    }
  }

  if (error.empty()) {
    if (!this->make_dirs_(relative)) {
      error = "can't make the folder";
      card_gone = errno == EIO;
    } else if ((fd = open(part_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644)) < 0) {
      error = std::string("can't make the file (") + strerror(errno) + ")";
      card_gone = errno == EIO;
    }
  }

  // Receive, and write each full 32 KB to the card.
  bool closed = false;
  while (error.empty()) {
    const bool want_more = !closed && (expected < 0 || got + fill < static_cast<size_t>(expected));
    if (want_more && fill < SAVE_CHUNK) {
      const int r = lwip_recv(sock, buf + fill, SAVE_CHUNK - fill, 0);
      if (r > 0) {
        fill += r;
        if (got + fill > max_bytes)
          error = "too big";
        continue;
      }
      if (r < 0 && expected < 0) {
        error = "the download stopped";
        break;
      }
      closed = true;  // the other side is done (or quiet for too long)
    }
    if (fill > 0) {
      if (write(fd, buf, fill) != static_cast<ssize_t>(fill)) {
        error = std::string("writing stopped (") + strerror(errno) + ")";
        card_gone = errno == EIO;
        break;
      }
      got += fill;
      this->save_bytes_.store(got);
      fill = 0;
      if (this->gentle_.load())
        vTaskDelay(pdMS_TO_TICKS(40));  // leave the SD host to the Wi-Fi chip for a moment
    }
    if (!(!closed && (expected < 0 || got < static_cast<size_t>(expected))))
      break;
  }
  if (sock >= 0)
    lwip_close(sock);
  if (buf != nullptr)
    heap_caps_free(buf);
  if (fd >= 0) {
    if (error.empty() && fsync(fd) != 0) {
      error = std::string("writing stopped (") + strerror(errno) + ")";
      card_gone = errno == EIO;
    }
    close(fd);
  }
  if (error.empty()) {
    if (expected >= 0 && got < static_cast<size_t>(expected)) {
      error = "the download stopped";
    } else if (got == 0) {
      error = "nothing came";
    }
  }
  if (error.empty()) {
    unlink(final_path.c_str());
    if (rename(part_path.c_str(), final_path.c_str()) != 0) {
      error = std::string("can't rename it (") + strerror(errno) + ")";
      card_gone = errno == EIO;
    }
  }
  if (!error.empty() && fd >= 0)
    unlink(part_path.c_str());

  if (error.empty()) {
    ESP_LOGI(TAG, "Saved %s (%u bytes, %.1f s)", relative.c_str(), (unsigned) got,
             (esp_timer_get_time() - t0) / 1e6);
    // The folder listing and free space are up to date before "saved".
    this->update_space_();
    this->scan_();
    this->save_state_.store(SAVE_DONE);
    this->notify_change();
    return;
  }
  ESP_LOGW(TAG, "Not saved %s: %s", relative.c_str(), error.c_str());
  {
    std::lock_guard<std::mutex> guard(this->lock_);
    this->save_error_ = error;
  }
  this->save_state_.store(SAVE_FAILED);
  this->notify_change();
  if (card_gone)
    this->report_card_error("save");
}

}  // namespace esphome::tab5_sdcard

#endif  // USE_ESP_IDF
