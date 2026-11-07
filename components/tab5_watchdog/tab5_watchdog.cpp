#include "tab5_watchdog.h"

#if defined(USE_ESP_IDF)

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <esp_attr.h>
#include <esp_private/freertos_debug.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <soc/soc.h>
#if CONFIG_IDF_TARGET_ARCH_RISCV
#include <riscv/rvruntime-frames.h>
#endif

#include <algorithm>
#include <atomic>
#include <cinttypes>
#include <cstdio>
#include <cstring>

namespace esphome::tab5_watchdog {

static const char *const TAG = "tab5_watchdog";

namespace {

constexpr uint32_t kMagic = 0x54355745;  // "T5WE"
constexpr int kCores = 2;
constexpr int kNameLen = 16;
// Code addresses kept from the main loop's stack, and how much of it is read.
constexpr int kMaxCandidates = 40;
constexpr int kScanWords = 384;
// After a restart, the report is logged this often for this long (so it shows
// whenever the log is opened), and on every dump_config.
constexpr uint32_t kLogEveryMs = 20000;
constexpr uint8_t kLogTimes = 30;

// Which task each core runs is looked at every kSampleMs, from the other
// core; the last kSamples (2.5 s) are kept.
constexpr uint32_t kSampleMs = 25;
constexpr int kSamples = 100;
constexpr int kTop = 3;
constexpr int kMaxDistinct = 10;
constexpr UBaseType_t kSamplerPriority = 20;

// The main loop is rescued once another task has kept it from running this
// long (the task watchdog restarts the chip at 5 s): that task gets the main
// loop's priority, so the two take turns.
constexpr uint32_t kRescueAfterMs = 2500;
// ... if it had the main loop's core for at least this share of the last 2 s.
constexpr int kWindow = 80;
constexpr int kRescueMinPercent = 40;
// Its priority goes back once the main loop has run for a while and the task
// has calmed down (less than this share of the last 2 s).
constexpr uint32_t kRestoreAfterMs = 3000;
constexpr int kCalmPercent = 30;
constexpr UBaseType_t kLoopPriority = 1;

struct Busy {
  char name[kNameLen];
  uint8_t count;
};

// Written by the task watchdog's interrupt just before the restart; kept
// over a software restart (.noinit).
struct Report {
  uint32_t magic;
  uint32_t uptime_ms;
  char task[kCores][kNameLen];
  uint8_t prio[kCores];
  uint8_t loop_state;  // eTaskState of the main loop task, or 0xFF
  int8_t loop_core;    // core the main loop was running on, -1 = not running
  const LogString *component;  // component the main loop was in
  const LogString *source;     // what in it (script, automation ...)
  uint32_t component_ms;       // how long the main loop had been in it
  uint32_t pc;                 // main loop task: where it was switched out
  uint32_t ra;
  uint32_t candidates[kMaxCandidates];
  uint8_t n_candidates;
  Busy busy[kCores][kTop];  // what each core ran most in the last 2.5 s
  uint8_t n_samples[kCores];
  uint32_t check;
};

Report s_report __attribute__((section(".noinit")));

// What one core ran, newest last.
struct Ring {
  char name[kSamples][kNameLen];
  TaskHandle_t handle[kSamples];
  uint8_t prio[kSamples];
  std::atomic<uint32_t> count{0};
};
Ring s_ring[kCores];

TaskHandle_t s_loop_task = nullptr;
int s_loop_core = 1;
std::atomic<uint32_t> s_loop_runs{0};

// The last rescue, for the main loop to log. seq is odd while it changes.
struct RescueNote {
  char name[kNameLen];
  uint8_t old_prio;
  uint8_t percent;
  uint32_t at_ms;
  uint32_t stalled_ms;
  bool restored;
  uint32_t restored_at_ms;
};
RescueNote s_note{};
std::atomic<uint32_t> s_note_seq{0};
std::atomic<uint32_t> s_rescues{0};

inline uint32_t IRAM_ATTR now_ms() { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }

inline bool IRAM_ATTR is_code_addr(uint32_t a) {
  return (a >= SOC_IROM_LOW && a < SOC_IROM_HIGH) || (a >= SOC_IRAM_LOW && a < SOC_IRAM_HIGH);
}

uint32_t IRAM_ATTR report_check(const Report &r) {
  const auto *p = reinterpret_cast<const uint8_t *>(&r);
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < offsetof(Report, check); i++)
    h = (h ^ p[i]) * 16777619u;
  return h;
}

bool IRAM_ATTR same_name(const char *a, const char *b) {
  for (int i = 0; i < kNameLen; i++) {
    if (a[i] != b[i])
      return false;
    if (a[i] == 0)
      return true;
  }
  return true;
}

// The tasks one core ran most in the newest `window` samples, busiest
// first. Returns how many samples there were.
int IRAM_ATTR tally(const Ring &ring, int window, Busy *top, int n_top, TaskHandle_t *top_handle = nullptr) {
  const uint32_t count = ring.count.load(std::memory_order_acquire);
  const int n = static_cast<int>(std::min<uint32_t>(count, window));
  Busy seen[kMaxDistinct]{};
  TaskHandle_t seen_handle[kMaxDistinct]{};
  int distinct = 0;
  for (int k = 0; k < n; k++) {
    const int i = static_cast<int>((count - 1 - k) % kSamples);
    const char *name = ring.name[i];
    int j = 0;
    while (j < distinct && !same_name(seen[j].name, name))
      j++;
    if (j == distinct) {
      if (distinct == kMaxDistinct)
        continue;
      memcpy(seen[j].name, name, kNameLen);
      seen[j].name[kNameLen - 1] = 0;
      seen_handle[j] = ring.handle[i];
      distinct++;
    }
    if (seen[j].count < 255)
      seen[j].count++;
  }
  for (int t = 0; t < n_top; t++) {
    int best = -1;
    for (int j = 0; j < distinct; j++) {
      if (seen[j].count > 0 && (best < 0 || seen[j].count > seen[best].count))
        best = j;
    }
    if (best < 0) {
      top[t] = Busy{};
      if (top_handle != nullptr)
        top_handle[t] = nullptr;
      continue;
    }
    top[t] = seen[best];
    if (top_handle != nullptr)
      top_handle[t] = seen_handle[best];
    seen[best].count = 0;
  }
  return n;
}

#if CONFIG_IDF_TARGET_ARCH_RISCV
// A return address points just after a call (JAL/JALR with rd = ra, or
// c.jalr). Read at start-up, not in the interrupt.
bool is_return_addr(uint32_t addr) {
  if (!is_code_addr(addr) || addr < 4)
    return false;
  uint32_t inst;
  memcpy(&inst, reinterpret_cast<const void *>(addr - 4), sizeof(inst));
  const uint32_t opcode = inst & 0x7f;
  const uint32_t rd = inst & 0xf80;
  if ((opcode == 0x6f || opcode == 0x67) && rd == 0x80)
    return true;
  uint16_t c_inst;
  memcpy(&c_inst, reinterpret_cast<const void *>(addr - 2), sizeof(c_inst));
  return (c_inst & 0xf07f) == 0x9002 && (c_inst & 0x0f80) != 0;
}
#else
bool is_return_addr(uint32_t addr) { return is_code_addr(addr); }
#endif

const char *state_words(uint8_t state) {
  switch (state) {
    case eRunning:
      return "running";
    case eReady:
      return "ready but not given the processor (another task kept it busy)";
    case eBlocked:
      return "waiting for something (blocked)";
    case eSuspended:
      return "suspended, or waiting with no time limit";
    case eDeleted:
      return "deleted";
    default:
      return "unknown";
  }
}

void write_note(const RescueNote &note) {
  s_note_seq.fetch_add(1, std::memory_order_acq_rel);
  s_note = note;
  s_note_seq.fetch_add(1, std::memory_order_acq_rel);
}

bool read_note(RescueNote &note) {
  for (int tries = 0; tries < 4; tries++) {
    const uint32_t before = s_note_seq.load(std::memory_order_acquire);
    if (before & 1)
      continue;
    note = s_note;
    if (s_note_seq.load(std::memory_order_acquire) == before)
      return true;
  }
  return false;
}

// Watches the main loop from the other core, and gives it a turn when
// another task has kept it from running for too long.
class Guard {
 public:
  void step(const Ring &ring) {
    const uint32_t now = now_ms();
    const uint32_t runs = s_loop_runs.load(std::memory_order_relaxed);
    if (runs != this->last_runs_) {
      this->last_runs_ = runs;
      this->last_change_ = now;
    }
    // Only once the main loop has started (not during start-up).
    if (runs == 0 || s_loop_task == nullptr)
      return;
    const uint32_t stalled = now - this->last_change_;
    if (!this->active_) {
      if (stalled >= kRescueAfterMs)
        this->rescue_(ring, now, stalled);
    } else if (stalled < 200 && now - this->note_.at_ms >= kRestoreAfterMs) {
      this->restore_(ring, now);
    }
  }

 protected:
  void rescue_(const Ring &ring, uint32_t now, uint32_t stalled) {
    // Only when it is crowded out, not when it waits for something.
    if (eTaskGetState(s_loop_task) != eReady)
      return;
    Busy top[kTop];
    TaskHandle_t handle[kTop];
    const int n = tally(ring, kWindow, top, kTop, handle);
    if (n < kWindow / 2)
      return;
    for (int t = 0; t < kTop; t++) {
      if (handle[t] == nullptr || handle[t] == s_loop_task || top[t].count * 100 < kRescueMinPercent * n)
        continue;
      if (strncmp(top[t].name, "IDLE", 4) == 0 || strncmp(top[t].name, "t5wd", 4) == 0)
        continue;
      // Never touch a handle that may be gone: look the task up again.
      if (xTaskGetHandle(top[t].name) != handle[t])
        continue;
      const UBaseType_t old = uxTaskPriorityGet(handle[t]);
      // Not system tasks (timer, inter-processor calls), not its equals.
      if (old <= kLoopPriority || old >= 22)
        continue;
      vTaskPrioritySet(handle[t], kLoopPriority);
      this->active_ = true;
      this->note_ = RescueNote{};
      memcpy(this->note_.name, top[t].name, kNameLen);
      this->note_.old_prio = static_cast<uint8_t>(old);
      this->note_.percent = static_cast<uint8_t>(top[t].count * 100 / n);
      this->note_.at_ms = now;
      this->note_.stalled_ms = stalled;
      write_note(this->note_);
      s_rescues.fetch_add(1, std::memory_order_acq_rel);
      return;
    }
  }

  void restore_(const Ring &ring, uint32_t now) {
    Busy top[kTop];
    const int n = tally(ring, kWindow, top, kTop);
    for (int t = 0; t < kTop; t++) {
      if (same_name(top[t].name, this->note_.name) && top[t].count * 100 >= kCalmPercent * std::max(n, 1))
        return;
    }
    // (A task that has ended is not found, or is a new one with its own
    // priority.)
    TaskHandle_t h = xTaskGetHandle(this->note_.name);
    if (h != nullptr && uxTaskPriorityGet(h) == kLoopPriority)
      vTaskPrioritySet(h, this->note_.old_prio);
    this->active_ = false;
    this->note_.restored = true;
    this->note_.restored_at_ms = now;
    write_note(this->note_);
  }

  uint32_t last_runs_{0};
  uint32_t last_change_{0};
  bool active_{false};
  RescueNote note_{};
};

void sampler_task(void *arg) {
  const int watch = static_cast<int>(reinterpret_cast<intptr_t>(arg));
  Ring &ring = s_ring[watch];
  Guard guard;
  // The two watchers take turns, so neither sees the other one running.
  if (watch == 0)
    vTaskDelay(std::max<TickType_t>(1, pdMS_TO_TICKS(kSampleMs / 2)));
  TickType_t wake = xTaskGetTickCount();
  const TickType_t period = std::max<TickType_t>(1, pdMS_TO_TICKS(kSampleMs));
  for (;;) {
    vTaskDelayUntil(&wake, period);
    TaskHandle_t h = xTaskGetCurrentTaskHandleForCore(watch);
    const uint32_t count = ring.count.load(std::memory_order_relaxed);
    const int i = static_cast<int>(count % kSamples);
    ring.handle[i] = h;
    const char *name = h != nullptr ? pcTaskGetName(h) : nullptr;
    if (name != nullptr) {
      strncpy(ring.name[i], name, kNameLen - 1);
      ring.name[i][kNameLen - 1] = 0;
      ring.prio[i] = static_cast<uint8_t>(uxTaskPriorityGet(h));
    } else {
      ring.name[i][0] = 0;
      ring.prio[i] = 0;
    }
    ring.count.store(count + 1, std::memory_order_release);
    if (watch == s_loop_core)
      guard.step(ring);
  }
}

}  // namespace

}  // namespace esphome::tab5_watchdog

// Called by ESP-IDF's task watchdog interrupt after it printed the tasks
// that missed it, right before it restarts the chip.
extern "C" void IRAM_ATTR esp_task_wdt_isr_user_handler(void) {
  using namespace esphome::tab5_watchdog;
  Report &r = s_report;
  memset(&r, 0, sizeof(r));
  r.uptime_ms = now_ms();
  r.loop_core = -1;
  r.loop_state = 0xFF;
  for (int core = 0; core < kCores && core < CONFIG_FREERTOS_NUMBER_OF_CORES; core++) {
    TaskHandle_t h = xTaskGetCurrentTaskHandleForCore(core);
    if (h != nullptr) {
      const char *name = pcTaskGetName(h);
      if (name != nullptr)
        strncpy(r.task[core], name, sizeof(r.task[core]) - 1);
      r.prio[core] = static_cast<uint8_t>(uxTaskPriorityGetFromISR(h));
      if (h == s_loop_task)
        r.loop_core = static_cast<int8_t>(core);
    }
    r.n_samples[core] = static_cast<uint8_t>(tally(s_ring[core], kSamples, r.busy[core], kTop));
  }
  if (s_loop_task != nullptr) {
    r.loop_state = static_cast<uint8_t>(eTaskGetState(s_loop_task));
    esphome::Component *c = esphome::App.get_current_component();
    if (c != nullptr) {
      r.component = c->get_component_log_str();
      r.component_ms = now_ms() - esphome::App.get_loop_component_start_time();
    }
    r.source = esphome::App.get_current_source();
#if CONFIG_IDF_TARGET_ARCH_RISCV
    // Not running: its registers were saved on its stack when it was
    // switched out. (Running: the crash report shows where.)
    TaskSnapshot_t snap{};
    if (r.loop_core < 0 && vTaskGetSnapshot(s_loop_task, &snap) == pdTRUE && snap.pxTopOfStack != nullptr) {
      const auto *frame = reinterpret_cast<const RvExcFrame *>(snap.pxTopOfStack);
      r.pc = frame->mepc;
      r.ra = frame->ra;
      const auto *sp = reinterpret_cast<const uint32_t *>(frame->sp);
      const auto *end = reinterpret_cast<const uint32_t *>(snap.pxEndOfStack);
      const auto *top = reinterpret_cast<const uint32_t *>(snap.pxTopOfStack);
      if (sp >= top && sp < end) {
        for (int i = 0; i < kScanWords && sp + i < end && r.n_candidates < kMaxCandidates; i++) {
          const uint32_t w = sp[i];
          if (is_code_addr(w))
            r.candidates[r.n_candidates++] = w;
        }
      }
    }
#endif
  }
  r.magic = kMagic;
  r.check = report_check(r);
}

namespace esphome::tab5_watchdog {

void Tab5Watchdog::setup() {
  // setup() runs on the main loop task.
  s_loop_task = xTaskGetCurrentTaskHandle();
  const BaseType_t core = xTaskGetCoreID(s_loop_task);
  s_loop_core = (core == 0 || core == 1) ? static_cast<int>(core) : 1;

  // One watcher per core, each running on the other core.
  for (int watch = 0; watch < kCores && watch < CONFIG_FREERTOS_NUMBER_OF_CORES; watch++) {
    char name[configMAX_TASK_NAME_LEN];
    snprintf(name, sizeof(name), "t5wd_cpu%d", watch);
    const BaseType_t on = CONFIG_FREERTOS_NUMBER_OF_CORES > 1 ? 1 - watch : 0;
    if (xTaskCreatePinnedToCore(sampler_task, name, 2560, reinterpret_cast<void *>(static_cast<intptr_t>(watch)),
                                kSamplerPriority, nullptr, on) != pdPASS) {
      ESP_LOGW(TAG, "Couldn't start the CPU %d watcher", watch);
    }
  }

  Report r = s_report;
  s_report.magic = 0;
  if (r.magic != kMagic || r.check != report_check(r))
    return;
  r.n_candidates = std::min<uint8_t>(r.n_candidates, kMaxCandidates);
  for (auto &t : r.task)
    t[sizeof(t) - 1] = 0;
  for (auto &core : r.busy) {
    for (auto &b : core)
      b.name[kNameLen - 1] = 0;
  }

  // The main loop's code: where it was, the caller, then the return
  // addresses on its stack.
  if (r.pc != 0 && is_code_addr(r.pc))
    this->trace_[this->trace_len_++] = r.pc;
  if (r.ra != 0 && is_code_addr(r.ra) && r.ra != r.pc)
    this->trace_[this->trace_len_++] = r.ra;
  for (int i = 0; i < r.n_candidates && this->trace_len_ < sizeof(this->trace_) / sizeof(this->trace_[0]); i++) {
    const uint32_t a = r.candidates[i];
    if (a == r.ra || !is_return_addr(a))
      continue;
    this->trace_[this->trace_len_++] = a;
  }

  const uint32_t s = r.uptime_ms / 1000;
  std::string loop;
  if (r.loop_core >= 0) {
    char c[24];
    snprintf(c, sizeof(c), "busy on CPU %d", r.loop_core);
    loop = c;
  } else {
    loop = state_words(r.loop_state);
  }
  char b[320];
  snprintf(b, sizeof(b),
           "Froze %" PRIu32 " min %" PRIu32 " s after start-up. Main loop: %s, in %s%s%s (for %" PRIu32
           " ms). At the end CPU 0 ran %s (priority %u), CPU 1 ran %s (priority %u).",
           s / 60, s % 60, loop.c_str(), r.component != nullptr ? LOG_STR_ARG(r.component) : "?",
           r.source != nullptr ? " / " : "", r.source != nullptr ? LOG_STR_ARG(r.source) : "", r.component_ms,
           r.task[0][0] ? r.task[0] : "?", r.prio[0], r.task[1][0] ? r.task[1] : "?", r.prio[1]);
  this->freeze_ = b;

  // What each core ran most in its last 2.5 s.
  std::string busy;
  for (int core = 0; core < kCores; core++) {
    const int n = r.n_samples[core];
    if (n == 0)
      continue;
    char part[120];
    int len = snprintf(part, sizeof(part), "%sCPU %d:", busy.empty() ? "" : " ", core);
    for (const auto &t : r.busy[core]) {
      if (t.count == 0 || len >= static_cast<int>(sizeof(part)))
        continue;
      len += snprintf(part + len, sizeof(part) - len, " %s %d%%", t.name[0] ? t.name : "?", t.count * 100 / n);
    }
    busy += part;
  }
  this->freeze_busy_ = busy;
}

void Tab5Watchdog::loop() {
  s_loop_runs.fetch_add(1, std::memory_order_relaxed);

  const uint32_t rescues = s_rescues.load(std::memory_order_acquire);
  RescueNote note;
  if (rescues != this->rescues_seen_ && read_note(note)) {
    this->rescues_seen_ = rescues;
    this->restore_logged_ = false;
    note.name[kNameLen - 1] = 0;
    const uint32_t s = note.at_ms / 1000;
    char b[200];
    snprintf(b, sizeof(b),
             "Main loop rescued %" PRIu32 " min %" PRIu32 " s after start-up from %s (priority %u), which had CPU %d "
             "%u%% of the time; the main loop hadn't run for %.1f s",
             s / 60, s % 60, note.name, note.old_prio, s_loop_core, note.percent, note.stalled_ms / 1000.0f);
    this->rescue_ = b;
    ESP_LOGW(TAG, "%s. %s now takes turns with the main loop.", b, note.name);
    this->log_rescues_();
  } else if (!this->restore_logged_ && read_note(note) && note.restored) {
    this->restore_logged_ = true;
    note.name[kNameLen - 1] = 0;
    ESP_LOGI(TAG, "%s calmed down: priority %u again", note.name, note.old_prio);
  }

  if (this->freeze_.empty() || this->times_logged_ >= kLogTimes)
    return;
  const uint32_t now = millis();
  if (this->times_logged_ != 0 && now - this->last_log_ms_ < kLogEveryMs)
    return;
  this->last_log_ms_ = now;
  this->times_logged_++;
  this->log_freeze_();
}

std::string Tab5Watchdog::get_report() const {
  std::string out;
  if (this->freeze_.empty()) {
    out = "No freeze since power-up.";
  } else {
    out = this->freeze_;
    if (!this->freeze_busy_.empty())
      out += " Last 2.5 s: " + this->freeze_busy_ + ".";
  }
  if (!this->rescue_.empty()) {
    char n[48];
    snprintf(n, sizeof(n), " Rescued %" PRIu32 "x; last: ", this->rescues_seen_);
    out += n + this->rescue_ + ".";
  }
  if (out.size() > 250)
    out.resize(250);
  return out;
}

void Tab5Watchdog::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Tab5 watchdog:\n"
                "  Main loop: CPU %d, rescued after %" PRIu32 " ms\n"
                "  Last freeze: %s",
                s_loop_core, kRescueAfterMs, this->freeze_.empty() ? "none since power-up" : "see below");
  if (!this->freeze_.empty())
    this->log_freeze_();
  if (!this->rescue_.empty()) {
    ESP_LOGCONFIG(TAG, "  Rescued %" PRIu32 " times; last: %s", this->rescues_seen_, this->rescue_.c_str());
  }
}

void Tab5Watchdog::log_freeze_() {
  ESP_LOGE(TAG, "The main loop froze before the last restart (task watchdog):");
  ESP_LOGE(TAG, "  %s", this->freeze_.c_str());
  if (!this->freeze_busy_.empty())
    ESP_LOGE(TAG, "  What the cores ran most in the last 2.5 s: %s", this->freeze_busy_.c_str());
  if (this->trace_len_ == 0)
    return;
  ESP_LOGE(TAG, "  Main loop code, innermost first:");
  for (uint8_t i = 0; i < this->trace_len_; i++)
    ESP_LOGE(TAG, "  BT%u: 0x%08" PRIX32, (unsigned) i, this->trace_[i]);
}

void Tab5Watchdog::log_rescues_() {
  // What the cores ran most in the last 2 s, as seen right now.
  for (int core = 0; core < kCores; core++) {
    Busy top[kTop];
    const int n = tally(s_ring[core], kWindow, top, kTop);
    if (n == 0)
      continue;
    char b[120];
    int len = snprintf(b, sizeof(b), "CPU %d in the last 2 s:", core);
    for (const auto &t : top) {
      if (t.count == 0 || len >= static_cast<int>(sizeof(b)))
        continue;
      len += snprintf(b + len, sizeof(b) - len, " %s %d%%", t.name[0] ? t.name : "?", t.count * 100 / n);
    }
    ESP_LOGW(TAG, "  %s", b);
  }
}

}  // namespace esphome::tab5_watchdog

#endif
