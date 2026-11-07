#include "voicemail.h"

#ifdef USE_TAB5_SDCARD

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace esphome::tab5_sdcard {

static bool ends_with(const std::string &s, const char *tail) {
  const size_t n = strlen(tail);
  return s.size() >= n && strcasecmp(s.c_str() + s.size() - n, tail) == 0;
}

static bool digits(const std::string &s, size_t at, size_t n) {
  if (s.size() < at + n)
    return false;
  for (size_t i = at; i < at + n; i++) {
    if (!isdigit(static_cast<unsigned char>(s[i])))
      return false;
  }
  return true;
}

Voicemail parse_voicemail(const std::string &file) {
  Voicemail vm;
  vm.file = file;
  std::string base = ends_with(file, ".wav") ? file.substr(0, file.size() - 4) : file;
  if (ends_with(base, "_new")) {
    vm.is_new = true;
    base.resize(base.size() - 4);
  }
  vm.key = base;
  vm.from = base;
  // "YYYYMMDD-HHMMSS_<from>_<n>s"
  if (base.size() >= 19 && digits(base, 0, 8) && base[8] == '-' && digits(base, 9, 6) && base[15] == '_') {
    std::string rest = base.substr(16);
    const size_t us = rest.rfind('_');
    if (us != std::string::npos && rest.size() >= us + 3 && rest.back() == 's' &&
        digits(rest, us + 1, rest.size() - us - 2)) {
      vm.seconds = atoi(rest.c_str() + us + 1);
      rest.resize(us);
    }
    vm.from = rest;
    const int y = atoi(base.substr(0, 4).c_str()), mo = atoi(base.substr(4, 2).c_str());
    const int d = atoi(base.substr(6, 2).c_str()), h = atoi(base.substr(9, 2).c_str());
    const int mi = atoi(base.substr(11, 2).c_str());
    if (y > 2000 && mo >= 1 && mo <= 12 && d >= 1 && d <= 31 && h < 24 && mi < 60) {
      vm.year = y;
      vm.month = mo;
      vm.day = d;
      vm.hour = h;
      vm.minute = mi;
    }
  }
  if (vm.from.empty())
    vm.from = "?";
  return vm;
}

std::string voicemail_file_name(const ESPTime &when, const std::string &from, int seconds) {
  std::string who;
  for (char c : from) {
    if (who.size() >= 24)
      break;
    if (isalnum(static_cast<unsigned char>(c)) || c == '-' || (c == ' ' && !who.empty() && who.back() != ' '))
      who += c;
  }
  while (!who.empty() && who.back() == ' ')
    who.pop_back();
  if (who.empty())
    who = "Someone";
  char stamp[32] = "00000000-000000";
  if (when.is_valid()) {
    snprintf(stamp, sizeof(stamp), "%04d%02d%02d-%02d%02d%02d", when.year, when.month, when.day_of_month, when.hour,
             when.minute, when.second);
  }
  return std::string(stamp) + "_" + who + "_" + std::to_string(seconds < 0 ? 0 : seconds) + "s_new.wav";
}

std::string voicemail_heard_name(const std::string &file) {
  if (ends_with(file, "_new.wav"))
    return file.substr(0, file.size() - 8) + ".wav";
  return file;
}

std::string voicemail_time(const Voicemail &vm, bool clock_24h) {
  if (vm.year == 0)
    return "";
  char b[16];
  if (clock_24h) {
    snprintf(b, sizeof(b), "%d:%02d", vm.hour, vm.minute);
  } else {
    snprintf(b, sizeof(b), "%d:%02d %s", vm.hour % 12 ? vm.hour % 12 : 12, vm.minute, vm.hour < 12 ? "AM" : "PM");
  }
  return b;
}

std::string voicemail_when(const Voicemail &vm, const ESPTime &now, bool clock_24h) {
  if (vm.year == 0)
    return "";
  const std::string t = voicemail_time(vm, clock_24h);
  if (now.is_valid()) {
    if (vm.year == now.year && vm.month == now.month && vm.day == now.day_of_month)
      return "Today " + t;
    const ESPTime y = ESPTime::from_epoch_local(now.timestamp - 86400);
    if (vm.year == y.year && vm.month == y.month && vm.day == y.day_of_month)
      return "Yesterday " + t;
  }
  static const char *const months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                       "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
  char b[24];
  snprintf(b, sizeof(b), "%s %d, ", months[vm.month - 1], vm.day);
  return b + t;
}

std::string voicemail_length(int seconds) {
  if (seconds < 0)
    return "";
  char b[16];
  snprintf(b, sizeof(b), "%d:%02d", seconds / 60, seconds % 60);
  return b;
}

}  // namespace esphome::tab5_sdcard

#endif  // USE_TAB5_SDCARD
