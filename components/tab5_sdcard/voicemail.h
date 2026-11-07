#pragma once

#include "esphome/core/defines.h"

#ifdef USE_TAB5_SDCARD

#include "esphome/core/time.h"

#include <string>

namespace esphome::tab5_sdcard {

/// One voicemail file: "20261026-224301_Mom_23s_new.wav" (left at
/// 2026-10-26 22:43:01 by Mom, 23 s long, not played yet). Once played it
/// loses "_new".
struct Voicemail {
  std::string file;  ///< the name on the card
  std::string key;   ///< the name without "_new" (stays the same once played)
  std::string from;  ///< "Mom" (the name without ".wav" for other files)
  int year{0}, month{0}, day{0}, hour{0}, minute{0};  ///< 0 = unknown
  int seconds{-1};   ///< -1 = unknown
  bool is_new{false};
};

/// Reads what a voicemail's file name says.
Voicemail parse_voicemail(const std::string &file);
/// The file name for a new voicemail. `from` keeps letters, digits, spaces
/// and dashes (at most 24); the time is "00000000-000000" if not known.
std::string voicemail_file_name(const ESPTime &when, const std::string &from, int seconds);
/// The file name once played (without "_new").
std::string voicemail_heard_name(const std::string &file);
/// "Today 2:41 PM", "Yesterday 9:05 AM", "Oct 26, 9:05 AM" ("" if unknown).
std::string voicemail_when(const Voicemail &vm, const ESPTime &now, bool clock_24h);
/// "2:41 PM" or "14:41" ("" if unknown).
std::string voicemail_time(const Voicemail &vm, bool clock_24h);
/// "0:23", "1:00" ("" if unknown).
std::string voicemail_length(int seconds);

}  // namespace esphome::tab5_sdcard

#endif  // USE_TAB5_SDCARD
