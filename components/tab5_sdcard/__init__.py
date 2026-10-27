"""microSD card for the M5Stack Tab5 (ESP32-P4), next to its Wi-Fi chip.

The Tab5's card slot is on the P4's SD host, slot 0 (4-bit: CLK 43, CMD 44,
D0-D3 39-42), and its pins get their power from the P4's on-chip LDO 4. The
ESP32-C6 Wi-Fi chip (esp32_hosted) sits on slot 1 of the same SD host, so
this component:

- powers the slot with ESP-IDF's on-chip LDO driver, starts only slot 0
  (the host is already running for the Wi-Fi chip) and never shuts the
  host or the slot down again, so Wi-Fi is never cut off;
- mounts the card once at start-up, before Wi-Fi connects (setting up a
  slot changes the shared host's clock), and again only when asked
  (``check()``), never by itself in the middle of a call;
- does its slow work (free space, the read/write test, saving downloads)
  on a task of its own with its stack in PSRAM.

The card has to be FAT32 (cards up to 32 GB come that way). Files are then
plain C/POSIX files under ``mount_point`` (fopen, open, stat, unlink ...).

ESPHome 2026.10 leaves ESP-IDF's FAT and SD card parts out of builds by
default; this component puts them back.
"""

from esphome import automation
import esphome.codegen as cg
from esphome.components import psram
from esphome.components.esp32 import (
    VARIANT_ESP32P4,
    add_idf_sdkconfig_option,
    include_builtin_idf_component,
    only_on_variant,
    require_fatfs,
    require_vfs_dir,
)
import esphome.config_validation as cv
from esphome.const import CONF_FREQUENCY, CONF_ID
from esphome.core import CORE

CODEOWNERS = ["@Krusnix"]
DEPENDENCIES = ["esp32"]

CONF_MOUNT_POINT = "mount_point"
CONF_LDO_CHANNEL = "ldo_channel"
CONF_MAX_FILES = "max_files"
CONF_ON_CHANGE = "on_change"

tab5_sdcard_ns = cg.esphome_ns.namespace("tab5_sdcard")
SdCard = tab5_sdcard_ns.class_("SdCard", cg.Component)


def _mount_point(value):
    value = cv.string_strict(value)
    if not value.startswith("/") or value.endswith("/") or len(value) < 2:
        raise cv.Invalid("mount_point must look like /sdcard")
    return value


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SdCard),
            cv.Optional(CONF_MOUNT_POINT, default="/sdcard"): _mount_point,
            # On-chip LDO channel that powers the slot's pins (Tab5: 4);
            # 0 = the board powers them itself.
            cv.Optional(CONF_LDO_CHANNEL, default=4): cv.int_range(min=0, max=4),
            # 40 MHz (high speed) like M5Stack's own Tab5 code; the same
            # host clock as the Wi-Fi chip's 40 MHz. Cards that can't do it
            # run at 20 MHz.
            cv.Optional(CONF_FREQUENCY, default="40MHz"): cv.All(
                cv.frequency, cv.one_of(20e6, 40e6)
            ),
            # Files open at the same time.
            cv.Optional(CONF_MAX_FILES, default=4): cv.int_range(min=1, max=10),
            # Something changed: mounted or not, free space, test result.
            cv.Optional(CONF_ON_CHANGE): automation.validate_automation(single=True),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
    only_on_variant(supported=[VARIANT_ESP32P4]),
)


async def to_code(config):
    if CORE.using_arduino:
        raise cv.Invalid("tab5_sdcard requires the esp-idf framework")
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_mount_point(config[CONF_MOUNT_POINT]))
    cg.add(var.set_ldo_channel(config[CONF_LDO_CHANNEL]))
    cg.add(var.set_max_freq_khz(int(config[CONF_FREQUENCY] // 1000)))
    cg.add(var.set_max_files(config[CONF_MAX_FILES]))
    if on_change := config.get(CONF_ON_CHANGE):
        await automation.build_automation(var.get_change_trigger(), [], on_change)

    cg.add_define("USE_TAB5_SDCARD")
    # ESP-IDF's SD card and FAT parts (ESPHome leaves them out by default).
    for name in ("fatfs", "sdmmc", "esp_driver_sdmmc", "wear_levelling"):
        include_builtin_idf_component(name)
    require_fatfs()
    # Folders, deleting and renaming files (ESPHome turns these off by
    # default: without them unlink, rename, stat, mkdir and opendir fail).
    require_vfs_dir()
    # Long file names, FAT's buffers in PSRAM (the internal memory is
    # scarce on the Tab5), sector buffers only as big as the card's sectors.
    add_idf_sdkconfig_option("CONFIG_FATFS_LFN_HEAP", True)
    add_idf_sdkconfig_option("CONFIG_FATFS_MAX_LFN", 255)
    add_idf_sdkconfig_option("CONFIG_FATFS_ALLOC_PREFER_EXTRAM", True)
    add_idf_sdkconfig_option("CONFIG_FATFS_USE_DYN_BUFFERS", True)
    # The card's own work runs on a task with its stack in PSRAM.
    psram.request_external_task_stack()
