"""A moving wallpaper behind the lock screen of a plain ESPHome mipi_dsi
display (no LVGL), for the M5Stack Tab5 (ESP32-P4).

The wallpaper is a looping clip on the SD card: one file of JPEG pictures
one after the other (an "MJPEG" file, as ``ffmpeg -f mjpeg`` writes it).
The display lambda draws the lock screen on black as usual and calls
``lock_screen_drawn()`` / ``lock_screen_gone()``; this component's task
decodes each picture with the P4's hardware JPEG decoder and lays the lock
screen over it with the PPA (black is see-through), straight into the
panel's frame buffer.
"""

from esphome import automation
import esphome.codegen as cg
from esphome.components import psram
from esphome.components.esp32 import VARIANT_ESP32P4, only_on_variant
from esphome.components.mipi_dsi.display import MipiDsi
import esphome.config_validation as cv
from esphome.const import CONF_DISPLAY_ID, CONF_FILE, CONF_ID
from esphome.core import CORE

CODEOWNERS = ["@Krusnix"]
DEPENDENCIES = ["esp32", "display"]

CONF_FRAMERATE = "framerate"
CONF_MAX_PICTURE_SIZE = "max_picture_size"
CONF_ON_CHANGE = "on_change"

tab5_wallpaper_ns = cg.esphome_ns.namespace("tab5_wallpaper")
Tab5Wallpaper = tab5_wallpaper_ns.class_("Tab5Wallpaper", cg.Component)


def _file_path(value):
    value = cv.string_strict(value)
    if not value.startswith("/") or value.endswith("/"):
        raise cv.Invalid("file must be a full path, like /sdcard/wallpaper/wallpaper.mjpeg")
    return value


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Tab5Wallpaper),
            # The screen (platform: mipi_dsi, 16-bit colour).
            cv.Required(CONF_DISPLAY_ID): cv.use_id(MipiDsi),
            # The clip. Pictures of 720 x 1280 (turned for the Tab5's panel)
            # play quickest; 1280 x 720 works too (turned by the PPA).
            cv.Optional(
                CONF_FILE, default="/sdcard/wallpaper/wallpaper.mjpeg"
            ): _file_path,
            # Pictures a second, unless a JPEG comment in the file says
            # "fps=NN".
            cv.Optional(CONF_FRAMERATE, default=20): cv.int_range(min=1, max=30),
            # Biggest single picture (one PSRAM buffer this size).
            cv.Optional(CONF_MAX_PICTURE_SIZE, default="800kB"): cv.All(
                cv.validate_bytes, cv.int_range(min=64 * 1024, max=4 * 1024 * 1024)
            ),
            # The state words (get_status()) changed.
            cv.Optional(CONF_ON_CHANGE): automation.validate_automation(single=True),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
    only_on_variant(supported=[VARIANT_ESP32P4]),
)


async def to_code(config):
    if CORE.using_arduino:
        raise cv.Invalid("tab5_wallpaper requires the esp-idf framework")
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    disp = await cg.get_variable(config[CONF_DISPLAY_ID])
    cg.add(var.set_display(disp))
    cg.add(var.set_file(config[CONF_FILE]))
    cg.add(var.set_framerate(config[CONF_FRAMERATE]))
    cg.add(var.set_max_picture_bytes(config[CONF_MAX_PICTURE_SIZE]))
    if on_change := config.get(CONF_ON_CHANGE):
        await automation.build_automation(var.get_change_trigger(), [], on_change)
    # The task's stack lives in PSRAM.
    psram.request_external_task_stack()
