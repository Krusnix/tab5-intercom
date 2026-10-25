"""Video calls on a plain ESPHome display (no LVGL) with VoIP Stack.

VoIP Stack's own p4_video_renderer needs LVGL, and ESPHome does not allow
LVGL next to a display lambda. This sink decodes each received JPEG picture
with the ESP32-P4 hardware decoder, scales and rotates it with the PPA, and
writes it straight to the panel inside one rectangle of the screen. The
display lambda keeps drawing everything else.

The optional camera: block adds a gate in front of the camera's JPEG source
(give it to voip_stack as video.source). It keeps the camera off at the start
of every call and lets it be switched on and off during the call.
"""

import esphome.codegen as cg
from esphome import automation
from esphome.components import display, voip_stack
from esphome.components.esp32 import VARIANT_ESP32P4, only_on_variant
import esphome.config_validation as cv
from esphome.const import (
    CONF_DISPLAY_ID,
    CONF_HEIGHT,
    CONF_ID,
    CONF_SOURCE_ID,
    CONF_WIDTH,
    CONF_X,
    CONF_Y,
)
from esphome.core import CORE

CODEOWNERS = ["@Krusnix"]
DEPENDENCIES = ["esp32", "voip_stack", "display"]

CONF_FRAMERATE = "framerate"
CONF_MAX_DECODE_WIDTH = "max_decode_width"
CONF_MAX_DECODE_HEIGHT = "max_decode_height"
CONF_DISPLAY_ROTATION = "display_rotation"
CONF_AREA = "area"
CONF_MAX_CROP = "max_crop"
CONF_SWAP_RED_BLUE = "swap_red_blue"
CONF_ON_FIRST_FRAME = "on_first_frame"
CONF_ON_VIDEO_ENDED = "on_video_ended"
CONF_CAMERA = "camera"
CONF_SELF_VIEW = "self_view"
CONF_SELF_VIEW_MIRROR = "self_view_mirror"
CONF_SELF_VIEW_FRAMERATE = "self_view_framerate"
CONF_NO_VIDEO_TIMEOUT = "no_video_timeout"

tab5_video_ns = cg.esphome_ns.namespace("tab5_video")
Tab5VideoView = tab5_video_ns.class_(
    "Tab5VideoView", cg.Component, voip_stack.EncodedVideoSink
)
Tab5CameraGate = tab5_video_ns.class_(
    "Tab5CameraGate", voip_stack.EncodedVideoSource
)

CAMERA_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Tab5CameraGate),
        # The camera's own VoIP Stack source (esp_jpeg_video_source).
        cv.Required(CONF_SOURCE_ID): cv.use_id(voip_stack.EncodedVideoSource),
        # Self view: this device's own picture, drawn small where the display
        # lambda puts it with set_self_view(). Only while the camera is on.
        cv.Optional(CONF_SELF_VIEW, default=True): cv.boolean,
        # Show it like a mirror (the other side sees it the right way round).
        cv.Optional(CONF_SELF_VIEW_MIRROR, default=True): cv.boolean,
        cv.Optional(CONF_SELF_VIEW_FRAMERATE, default=10): cv.int_range(
            min=1, max=30
        ),
        # Size of the camera's call pictures (as in esp_jpeg_video_source).
        cv.Optional(CONF_WIDTH, default=640): cv.int_range(min=64, max=1280),
        cv.Optional(CONF_HEIGHT, default=360): cv.int_range(min=64, max=800),
    }
)

AREA_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_X, default=0): cv.int_range(min=0, max=2048),
        cv.Optional(CONF_Y, default=0): cv.int_range(min=0, max=2048),
        cv.Required(CONF_WIDTH): cv.int_range(min=16, max=2048),
        cv.Required(CONF_HEIGHT): cv.int_range(min=16, max=2048),
    }
)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Tab5VideoView),
            cv.Required(CONF_DISPLAY_ID): cv.use_id(display.Display),
            # Must match the display's own rotation: setting.
            cv.Optional(CONF_DISPLAY_ROTATION, default=0): cv.one_of(
                0, 90, 180, 270, int=True
            ),
            # Where the picture goes, in screen coordinates (after rotation).
            cv.Required(CONF_AREA): AREA_SCHEMA,
            # Picture size and rate asked of the other side (SDP).
            cv.Optional(CONF_WIDTH, default=640): cv.int_range(min=64, max=1280),
            cv.Optional(CONF_HEIGHT, default=360): cv.int_range(min=64, max=800),
            cv.Optional(CONF_FRAMERATE, default=10): cv.int_range(min=1, max=30),
            # Largest picture accepted from any caller (decode buffer size).
            cv.Optional(CONF_MAX_DECODE_WIDTH, default=1280): cv.int_range(
                min=64, max=1920
            ),
            cv.Optional(CONF_MAX_DECODE_HEIGHT, default=720): cv.int_range(
                min=64, max=1080
            ),
            # Fill the area by cutting off up to this share of the picture;
            # if more would be lost, fit it with black bars instead.
            cv.Optional(CONF_MAX_CROP, default="15%"): cv.percentage,
            cv.Optional(CONF_SWAP_RED_BLUE, default=False): cv.boolean,
            # The other side may stop its camera but keep the call: take the
            # picture off after this long without one (0 = never).
            cv.Optional(
                CONF_NO_VIDEO_TIMEOUT, default="3s"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_CAMERA): CAMERA_SCHEMA,
            cv.Optional(CONF_ON_FIRST_FRAME): automation.validate_automation(
                single=True
            ),
            cv.Optional(CONF_ON_VIDEO_ENDED): automation.validate_automation(
                single=True
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
    only_on_variant(supported=[VARIANT_ESP32P4]),
)


async def to_code(config):
    if CORE.using_arduino:
        raise cv.Invalid("tab5_video requires the esp-idf framework")

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    disp = await cg.get_variable(config[CONF_DISPLAY_ID])
    cg.add(var.set_display(disp))
    cg.add(var.set_display_rotation(config[CONF_DISPLAY_ROTATION]))
    area = config[CONF_AREA]
    cg.add(
        var.set_area(area[CONF_X], area[CONF_Y], area[CONF_WIDTH], area[CONF_HEIGHT])
    )
    cg.add(var.set_receive_size(config[CONF_WIDTH], config[CONF_HEIGHT]))
    cg.add(var.set_framerate(config[CONF_FRAMERATE]))
    cg.add(
        var.set_max_decode(
            config[CONF_MAX_DECODE_WIDTH], config[CONF_MAX_DECODE_HEIGHT]
        )
    )
    cg.add(var.set_max_crop(config[CONF_MAX_CROP]))
    cg.add(var.set_swap_red_blue(config[CONF_SWAP_RED_BLUE]))
    cg.add(var.set_no_video_timeout(config[CONF_NO_VIDEO_TIMEOUT]))

    if camera := config.get(CONF_CAMERA):
        gate = cg.new_Pvariable(camera[CONF_ID])
        source = await cg.get_variable(camera[CONF_SOURCE_ID])
        cg.add(gate.set_source(source))
        if camera[CONF_SELF_VIEW]:
            cg.add(
                var.set_self_view_source_size(camera[CONF_WIDTH], camera[CONF_HEIGHT])
            )
            cg.add(var.set_self_view_mirror(camera[CONF_SELF_VIEW_MIRROR]))
            cg.add(var.set_self_view_framerate(camera[CONF_SELF_VIEW_FRAMERATE]))
            cg.add(gate.set_self_view(var))

    if on_first := config.get(CONF_ON_FIRST_FRAME):
        await automation.build_automation(
            var.get_first_frame_trigger(), [], on_first
        )
    if on_ended := config.get(CONF_ON_VIDEO_ENDED):
        await automation.build_automation(
            var.get_video_ended_trigger(), [], on_ended
        )
