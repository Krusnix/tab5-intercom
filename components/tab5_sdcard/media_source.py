"""Plays WAV files from the Tab5's SD card through a speaker_source media
player: ``sdcard://voicemail/20261026-224301_Mom_23s.wav`` plays
``<mount_point>/voicemail/20261026-224301_Mom_23s.wav``.

Only plain PCM WAV files (16-bit, mono or stereo, 8 to 48 kHz), like the
Tab5's own recordings; no decoder is needed. The file is read in 8 KB pieces
on a task of its own with its stack in PSRAM.
"""

import esphome.codegen as cg
from esphome.components import media_source, psram
import esphome.config_validation as cv
from esphome.const import CONF_ID

from . import SdCard, tab5_sdcard_ns

AUTO_LOAD = ["audio"]
DEPENDENCIES = ["tab5_sdcard"]

CONF_SD_CARD_ID = "sd_card_id"

SdFileMediaSource = tab5_sdcard_ns.class_(
    "SdFileMediaSource", cg.Component, media_source.MediaSource
)

CONFIG_SCHEMA = cv.All(
    media_source.media_source_schema(SdFileMediaSource)
    .extend({cv.GenerateID(CONF_SD_CARD_ID): cv.use_id(SdCard)})
    .extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await media_source.register_media_source(var, config)
    card = await cg.get_variable(config[CONF_SD_CARD_ID])
    cg.add(var.set_card(card))
    cg.add_define("USE_TAB5_SDCARD_MEDIA_SOURCE")
    psram.request_external_task_stack()
