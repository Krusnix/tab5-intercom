"""Pass-through speaker that keeps volume and mute to itself.

A media player sets the volume and mute of its speaker, and ESPHome's mixer
and resampler speakers pass those down to the hardware speaker, where they
would change the volume of everything else too (calls, the ring). Put this
speaker between the media player and the rest of its chain: the audio goes
through unchanged, while volume and mute stop here. Use the media player's
own volume (for example with the mixer's ducking) to set how loud it plays.

``get_played_bytes()`` / ``reset_played_bytes()`` count the audio that went
through, so a lambda can tell whether a playback produced any sound.
"""

import esphome.codegen as cg
from esphome.components import audio, speaker
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_OUTPUT_SPEAKER

from . import tab5_broadcast_ns

AUTO_LOAD = ["audio"]
CODEOWNERS = ["@Krusnix"]

BroadcastSpeaker = tab5_broadcast_ns.class_(
    "BroadcastSpeaker", cg.Component, speaker.Speaker
)


def _set_stream_limits(config):
    audio.set_stream_limits(
        min_bits_per_sample=16,
        max_bits_per_sample=16,
        min_channels=1,
        max_channels=2,
    )(config)
    return config


CONFIG_SCHEMA = cv.All(
    speaker.SPEAKER_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(BroadcastSpeaker),
            cv.Required(CONF_OUTPUT_SPEAKER): cv.use_id(speaker.Speaker),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
    _set_stream_limits,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await speaker.register_speaker(var, config)
    out = await cg.get_variable(config[CONF_OUTPUT_SPEAKER])
    cg.add(var.set_output_speaker(out))
