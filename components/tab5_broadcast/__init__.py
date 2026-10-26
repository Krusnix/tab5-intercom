"""Broadcasts between Tab5s (voice clips over the home network).

The hub (``tab5_broadcast:``) keeps this device's newest recorded clip and
serves it as a WAV file on a small HTTP server of its own (plain http, on
the home network only). The other Tab5 plays it straight from here with its
media player, so the audio never goes through Home Assistant.

``publish_clip()`` (from a lambda) copies 16-bit mono samples into a new
clip and returns its address, ``http://<ip>:<port>/broadcast/<id>.wav``.
Only the newest clip is kept; an older address then answers 404.

The ``tab5_broadcast`` speaker platform (speaker.py) is the other half: a
pass-through speaker for a media player that keeps volume and mute to
itself instead of passing them down to the hardware speaker.
"""

import esphome.codegen as cg
from esphome.components import psram, socket
from esphome.components.esp32 import include_builtin_idf_component
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_PORT
from esphome.core import CORE

CODEOWNERS = ["@Krusnix"]
DEPENDENCIES = ["esp32", "network"]

tab5_broadcast_ns = cg.esphome_ns.namespace("tab5_broadcast")
ClipServer = tab5_broadcast_ns.class_("ClipServer", cg.Component)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ClipServer),
            cv.Optional(CONF_PORT, default=8099): cv.port,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
    # ESP-IDF's HTTP server: two clients at a time, its listening socket,
    # and its internal control sockets.
    socket.consume_sockets(2, "tab5_broadcast"),
    socket.consume_sockets(1, "tab5_broadcast", socket.SocketType.TCP_LISTEN),
    socket.consume_sockets(2, "tab5_broadcast", socket.SocketType.UDP),
)


async def to_code(config):
    if CORE.using_arduino:
        raise cv.Invalid("tab5_broadcast requires the esp-idf framework")
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add_define("USE_TAB5_BROADCAST_SERVER")
    include_builtin_idf_component("esp_http_server")
    # The server's task keeps its stack in PSRAM.
    psram.request_external_task_stack()
