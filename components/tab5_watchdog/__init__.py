"""Keeps the ESPHome main loop from being frozen out, and says what froze it.

When the main loop can't run for longer than the task watchdog allows (5 s
by default), ESP-IDF restarts the chip, and ESPHome's crash report only
shows where the processor that restarted happened to be. This component:

- looks every 25 ms, from the other core, at which task each core runs, and
  keeps the last 2.5 s;
- rescues the main loop when another task has kept it from running for
  2.5 s: that task gets the main loop's priority (1) so the two take turns,
  until it calms down. This is logged (``tab5_watchdog``) with the task's
  name;
- if the chip is restarted anyway, notes just before the restart which task
  each core was running, what they ran most in the last 2.5 s, whether the
  main loop was running or waiting, where it was in the code (return
  addresses on its stack) and which ESPHome component it was in.

After a restart it logs this with the code addresses written as
``BT0: 0x...``, which the ESPHome Builder's log view turns into function
names, and ``get_report()`` gives a one-line summary for a text sensor.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.core import CORE

CODEOWNERS = ["@Krusnix"]
DEPENDENCIES = ["esp32"]

tab5_watchdog_ns = cg.esphome_ns.namespace("tab5_watchdog")
Tab5Watchdog = tab5_watchdog_ns.class_("Tab5Watchdog", cg.Component)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Tab5Watchdog),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
)


async def to_code(config):
    if CORE.using_arduino:
        raise cv.Invalid("tab5_watchdog requires the esp-idf framework")
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
