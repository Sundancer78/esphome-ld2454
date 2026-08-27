import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import switch
from esphome.const import ENTITY_CATEGORY_CONFIG

from . import CONF_LD2454_ID, LD2454Component, ld2454_ns

DEPENDENCIES = ["ld2454"]

CONF_MULTI_TARGET = "multi_target"

LD2454MultiTargetSwitch = ld2454_ns.class_(
    "LD2454MultiTargetSwitch",
    switch.Switch,
)

CONFIG_SCHEMA = {
    cv.GenerateID(CONF_LD2454_ID): cv.use_id(LD2454Component),
    cv.Optional(CONF_MULTI_TARGET): switch.switch_schema(
        LD2454MultiTargetSwitch,
        entity_category=ENTITY_CATEGORY_CONFIG,
    ),
}


async def to_code(config):
    parent = await cg.get_variable(config[CONF_LD2454_ID])

    if multi_target_config := config.get(CONF_MULTI_TARGET):
        sw = await switch.new_switch(multi_target_config)
        cg.add(sw.set_parent(parent))
        cg.add(parent.set_multi_target_switch(sw))
