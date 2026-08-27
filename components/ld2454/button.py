import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import button
from esphome.const import DEVICE_CLASS_RESTART, ENTITY_CATEGORY_CONFIG

from . import CONF_LD2454_ID, LD2454Component, ld2454_ns

DEPENDENCIES = ["ld2454"]

CONF_RESTART = "restart"
CONF_FACTORY_RESET = "factory_reset"

LD2454RestartButton = ld2454_ns.class_(
    "LD2454RestartButton",
    button.Button,
)

LD2454FactoryResetButton = ld2454_ns.class_(
    "LD2454FactoryResetButton",
    button.Button,
)

CONFIG_SCHEMA = {
    cv.GenerateID(CONF_LD2454_ID): cv.use_id(LD2454Component),
    cv.Optional(CONF_RESTART): button.button_schema(
        LD2454RestartButton,
        entity_category=ENTITY_CATEGORY_CONFIG,
        device_class=DEVICE_CLASS_RESTART,
    ),
    cv.Optional(CONF_FACTORY_RESET): button.button_schema(
        LD2454FactoryResetButton,
        entity_category=ENTITY_CATEGORY_CONFIG,
    ),
}


async def to_code(config):
    parent = await cg.get_variable(config[CONF_LD2454_ID])

    if restart_config := config.get(CONF_RESTART):
        btn = await button.new_button(restart_config)
        cg.add(btn.set_parent(parent))

    if factory_reset_config := config.get(CONF_FACTORY_RESET):
        btn = await button.new_button(factory_reset_config)
        cg.add(btn.set_parent(parent))
