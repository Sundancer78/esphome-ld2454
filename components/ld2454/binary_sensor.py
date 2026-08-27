import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import binary_sensor
from esphome.const import (
    DEVICE_CLASS_CONNECTIVITY,
    DEVICE_CLASS_PRESENCE,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

from . import CONF_LD2454_ID, LD2454Component


DEPENDENCIES = ["ld2454"]

CONF_PRESENCE = "presence"
CONF_ONLINE = "online"


CONFIG_SCHEMA = {
    cv.GenerateID(CONF_LD2454_ID): cv.use_id(LD2454Component),

    cv.Optional(CONF_PRESENCE): binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_PRESENCE,
    ),

    cv.Optional(CONF_ONLINE): binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_CONNECTIVITY,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
}


async def to_code(config):
    parent = await cg.get_variable(config[CONF_LD2454_ID])

    if presence_config := config.get(CONF_PRESENCE):
        sens = await binary_sensor.new_binary_sensor(presence_config)
        cg.add(parent.set_presence_binary_sensor(sens))
    if online_config := config.get(CONF_ONLINE):
        sens = await binary_sensor.new_binary_sensor(online_config)
        cg.add(parent.set_online_binary_sensor(sens))
