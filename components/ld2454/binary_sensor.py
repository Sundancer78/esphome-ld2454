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
CONF_MOVING_PRESENCE = "moving_presence"
CONF_STILL_PRESENCE = "still_presence"

CONF_TARGET_1_ACTIVE = "target_1_active"
CONF_TARGET_2_ACTIVE = "target_2_active"
CONF_TARGET_3_ACTIVE = "target_3_active"

CONF_ONLINE = "online"


CONFIG_SCHEMA = {
    cv.GenerateID(CONF_LD2454_ID): cv.use_id(LD2454Component),

    cv.Optional(CONF_PRESENCE): binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_PRESENCE,
    ),

    cv.Optional(CONF_MOVING_PRESENCE): binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_PRESENCE,
    ),

    cv.Optional(CONF_STILL_PRESENCE): binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_PRESENCE,
    ),

    cv.Optional(CONF_TARGET_1_ACTIVE): binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_PRESENCE,
    ),

    cv.Optional(CONF_TARGET_2_ACTIVE): binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_PRESENCE,
    ),

    cv.Optional(CONF_TARGET_3_ACTIVE): binary_sensor.binary_sensor_schema(
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

    if moving_config := config.get(CONF_MOVING_PRESENCE):
        sens = await binary_sensor.new_binary_sensor(moving_config)
        cg.add(parent.set_moving_presence_binary_sensor(sens))

    if still_config := config.get(CONF_STILL_PRESENCE):
        sens = await binary_sensor.new_binary_sensor(still_config)
        cg.add(parent.set_still_presence_binary_sensor(sens))

    if target_1_config := config.get(CONF_TARGET_1_ACTIVE):
        sens = await binary_sensor.new_binary_sensor(target_1_config)
        cg.add(parent.set_target_active_binary_sensor(0, sens))

    if target_2_config := config.get(CONF_TARGET_2_ACTIVE):
        sens = await binary_sensor.new_binary_sensor(target_2_config)
        cg.add(parent.set_target_active_binary_sensor(1, sens))

    if target_3_config := config.get(CONF_TARGET_3_ACTIVE):
        sens = await binary_sensor.new_binary_sensor(target_3_config)
        cg.add(parent.set_target_active_binary_sensor(2, sens))

    if online_config := config.get(CONF_ONLINE):
        sens = await binary_sensor.new_binary_sensor(online_config)
        cg.add(parent.set_online_binary_sensor(sens))