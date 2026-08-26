import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import text_sensor

from . import CONF_LD2454_ID, LD2454Component


DEPENDENCIES = ["ld2454"]

CONF_TARGET_1_DIRECTION = "target_1_direction"
CONF_TARGET_2_DIRECTION = "target_2_direction"
CONF_TARGET_3_DIRECTION = "target_3_direction"


CONFIG_SCHEMA = {
    cv.GenerateID(CONF_LD2454_ID): cv.use_id(LD2454Component),

    cv.Optional(CONF_TARGET_1_DIRECTION):
        text_sensor.text_sensor_schema(),

    cv.Optional(CONF_TARGET_2_DIRECTION):
        text_sensor.text_sensor_schema(),

    cv.Optional(CONF_TARGET_3_DIRECTION):
        text_sensor.text_sensor_schema(),
}


async def to_code(config):
    parent = await cg.get_variable(config[CONF_LD2454_ID])

    if target_1_config := config.get(CONF_TARGET_1_DIRECTION):
        sens = await text_sensor.new_text_sensor(target_1_config)
        cg.add(parent.set_target_direction_sensor(0, sens))

    if target_2_config := config.get(CONF_TARGET_2_DIRECTION):
        sens = await text_sensor.new_text_sensor(target_2_config)
        cg.add(parent.set_target_direction_sensor(1, sens))

    if target_3_config := config.get(CONF_TARGET_3_DIRECTION):
        sens = await text_sensor.new_text_sensor(target_3_config)
        cg.add(parent.set_target_direction_sensor(2, sens))