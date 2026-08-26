import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import sensor
from esphome.const import (
    STATE_CLASS_MEASUREMENT,
    UNIT_DEGREES,
    UNIT_MILLIMETER,
)

from . import CONF_LD2454_ID, LD2454Component

DEPENDENCIES = ["ld2454"]

CONF_TARGET_COUNT = "target_count"
CONF_MOVING_TARGET_COUNT = "moving_target_count"
CONF_STILL_TARGET_COUNT = "still_target_count"

CONF_TARGET_1_X = "target_1_x"
CONF_TARGET_1_Y = "target_1_y"
CONF_TARGET_1_DISTANCE = "target_1_distance"
CONF_TARGET_1_ANGLE = "target_1_angle"
CONF_TARGET_1_SPEED = "target_1_speed"
CONF_TARGET_1_RESOLUTION = "target_1_resolution"

CONF_TARGET_2_X = "target_2_x"
CONF_TARGET_2_Y = "target_2_y"
CONF_TARGET_2_DISTANCE = "target_2_distance"
CONF_TARGET_2_ANGLE = "target_2_angle"
CONF_TARGET_2_SPEED = "target_2_speed"
CONF_TARGET_2_RESOLUTION = "target_2_resolution"

CONF_TARGET_3_X = "target_3_x"
CONF_TARGET_3_Y = "target_3_y"
CONF_TARGET_3_DISTANCE = "target_3_distance"
CONF_TARGET_3_ANGLE = "target_3_angle"
CONF_TARGET_3_SPEED = "target_3_speed"
CONF_TARGET_3_RESOLUTION = "target_3_resolution"


POSITION_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_MILLIMETER,
    accuracy_decimals=0,
    state_class=STATE_CLASS_MEASUREMENT,
)

DISTANCE_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_MILLIMETER,
    accuracy_decimals=0,
    state_class=STATE_CLASS_MEASUREMENT,
)

ANGLE_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_DEGREES,
    accuracy_decimals=1,
    state_class=STATE_CLASS_MEASUREMENT,
)

SPEED_SCHEMA = sensor.sensor_schema(
    unit_of_measurement="mm/s",
    accuracy_decimals=0,
    state_class=STATE_CLASS_MEASUREMENT,
)

COUNT_SCHEMA = sensor.sensor_schema(
    accuracy_decimals=0,
    state_class=STATE_CLASS_MEASUREMENT,
)


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_LD2454_ID): cv.use_id(LD2454Component),

        cv.Optional(CONF_TARGET_COUNT): COUNT_SCHEMA,
        cv.Optional(CONF_MOVING_TARGET_COUNT): COUNT_SCHEMA,
        cv.Optional(CONF_STILL_TARGET_COUNT): COUNT_SCHEMA,

        cv.Optional(CONF_TARGET_1_X): POSITION_SCHEMA,
        cv.Optional(CONF_TARGET_1_Y): POSITION_SCHEMA,
        cv.Optional(CONF_TARGET_1_DISTANCE): DISTANCE_SCHEMA,
        cv.Optional(CONF_TARGET_1_ANGLE): ANGLE_SCHEMA,
        cv.Optional(CONF_TARGET_1_SPEED): SPEED_SCHEMA,
        cv.Optional(CONF_TARGET_1_RESOLUTION): POSITION_SCHEMA,

        cv.Optional(CONF_TARGET_2_X): POSITION_SCHEMA,
        cv.Optional(CONF_TARGET_2_Y): POSITION_SCHEMA,
        cv.Optional(CONF_TARGET_2_DISTANCE): DISTANCE_SCHEMA,
        cv.Optional(CONF_TARGET_2_ANGLE): ANGLE_SCHEMA,
        cv.Optional(CONF_TARGET_2_SPEED): SPEED_SCHEMA,
        cv.Optional(CONF_TARGET_2_RESOLUTION): POSITION_SCHEMA,

        cv.Optional(CONF_TARGET_3_X): POSITION_SCHEMA,
        cv.Optional(CONF_TARGET_3_Y): POSITION_SCHEMA,
        cv.Optional(CONF_TARGET_3_DISTANCE): DISTANCE_SCHEMA,
        cv.Optional(CONF_TARGET_3_ANGLE): ANGLE_SCHEMA,
        cv.Optional(CONF_TARGET_3_SPEED): SPEED_SCHEMA,
        cv.Optional(CONF_TARGET_3_RESOLUTION): POSITION_SCHEMA,
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_LD2454_ID])

    mappings = [
        (CONF_TARGET_1_X, "set_target_x_sensor", 0),
        (CONF_TARGET_1_Y, "set_target_y_sensor", 0),
        (CONF_TARGET_1_DISTANCE, "set_target_distance_sensor", 0),
        (CONF_TARGET_1_ANGLE, "set_target_angle_sensor", 0),
        (CONF_TARGET_1_SPEED, "set_target_speed_sensor", 0),
        (CONF_TARGET_1_RESOLUTION, "set_target_resolution_sensor", 0),

        (CONF_TARGET_2_X, "set_target_x_sensor", 1),
        (CONF_TARGET_2_Y, "set_target_y_sensor", 1),
        (CONF_TARGET_2_DISTANCE, "set_target_distance_sensor", 1),
        (CONF_TARGET_2_ANGLE, "set_target_angle_sensor", 1),
        (CONF_TARGET_2_SPEED, "set_target_speed_sensor", 1),
        (CONF_TARGET_2_RESOLUTION, "set_target_resolution_sensor", 1),

        (CONF_TARGET_3_X, "set_target_x_sensor", 2),
        (CONF_TARGET_3_Y, "set_target_y_sensor", 2),
        (CONF_TARGET_3_DISTANCE, "set_target_distance_sensor", 2),
        (CONF_TARGET_3_ANGLE, "set_target_angle_sensor", 2),
        (CONF_TARGET_3_SPEED, "set_target_speed_sensor", 2),
        (CONF_TARGET_3_RESOLUTION, "set_target_resolution_sensor", 2),
    ]

    for key, setter, target in mappings:
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(parent, setter)(target, sens))

    if CONF_TARGET_COUNT in config:
        sens = await sensor.new_sensor(config[CONF_TARGET_COUNT])
        cg.add(parent.set_target_count_sensor(sens))

    if CONF_MOVING_TARGET_COUNT in config:
        sens = await sensor.new_sensor(config[CONF_MOVING_TARGET_COUNT])
        cg.add(parent.set_moving_target_count_sensor(sens))

    if CONF_STILL_TARGET_COUNT in config:
        sens = await sensor.new_sensor(config[CONF_STILL_TARGET_COUNT])
        cg.add(parent.set_still_target_count_sensor(sens))