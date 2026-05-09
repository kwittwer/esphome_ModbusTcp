import esphome.codegen as cg
from esphome.components import sensor
from esphome.components.modbus.helpers import SENSOR_VALUE_TYPE, TYPE_REGISTER_MAP
import esphome.config_validation as cv
from esphome.const import (
    CONF_ADDRESS,
    CONF_ID,
    CONF_OFFSET,
    CONF_PORT,
    CONF_TIMEOUT,
)

DEPENDENCIES = ["wifi"]
MULTI_CONF = True

CONF_HOST = "host"
CONF_REGISTER_TYPE = "register_type"
CONF_REGISTERS = "registers"
CONF_SCALE = "scale"
CONF_UNIT_ID = "unit_id"
CONF_VALUE_TYPE = "value_type"

modbus_tcp_master_ns = cg.esphome_ns.namespace("modbus_tcp_master")
ModbusTcpMaster = modbus_tcp_master_ns.class_(
    "ModbusTcpMaster", cg.PollingComponent
)
ModbusTcpSensor = modbus_tcp_master_ns.class_("ModbusTcpSensor", sensor.Sensor)

REGISTER_TYPE_TO_FUNCTION_CODE = {
    "coil": 0x01,
    "discrete": 0x02,
    "holding": 0x03,
    "input": 0x04,
    "read": 0x04,
}

MASTER_REGISTER_SCHEMA = sensor.sensor_schema(ModbusTcpSensor).extend(
    {
        cv.Required(CONF_ADDRESS): cv.positive_int,
        cv.Optional(CONF_REGISTER_TYPE, default="holding"): cv.one_of(
            "holding", "input", "read", "coil", "discrete", lower=True
        ),
        cv.Optional(CONF_VALUE_TYPE, default="U_WORD"): cv.enum(SENSOR_VALUE_TYPE),
        cv.Optional(CONF_SCALE, default=1.0): cv.float_,
        cv.Optional(CONF_OFFSET, default=0.0): cv.float_,
    }
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(ModbusTcpMaster),
        cv.Required(CONF_HOST): cv.string_strict,
        cv.Optional(CONF_PORT, default=502): cv.port,
        cv.Optional(CONF_UNIT_ID, default=1): cv.int_range(min=0, max=255),
        cv.Optional(
            CONF_TIMEOUT, default="5s"
        ): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_REGISTERS, default=[]): cv.ensure_list(MASTER_REGISTER_SCHEMA),
    }
).extend(cv.polling_component_schema("60s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_host(config[CONF_HOST]))
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_unit_id(config[CONF_UNIT_ID]))
    cg.add(var.set_timeout(config[CONF_TIMEOUT]))

    for register in config[CONF_REGISTERS]:
        sens = await sensor.new_sensor(register)
        cg.add(sens.set_address(register[CONF_ADDRESS]))
        function_code = REGISTER_TYPE_TO_FUNCTION_CODE[register[CONF_REGISTER_TYPE]]
        cg.add(sens.set_function_code(function_code))
        cg.add(sens.set_value_type(register[CONF_VALUE_TYPE]))
        register_count = (
            1
            if register[CONF_REGISTER_TYPE] in ("coil", "discrete")
            else TYPE_REGISTER_MAP[register[CONF_VALUE_TYPE]]
        )
        cg.add(sens.set_register_count(register_count))
        cg.add(sens.set_scale(register[CONF_SCALE]))
        cg.add(sens.set_offset(register[CONF_OFFSET]))
        cg.add(var.add_sensor(sens))
