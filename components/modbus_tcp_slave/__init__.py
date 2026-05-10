import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ADDRESS, CONF_ID, CONF_PORT

DEPENDENCIES = ["wifi"]
MULTI_CONF = True

TYPE_REGISTER_MAP = {
    "U_WORD": 1,
    "S_WORD": 1,
    "U_DWORD": 2,
    "S_DWORD": 2,
    "U_DWORD_R": 2,
    "S_DWORD_R": 2,
    "FP32": 2,
    "FP32_R": 2,
    "U_QWORD": 4,
    "S_QWORD": 4,
    "U_QWORD_R": 4,
    "S_QWORD_R": 4,
    "RAW": 0,
}

CPP_TYPE_REGISTER_MAP = {
    "U_WORD": cg.uint16,
    "S_WORD": cg.int16,
    "U_DWORD": cg.uint32,
    "S_DWORD": cg.int32,
    "U_DWORD_R": cg.uint32,
    "S_DWORD_R": cg.int32,
    "FP32": cg.float_,
    "FP32_R": cg.float_,
    "U_QWORD": cg.uint64,
    "S_QWORD": cg.int64,
    "U_QWORD_R": cg.uint64,
    "S_QWORD_R": cg.int64,
    "RAW": cg.uint8,
}

CONF_COURTESY_RESPONSE = "courtesy_response"
CONF_ENABLED = "enabled"
CONF_READ_LAMBDA = "read_lambda"
CONF_REGISTER_LAST_ADDRESS = "register_last_address"
CONF_REGISTER_VALUE = "register_value"
CONF_REGISTERS = "registers"
CONF_UNIT_ID = "unit_id"
CONF_VALUE_TYPE = "value_type"
CONF_WRITE_LAMBDA = "write_lambda"

modbus_tcp_slave_ns = cg.esphome_ns.namespace("modbus_tcp_slave")
ModbusTcpSlave = modbus_tcp_slave_ns.class_("ModbusTcpSlave", cg.Component)
ServerCourtesyResponse = modbus_tcp_slave_ns.struct("ServerCourtesyResponse")
ServerRegister = modbus_tcp_slave_ns.class_("ServerRegister")
modbus_ns = cg.esphome_ns.namespace("modbus")
modbus_helpers_ns = modbus_ns.namespace("helpers")
SensorValueType_ns = modbus_helpers_ns.namespace("SensorValueType")
SensorValueType = SensorValueType_ns.enum("SensorValueType")

# Local Modbus Value Type Mapping
SENSOR_VALUE_TYPE = {
    "U_WORD": SensorValueType.U_WORD,
    "S_WORD": SensorValueType.S_WORD,
    "U_DWORD": SensorValueType.U_DWORD,
    "S_DWORD": SensorValueType.S_DWORD,
    "U_DWORD_R": SensorValueType.U_DWORD_R,
    "S_DWORD_R": SensorValueType.S_DWORD_R,
    "FP32": SensorValueType.FP32,
    "FP32_R": SensorValueType.FP32_R,
    "U_QWORD": SensorValueType.U_QWORD,
    "S_QWORD": SensorValueType.S_QWORD,
    "U_QWORD_R": SensorValueType.U_QWORD_R,
    "S_QWORD_R": SensorValueType.S_QWORD_R,
    "RAW": SensorValueType.RAW,
}


SERVER_COURTESY_RESPONSE_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_ENABLED, default=False): cv.boolean,
        cv.Optional(CONF_REGISTER_LAST_ADDRESS, default=0xFFFF): cv.hex_uint16_t,
        cv.Optional(CONF_REGISTER_VALUE, default=0): cv.hex_uint16_t,
    }
)

SERVER_REGISTER_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(ServerRegister),
        cv.Required(CONF_ADDRESS): cv.positive_int,
        cv.Optional(CONF_VALUE_TYPE, default="U_WORD"): cv.enum(SENSOR_VALUE_TYPE),
        cv.Required(CONF_READ_LAMBDA): cv.returning_lambda,
        cv.Optional(CONF_WRITE_LAMBDA): cv.returning_lambda,
    }
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(ModbusTcpSlave),
        cv.Optional(CONF_PORT, default=502): cv.port,
        cv.Optional(CONF_UNIT_ID, default=1): cv.int_range(min=0, max=255),
        cv.Optional(CONF_COURTESY_RESPONSE): SERVER_COURTESY_RESPONSE_SCHEMA,
        cv.Optional(CONF_REGISTERS, default=[]): cv.ensure_list(SERVER_REGISTER_SCHEMA),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_unit_id(config[CONF_UNIT_ID]))

    if courtesy_response := config.get(CONF_COURTESY_RESPONSE):
        cg.add(
            var.set_server_courtesy_response(
                cg.StructInitializer(
                    ServerCourtesyResponse,
                    ("enabled", courtesy_response[CONF_ENABLED]),
                    (
                        "register_last_address",
                        courtesy_response[CONF_REGISTER_LAST_ADDRESS],
                    ),
                    ("register_value", courtesy_response[CONF_REGISTER_VALUE]),
                )
            )
        )

    for register in config[CONF_REGISTERS]:
        register_var = cg.new_Pvariable(
            register[CONF_ID],
            register[CONF_ADDRESS],
            register[CONF_VALUE_TYPE],
            TYPE_REGISTER_MAP[register[CONF_VALUE_TYPE]],
        )
        cpp_type = CPP_TYPE_REGISTER_MAP[register[CONF_VALUE_TYPE]]
        cg.add(
            register_var.set_read_lambda(
                cg.TemplateArguments(cpp_type),
                await cg.process_lambda(
                    register[CONF_READ_LAMBDA],
                    [(cg.uint16, "address")],
                    return_type=cpp_type,
                ),
            )
        )
        if CONF_WRITE_LAMBDA in register:
            cg.add(
                register_var.set_write_lambda(
                    cg.TemplateArguments(cpp_type),
                    await cg.process_lambda(
                        register[CONF_WRITE_LAMBDA],
                        [(cg.uint16, "address"), (cpp_type, "x")],
                        return_type=cg.bool_,
                    ),
                )
            )
        cg.add(var.add_server_register(register_var))
