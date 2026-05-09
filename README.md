# esphome_ModbusTcp

Externe ESPHome-Komponenten für Modbus TCP.

## Enthaltene Komponenten

- `modbus_tcp_master`: Liest Coils, Discrete Inputs, Holding- oder Input-Register (FC 0x01/0x02/0x03/0x04) eines Modbus-TCP-Servers und veröffentlicht sie als Sensoren.
- `modbus_tcp_slave`: Stellt einen einfachen Modbus-TCP-Server mit konfigurierbaren Registern sowie Lese-/Schreib-Lambdas bereit (FC 0x01/0x02/0x03/0x04/0x05/0x06/0x0F/0x10).

## Beispiele

- `examples/modbus_tcp_master.yaml`
- `examples/modbus_tcp_slave.yaml`
