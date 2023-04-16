/*
 modbus_base.h - Modbus functions headers
 Copyright (C) 2020 Germain Masse

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef SRC_MODBUS_BASE_H_
#define SRC_MODBUS_BASE_H_

#include <ModbusMaster.h>
#include <ArduinoJson.h>

typedef bool modbus_state_target_t;

void preTransmission();
void postTransmission();
void initModbus(uint8_t slave_id);
void readModbusRegisterToJson(uint16_t idx, uint16_t register_id, ArduinoJson::JsonVariant variant);
void writeJsonToModbusRegisters(uint16_t idx, uint16_t register_id, ArduinoJson::JsonVariant variant);
void parseModbusToJson(ArduinoJson::JsonVariant variant);
modbus_state_target_t modbusGetState_target(ArduinoJson::JsonVariant variant);
void parseJsonToModbus_single(ArduinoJson::JsonVariant variant);
void parseJsonToModbus_multiple(ArduinoJson::JsonVariant variant);

void jk16_d10_set_DO_port(uint16_t port_id, bool state);
bool jk16_d10_get_DI_port(uint16_t port_id);

#endif  // SRC_MODBUS_BASE_H_
