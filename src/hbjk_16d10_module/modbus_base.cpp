/*
 modbus_base.cpp - Modbus functions
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

#include "hbjk_16d10_module/modbus_base.h"
#include "hbjk_16d10_module/modbus_registers.h"

#include "Arduino.h"
#include <ModbusMaster.h>
#include <ArduinoJson.h>


static const char __attribute__((__unused__)) *TAG = "Modbus_base";

/* The following symbols are passed via BUILD parameters
#define RXD 27 // aka R0
#define TXD 26 // aka DI
#define RTS 25 // aka DE and /RE
//if there is no flow control PIN
//#define RTS            NOT_A_PIN

#define MODBUS_BAUDRATE 9600
#define MODBUS_UNIT 10
#define MODBUS_RETRIES 2
#define MODBUS_SCANRATE 30 // in seconds
*/

// instantiate ModbusMaster object
ModbusMaster modbus_client;

void preTransmission() {
  digitalWrite(RTS, 1);
}

void postTransmission() {
  digitalWrite(RTS, 0);
}

void initModbus(uint8_t slave_id) {
  Serial2.begin(MODBUS_BAUDRATE, SERIAL_8N1, RXD, TXD);  // Using ESP32 UART2 for Modbus
  modbus_client.begin(slave_id, Serial2);

  // do we have a flow control pin?
  if (RTS != NOT_A_PIN) {
    // Init in receive mode
    pinMode(RTS, OUTPUT);
    digitalWrite(RTS, 0);

    // Callbacks allow us to configure the RS485 transceiver correctly
    modbus_client.preTransmission(preTransmission);
    modbus_client.postTransmission(postTransmission);
  }
}

bool _getModbusResultMsg(ModbusMaster *node, uint8_t result) {
  String tmpstr2 = "";
  switch (result) {
    case node->ku8MBSuccess:
      return true;
      break;
      case node->ku8MBIllegalFunction:
      tmpstr2 += "Illegal Function";
      break;
    case node->ku8MBIllegalDataAddress:
      tmpstr2 += "Illegal Data Address";
      break;
    case node->ku8MBIllegalDataValue:
      tmpstr2 += "Illegal Data Value";
      break;
    case node->ku8MBSlaveDeviceFailure:
      tmpstr2 += "Slave Device Failure";
      break;
    case node->ku8MBInvalidSlaveID:
      tmpstr2 += "Invalid Slave ID";
      break;
    case node->ku8MBInvalidFunction:
      tmpstr2 += "Invalid Function";
      break;
    case node->ku8MBResponseTimedOut:
      tmpstr2 += "Response Timed Out";
      break;
    case node->ku8MBInvalidCRC:
      tmpstr2 += "Invalid CRC";
      break;
    default:
      tmpstr2 += "Unknown error: " + String(result);
      break;
  }
  ESP_LOGW(TAG, "%s", tmpstr2.c_str());
  return false;
}

bool _getModbusValue(uint16_t register_id, modbus_entity_t modbus_entity, uint16_t *value_ptr) {
  uint8_t result;
  ESP_LOGD(TAG, "Requesting data");
  for (uint8_t i = 1; i <= MODBUS_RETRIES + 1; ++i) {
    ESP_LOGV(TAG, "Trial %d/%d", i, MODBUS_RETRIES + 1);
    switch (modbus_entity) {
      case MODBUS_TYPE_HOLDING:
        result = modbus_client.readHoldingRegisters(register_id, 1);
        if (_getModbusResultMsg(&modbus_client, result)) {
          *value_ptr = modbus_client.getResponseBuffer(0);
          ESP_LOGV(TAG, "Data read: %x", *value_ptr);
          return true;
        }
        break;
      case MODBUS_TYPE_INPUT:
        result = modbus_client.readInputRegisters(register_id, 1);
        if (_getModbusResultMsg(&modbus_client, result)) {
          *value_ptr = modbus_client.getResponseBuffer(0);
          ESP_LOGV(TAG, "Data read: %x", *value_ptr);
          return true;
        }
        break;
      case MODBUS_TYPE_DISCRETE:
        result = modbus_client.readDiscreteInputs(register_id, 1);
        if (_getModbusResultMsg(&modbus_client, result)) {
          *value_ptr = modbus_client.getResponseBuffer(0);
          ESP_LOGV(TAG, "Data read: %x", *value_ptr);
          return true;
        }
        break;
      case MODBUS_TYPE_READ_COILS:
        result = modbus_client.readCoils(register_id, 1);
        if (_getModbusResultMsg(&modbus_client, result)) {
          *value_ptr = modbus_client.getResponseBuffer(0);
          ESP_LOGV(TAG, "Data read: %x", *value_ptr);
          return true;
        }
        break;
      default:
        ESP_LOGW(TAG, "Unsupported Modbus entity type");
        value_ptr = nullptr;
        return false;
        break;
    }
  }
  // Time-out
  ESP_LOGW(TAG, "Time-out");
  value_ptr = nullptr;
  return false;
}

String _toBinary(uint16_t input) {
    String output;
    while (input != 0) {
      output = (input % 2 == 0 ? "0" : "1") + output;
      input /= 2;
    }
    return output;
}

bool _decodeDiematicDecimal(uint16_t int_input, int8_t decimals, float *value_ptr) {
  ESP_LOGV(TAG, "Decoding %#x with %d decimal(s)", int_input, decimals);
  if (int_input == 65535) {
    value_ptr = nullptr;
    return false;
  } else {
    uint16_t masked_input = int_input & 0x7FFF;
    float output = static_cast<float>(masked_input);
    if (int_input >> 15 == 1) {
      output = -output;
    }
    *value_ptr = output / pow(10, decimals);
    ESP_LOGV(TAG, "Decoded value: %f", *value_ptr);
    return true;
  }
}

void readModbusRegisterToJson(uint16_t idx, uint16_t register_id, ArduinoJson::JsonVariant variant) {
  // searchin for register matching register_id
  const uint8_t item_nb = sizeof(read_registers) / sizeof(modbus_register_t);
  for (uint8_t i = 0; i < item_nb; ++i) {

    if (read_registers[i].id != register_id) {
      // not this one
      continue;
    } else if (idx != i) {
      // skip registers if id number is not equal to target idx
      continue;
    } else {
      // register found
      ESP_LOGD(TAG, "Register id=%d type=0x%x name=%s", read_registers[i].id, read_registers[i].type, registers[i].name);
      uint16_t raw_value;
      if (_getModbusValue(read_registers[i].id, read_registers[i].modbus_entity, &raw_value)) {
        ESP_LOGV(TAG, "Raw value: %s=%#06x", read_registers[i].name, raw_value);
        switch (read_registers[i].type) {
          case REGISTER_TYPE_U16:
            ESP_LOGV(TAG, "Value: %u", raw_value);
            variant[read_registers[i].name] = raw_value;
            break;
          case REGISTER_TYPE_DIEMATIC_ONE_DECIMAL:
            float final_value;
            if (_decodeDiematicDecimal(raw_value, 1, &final_value)) {
              ESP_LOGV(TAG, "Value: %.1f", final_value);
              variant[read_registers[i].name] = final_value;
            } else {
              ESP_LOGD(TAG, "Value: Invalid Diematic value");
            }
            break;
          case REGISTER_TYPE_BITFIELD:
            for (uint8_t j = 0; j < 16; ++j) {
              const char *bit_varname = read_registers[i].optional_param.bitfield[j];
              if (bit_varname == nullptr) {
                ESP_LOGV(TAG, " [bit%02d] end of bitfield reached", j);
                break;
              }
              const uint8_t bit_value = raw_value >> j & 1;
              ESP_LOGV(TAG, " [bit%02d] %s=%d", j, bit_varname, bit_value);
              variant[bit_varname] = bit_value;
            }
            break;
          case REGISTER_TYPE_DEBUG:
            ESP_LOGI(TAG, "Raw DEBUG value: %s=%#06x %s", read_registers[i].name, raw_value, _toBinary(raw_value).c_str());
            break;
          default:
            // Unsupported type
            ESP_LOGW(TAG, "Unsupported register type");
            break;
        }
      } else {
        ESP_LOGW(TAG, "Request failed!");
      }

      // Задержка требуется для того, чтобы запросы Modbus клиента не захлебывались
      vTaskDelay(2 / portTICK_PERIOD_MS); // не меньше 2ms

      return;  // break the loop and exit the function
    }
  }
  // register not found
}

void writeJsonToModbusRegisters(uint16_t idx, uint16_t register_id, ArduinoJson::JsonVariant variant) {

}

void parseModbusToJson(ArduinoJson::JsonVariant variant) {
  ESP_LOGV(TAG, "Parsing all Modbus registers (Logging Tag: %s)", TAG);
  uint8_t item_nb = sizeof(read_registers) / sizeof(modbus_register_t);
  for (uint8_t i = 0; i < item_nb; ++i) {
    readModbusRegisterToJson(i, read_registers[i].id, variant);
  }
}

modbus_state_target_t modbusGetState_target(ArduinoJson::JsonVariant variant) {
  ESP_LOGI(TAG, "Parsing target Modbus register (Logging Tag: %s)", TAG);

  uint8_t result;
  uint8_t item_nb = sizeof(write_registers) / sizeof(modbus_register_t);

  for (uint8_t i = 0; i < item_nb; ++i) {
    if (!variant.containsKey(write_registers[i].name))
      continue;

    if (write_registers[i].modbus_entity == MODBUS_TYPE_COIL) {
      result = modbus_client.readCoils(write_registers[i].id, 1);
      if (_getModbusResultMsg(&modbus_client, result)) {
        variant[write_registers[i].name] = modbus_client.getResponseBuffer(0);

        return true;
      }
    }
  }

  return false;
}

void parseJsonToModbus_single(ArduinoJson::JsonVariant variant) {
  ESP_LOGV(TAG, "Parsing received Json params (Logging Tag: %s)", TAG);
  uint8_t item_nb = sizeof(write_registers) / sizeof(modbus_register_t);

  for (uint8_t i = 0; i < item_nb; ++i) {
    if (!variant.containsKey(write_registers[i].name))
      continue;

    switch(write_registers[i].modbus_entity) {
      case MODBUS_TYPE_COIL:
        ESP_LOGI(TAG, "JSON RES: %s=%d", write_registers[i].name, (bool) variant[write_registers[i].name]);
        modbus_client.writeSingleCoil(write_registers[i].id, (bool) variant[write_registers[i].name]);

        return;
        break;
    }
  }
}

void parseJsonToModbus_multiple(ArduinoJson::JsonVariant variant) {
  uint16_t buffer = 0u;

  ESP_LOGV(TAG, "Parsing received Json params (Logging Tag: %s)", TAG);
  uint8_t item_nb = sizeof(write_registers) / sizeof(modbus_register_t);

  for (uint8_t i = 0; i < item_nb; ++i) {
    ESP_LOGI(TAG, "JSON RES: %s=%d", write_registers[i].name, (bool) variant[write_registers[i].name]);
    buffer |= (uint16_t) ((bool) variant[write_registers[i].name] << i);
  }

  modbus_client.clearTransmitBuffer();
  modbus_client.setTransmitBuffer(0, buffer);
  modbus_client.writeMultipleCoils(0, item_nb);
}

void jk16_d10_set_DO_port(uint16_t port_id, bool state) {
  StaticJsonDocument<60> json_doc;
  JsonObject obj = json_doc.to<JsonObject>();

  uint8_t item_nb = sizeof(write_registers) / sizeof(modbus_register_t);
  for (uint8_t i = 0; i < item_nb; ++i) {
    if (port_id != write_registers[i].id)
      continue;

    obj[write_registers[i].name] = state;
    parseJsonToModbus_single(json_doc.as<JsonVariant>());
  }
}

bool jk16_d10_get_DI_port(uint16_t port_id) {
  uint8_t result;
  uint8_t item_nb = sizeof(read_registers) / sizeof(modbus_register_t);

  for (uint8_t i = 0; i < item_nb; ++i) {
    if (port_id != read_registers[i].id)
      continue;
    
    if (read_registers[i].modbus_entity != MODBUS_TYPE_DISCRETE)
      continue;

    result = modbus_client.readDiscreteInputs(read_registers[i].id, 1);

    if (_getModbusResultMsg(&modbus_client, result))
      return (bool) modbus_client.getResponseBuffer(0);

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }

  return false;
}
