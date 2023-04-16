/*
 modbus_registers.h - Modbus Registers Structure
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

#ifndef SRC_MODBUS_REGISTERS_H_
#define SRC_MODBUS_REGISTERS_H_

#include "Arduino.h"

typedef enum {
    MODBUS_TYPE_HOLDING = 0x00,         /*!< Modbus Holding register. */
    MODBUS_TYPE_INPUT,                  /*!< Modbus Input register. */
    MODBUS_TYPE_COIL,                   /*!< Modbus Coils. */
    MODBUS_TYPE_READ_COILS,             /*!< Modbus Read Coils. */
    MODBUS_TYPE_DISCRETE,               /*!< Modbus Discrete bits. */     
//    MODBUS_TYPE_COUNT,
//    MODBUS_TYPE_UNKNOWN = 0xFF
} modbus_entity_t;

typedef enum {
//    REGISTER_TYPE_U8 = 0x00,                   /*!< Unsigned 8 */
    REGISTER_TYPE_U16 = 0x01,                  /*!< Unsigned 16 */
//    REGISTER_TYPE_U32 = 0x02,                  /*!< Unsigned 32 */
//    REGISTER_TYPE_FLOAT = 0x03,                /*!< Float type */
//    REGISTER_TYPE_ASCII = 0x04,                 /*!< ASCII type */
    REGISTER_TYPE_DIEMATIC_ONE_DECIMAL = 0x05,
    REGISTER_TYPE_BITFIELD = 0x06,
    REGISTER_TYPE_DEBUG = 0x07
} register_type_t;

typedef union {
    const char* bitfield[16];
} optional_param_t;

typedef struct {
    uint16_t            id;
    modbus_entity_t     modbus_entity;      /*!< Type of modbus parameter */
    register_type_t     type;               /*!< Float, U8, U16, U32, ASCII, etc. */
    const char*         name;
    optional_param_t    optional_param;
} modbus_register_t;

const modbus_register_t read_registers[] = {
    { 2, MODBUS_TYPE_HOLDING, REGISTER_TYPE_U16, "slave_address" },
    { 3, MODBUS_TYPE_HOLDING, REGISTER_TYPE_U16, "baudrate" },
    { 4, MODBUS_TYPE_HOLDING, REGISTER_TYPE_U16, "parity_bits" },
    { 5, MODBUS_TYPE_HOLDING, REGISTER_TYPE_U16, "secure_delay" },
    { 0, MODBUS_TYPE_DISCRETE, REGISTER_TYPE_U16, "DI_0" },
    { 1, MODBUS_TYPE_DISCRETE, REGISTER_TYPE_U16, "DI_1" },
    { 2, MODBUS_TYPE_DISCRETE, REGISTER_TYPE_U16, "DI_2" },
    { 3, MODBUS_TYPE_DISCRETE, REGISTER_TYPE_U16, "DI_3" },
    { 4, MODBUS_TYPE_DISCRETE, REGISTER_TYPE_U16, "DI_4" },
    { 5, MODBUS_TYPE_DISCRETE, REGISTER_TYPE_U16, "DI_5" },
    { 6, MODBUS_TYPE_DISCRETE, REGISTER_TYPE_U16, "DI_6" },
    { 7, MODBUS_TYPE_DISCRETE, REGISTER_TYPE_U16, "DI_7" },
    { 8, MODBUS_TYPE_DISCRETE, REGISTER_TYPE_U16, "DI_8" },
    { 9, MODBUS_TYPE_DISCRETE, REGISTER_TYPE_U16, "DI_9" },
    { 10, MODBUS_TYPE_DISCRETE, REGISTER_TYPE_U16, "DI_10" },
    { 11, MODBUS_TYPE_DISCRETE, REGISTER_TYPE_U16, "DI_11" },
    { 12, MODBUS_TYPE_DISCRETE, REGISTER_TYPE_U16, "DI_12" },
    { 13, MODBUS_TYPE_DISCRETE, REGISTER_TYPE_U16, "DI_13" },
    { 14, MODBUS_TYPE_DISCRETE, REGISTER_TYPE_U16, "DI_14" },
    { 15, MODBUS_TYPE_DISCRETE, REGISTER_TYPE_U16, "DI_15" },
    { 0, MODBUS_TYPE_READ_COILS, REGISTER_TYPE_U16, "DO_0" },
    { 1, MODBUS_TYPE_READ_COILS, REGISTER_TYPE_U16, "DO_1" },
    { 2, MODBUS_TYPE_READ_COILS, REGISTER_TYPE_U16, "DO_2" },
    { 3, MODBUS_TYPE_READ_COILS, REGISTER_TYPE_U16, "DO_3" },
    { 4, MODBUS_TYPE_READ_COILS, REGISTER_TYPE_U16, "DO_4" },
    { 5, MODBUS_TYPE_READ_COILS, REGISTER_TYPE_U16, "DO_5" },
    { 6, MODBUS_TYPE_READ_COILS, REGISTER_TYPE_U16, "DO_6" },
    { 7, MODBUS_TYPE_READ_COILS, REGISTER_TYPE_U16, "DO_7" },
    { 8, MODBUS_TYPE_READ_COILS, REGISTER_TYPE_U16, "DO_8" },
    { 9, MODBUS_TYPE_READ_COILS, REGISTER_TYPE_U16, "DO_9" },
    { 10, MODBUS_TYPE_READ_COILS, REGISTER_TYPE_U16, "DO_10" },
    { 11, MODBUS_TYPE_READ_COILS, REGISTER_TYPE_U16, "DO_11" },
    { 12, MODBUS_TYPE_READ_COILS, REGISTER_TYPE_U16, "DO_12" },
    { 13, MODBUS_TYPE_READ_COILS, REGISTER_TYPE_U16, "DO_13" },
    { 14, MODBUS_TYPE_READ_COILS, REGISTER_TYPE_U16, "DO_14" },
    { 15, MODBUS_TYPE_READ_COILS, REGISTER_TYPE_U16, "DO_15" }
};

const modbus_register_t write_registers[] = {
    { 0, MODBUS_TYPE_COIL, REGISTER_TYPE_U16, "DO_0" },
    { 1, MODBUS_TYPE_COIL, REGISTER_TYPE_U16, "DO_1" },
    { 2, MODBUS_TYPE_COIL, REGISTER_TYPE_U16, "DO_2" },
    { 3, MODBUS_TYPE_COIL, REGISTER_TYPE_U16, "DO_3" },
    { 4, MODBUS_TYPE_COIL, REGISTER_TYPE_U16, "DO_4" },
    { 5, MODBUS_TYPE_COIL, REGISTER_TYPE_U16, "DO_5" },
    { 6, MODBUS_TYPE_COIL, REGISTER_TYPE_U16, "DO_6" },
    { 7, MODBUS_TYPE_COIL, REGISTER_TYPE_U16, "DO_7" },
    { 8, MODBUS_TYPE_COIL, REGISTER_TYPE_U16, "DO_8" },
    { 9, MODBUS_TYPE_COIL, REGISTER_TYPE_U16, "DO_9" },
    { 10, MODBUS_TYPE_COIL, REGISTER_TYPE_U16, "DO_10" },
    { 11, MODBUS_TYPE_COIL, REGISTER_TYPE_U16, "DO_11" },
    { 12, MODBUS_TYPE_COIL, REGISTER_TYPE_U16, "DO_12" },
    { 13, MODBUS_TYPE_COIL, REGISTER_TYPE_U16, "DO_13" },
    { 14, MODBUS_TYPE_COIL, REGISTER_TYPE_U16, "DO_14" },
    { 15, MODBUS_TYPE_COIL, REGISTER_TYPE_U16, "DO_15" },
};

#endif  // SRC_MODBUS_REGISTERS_H_
