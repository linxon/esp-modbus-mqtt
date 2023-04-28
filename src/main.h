/*
 main.h - esp-modbus-mqtt
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

#ifndef SRC_MAIN_H_
#define SRC_MAIN_H_

#include "Arduino.h"
#include "credentials.h"

#define MB_MAX_BUFFER_SIZE      1600u

#define NTP_GMT_OFFSET          3600l
#define NTP_DAYLIGHT_OFFSET     7200
#define NTP_SERVER_1            "0.ru.pool.ntp.org"
#define NTP_SERVER_2            "1.ru.pool.ntp.org"
#define NTP_SERVER_3            "2.ru.pool.ntp.org"

#define MQTT_CLIENT_PUBLISH_DATA(mq_client, target_topic, buffer, length) \
    do { \
        if (mq_client.connected()) { \
            String mqtt_topic = MQTT_TOPIC; \
            mqtt_topic += "/" + String(HOSTNAME) + "/" + target_topic; \
            ESP_LOGI(TAG, "MQTT Publishing data to topic: %s", mqtt_topic.c_str()); \
            mq_client.publish(mqtt_topic.c_str(), 2, true, buffer, length); \
        } \
    } while (0)

#endif  // SRC_MAIN_H_
