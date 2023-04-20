/*
 main.cpp - esp-modbus-mqtt
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

#include "main.h"

#include <esp_log.h>
#include <math.h>
#if defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#include <ESPmDNS.h>
// #include "esp32-hal-log.h"
#elif defined(ARDUINO_ARCH_ESP8266)
// TODO(gmasse): test ESP8266 #include <ESP8266WiFi.h>
// TODO(gmasse): test ESP8266 #include <ESP8266mDNS.h>
#endif
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
  #include "freertos/task.h"
}

#include <ArduinoJson.h>
#include <AsyncMqttClient.h>
#include <WiFiManager.h>

#include <Url.h>
#include "esp_base.h"
#include <rom/crc.h>
#ifndef MODBUS_DISABLED
//#include <modbus_base.h>
#include <hbjk_16d10_module/modbus_base.h>
#endif  // MODBUS_DISABLED

static char HOSTNAME[24] = "ESP-MM-FFFFFFFFFFFFFFFF";
static const char __attribute__((__unused__)) *TAG = "Main";

// static const char *FIRMWARE_URL = "https://domain.com/path/file.bin";
static const char *FIRMWARE_VERSION = "000.001.027";

static StaticJsonDocument<2000> json_doc_poll;  // instanciate JSON storage for poller
static StaticJsonDocument<2000> json_doc_main;  // instanciate JSON storage for receiver
static char mb_main_json_buffer[MB_BUFFER_SIZE];

static struct tm time_d;

// instanciate WiFiManager object
WiFiManager wifiManager;

// instanciate AsyncMqttClient object
AsyncMqttClient mqtt_client;

// instanciate timers
TimerHandle_t mqtt_reconnect_timer;
TimerHandle_t wifi_reconnect_timer;
TimerHandle_t modbus_poller_timer;

// instanciate task handlers
TaskHandle_t modbus_poller_task_handler = NULL;
TaskHandle_t modbus_main_task_handler = NULL;
TaskHandle_t ota_update_task_handler = NULL;

SemaphoreHandle_t modbus_poller_semaphore = NULL;
SemaphoreHandle_t modbus_main_semaphore = NULL;

QueueHandle_t json_data_queue = NULL;

static uint16_t volatile modbus_poller_last_crc16_v;
static bool volatile modbus_poller_force_crc16_chk = false;

void resetWiFi() {
  // Set WiFi to station mode
  // and disconnect from an AP if it was previously connected
  ESP_LOGD(TAG, "Resetting Wifi");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
}

void connectToWifi() {
  ESP_LOGD(TAG, "Connecting to '%s'", WIFI_SSID);
  WiFi.begin();
}

void connectToMqtt() {
  ESP_LOGD(TAG, "Connecting to MQTT");
  mqtt_client.connect();
}

void wiFiEvent(WiFiEvent_t event) {
  ESP_LOGD(TAG, "WiFi event: %d", event);
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      ESP_LOGI(TAG, "WiFi connected with IP address: %s", WiFi.localIP().toString().c_str());
      if (!MDNS.begin(HOSTNAME)) {  // init mdns
        ESP_LOGW(TAG, "Error setting up MDNS responder");
      }

      // часы нужны, чтобы работали TLS сертификаты
      configTime(3600, 7200, NTP_SERVER_1, NTP_SERVER_2, NTP_SERVER_3);  // init UTC time
      
      if (getLocalTime(&time_d)) {
        ESP_LOGI(TAG, "Time: %4d-%02d-%02d %02d:%02d:%02d",
          time_d.tm_year+1900, time_d.tm_mon+1, time_d.tm_mday,
          time_d.tm_hour, time_d.tm_min, time_d.tm_sec);
      } else {
        ESP_LOGW(TAG, "Failed to obtain time");
      }
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      ESP_LOGW(TAG, "WiFi lost connection");
      xTimerStop(modbus_poller_timer, 0);
      xTimerStop(mqtt_reconnect_timer, 0);
      xTimerStart(wifi_reconnect_timer, 0);
      break;
    default:
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  String mqtt_topic;

  ESP_LOGI(TAG, "Connected to MQTT");
  ESP_LOGD(TAG, "Session present: %s", sessionPresent ? "true" : "false");

  mqtt_topic = MQTT_TOPIC;
  mqtt_topic += "/" + String(HOSTNAME) + "/set";
  ESP_LOGI(TAG, "Subscribing at %s", mqtt_topic.c_str());
  mqtt_client.subscribe(mqtt_topic.c_str(), 1);

  mqtt_topic = MQTT_TOPIC;
  mqtt_topic += "/" + String(HOSTNAME) + "/get";
  ESP_LOGI(TAG, "Subscribing at %s", mqtt_topic.c_str());
  mqtt_client.subscribe(mqtt_topic.c_str(), 1);

  /*
  mqtt_topic = MQTT_TOPIC;
  mqtt_topic += "/" + String(HOSTNAME) + "/upgrade";
  ESP_LOGI(TAG, "Subscribing at %s", mqtt_topic.c_str());
  mqtt_client.subscribe(mqtt_topic.c_str(), 1);
  */

  if (xTimerStart(modbus_poller_timer, 0) != pdPASS) {
    // The timer could not be set into the Active state
  }
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  ESP_LOGW(TAG, "Disconnected from MQTT");

  if (xTimerStop(modbus_poller_timer, 0) != pdPASS) {
    // The timer could not be set into the Active state
  }

  if (WiFi.isConnected()) {
    xTimerStart(mqtt_reconnect_timer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  ESP_LOGD(TAG, "Subscribe acknowledged for packetId: %d qos: %d", packetId, qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  ESP_LOGD(TAG, "Unsubscribe acknowledged for packetId: %d", packetId);
}

void onMqttMessage(char *topic, char *payload,
  AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  ESP_LOGV(TAG, "Message received (topic=%s, qos=%d, dup=%d, retain=%d, len=%d, index=%d, total=%d): %s",
    topic, properties.qos, properties.dup, properties.retain, len, index, total, payload);

  String suffix = String(topic).substring(strlen(MQTT_TOPIC) + strlen(HOSTNAME) + 2);
  
  ESP_LOGI(TAG, "MQTT msg suffix=%s data=%s", suffix.c_str(), payload);

  if (suffix == "upgrade") {

    // нахой ненадо пока...
    ESP_LOGD(TAG, "MQTT OTA update requested");
    //vTaskResume(ota_update_task_handler);

  } else if (suffix == "get") {

    if (xTimerStop(modbus_poller_timer, 0) == pdFAIL) {
      ESP_LOGW(TAG, "Unable to reset Modbus Poller timer");
    }

    modbus_poller_force_crc16_chk = true;

    if (eTaskGetState(modbus_main_task_handler) == eSuspended) {
      xSemaphoreGive(modbus_poller_semaphore);
    }

    if (eTaskGetState(modbus_poller_task_handler) == eSuspended) {
      vTaskResume(modbus_poller_task_handler);
    }

  } else if (suffix == "set") {

    if (xTimerStop(modbus_poller_timer, 10) == pdFAIL) {
      ESP_LOGW(TAG, "Unable to stop Modbus Poller timer");
    }

    if (eTaskGetState(modbus_main_task_handler) == eSuspended) {
      vTaskResume(modbus_main_task_handler);
    }

    if (xQueueSend(json_data_queue, payload, 0) != pdPASS) {
      ESP_LOGW(TAG, "Unable to send queue for Modbus Main Task");
    }

    if (eTaskGetState(modbus_poller_task_handler) == eSuspended) {
      if (xSemaphoreGive(modbus_main_semaphore) != pdTRUE) {
        ESP_LOGW(TAG, "Unable to give semaphore for Modbus Main Task");
      }
    }

  } else {
    ESP_LOGW(TAG, "Unknow MQTT topic received: %s", topic);
  }
}

void onMqttPublish(uint16_t packetId) {
  ESP_LOGD(TAG, "Publish acknowledged for packetId: %d", packetId);
}

/*
void runOtaUpdateTask(void * pvParameters) {
  UBaseType_t __attribute__((__unused__)) uxHighWaterMark;

  uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  ESP_LOGV(TAG, "Entering OTA task. Unused stack size: %d", uxHighWaterMark);

  for (;;) {
    uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGV(TAG, "Suspending OTA task. Unused stack size: %d", uxHighWaterMark);
    vTaskSuspend(NULL);  // Task is suspended by default

    ESP_LOGV(TAG, "Resuming OTA task...");
    ESP_LOGI(TAG, "Checking if new firmware is available");
    if (checkFirmwareUpdate(FIRMWARE_URL, FIRMWARE_VERSION)) {
      ESP_LOGI(TAG, "New firmware found");
      ESP_LOGV(TAG, "Suspending modbus poller");
      if (xTimerStop(modbus_poller_timer, 10) == pdFAIL) {
        ESP_LOGW(TAG, "Unable to stop Modbus Poller timer for OTA update");
      }
      vTaskSuspend(modbus_poller_task_handler);

      if (updateOTA(FIRMWARE_URL)) {
        // Update is done. Rebooting...
        uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        ESP_LOGV(TAG, "Rebooting. Unused stack size: %d", uxHighWaterMark);
        ESP_LOGI(TAG, "************************ REBOOT IN PROGRESS *************************");
        ESP.restart();
      } else {
        ESP_LOGV(TAG, "OTA update failed. Restarting Modbus Poller");
// TODO(gmasse): retry?
        vTaskResume(modbus_poller_task_handler);
        if (xTimerStart(modbus_poller_timer, 10) == pdFAIL) {
          ESP_LOGE(TAG, "Unable to restart Modbus Poller timer after OTA update failure.");
          uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
          ESP_LOGV(TAG, "Rebooting. Unused stack size: %d", uxHighWaterMark);
          ESP_LOGI(TAG, "************************ REBOOT IN PROGRESS *************************");
          ESP.restart();
        }
      }
    }
  }
}
*/

void runModbusPollerTask(void * pvParameters) {
#ifndef MODBUS_DISABLED

  uint16_t curr_crc16_v = 0x0000;

  UBaseType_t __attribute__((__unused__)) uxHighWaterMark;
  /* Inspect our own high water mark on entering the task. */
  uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  ESP_LOGV(TAG, "Entering Modbus Poller task. Unused stack size: %d", uxHighWaterMark);

  vTaskSuspend(modbus_poller_task_handler);

  for (;;) {
    char json_buffer[MB_BUFFER_SIZE] = {0};
    size_t buff_size = 0;

    uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

    if (xTimerStop(modbus_poller_timer, 0) == pdFAIL) {
      ESP_LOGW(TAG, "Unable to stop Modbus Poller timer");
    }

    if (xSemaphoreTake(modbus_poller_semaphore, 10000 / portTICK_PERIOD_MS) != pdTRUE) {
      ESP_LOGW(TAG, "Unable to take semaphore for Modbus Poller Task");
    }

    parseModbusToJson(json_doc_poll.as<JsonVariant>());

    if (json_doc_poll.isNull()) {
      ESP_LOGE(TAG, "Error: modbus device is not responding!");
      goto MB_POLLER_SKIP_MQTT;
    }

    buff_size = serializeJson(json_doc_poll, json_buffer);
    ESP_LOGD(TAG, "JSON serialized: %s", json_buffer);

    if ((curr_crc16_v = crc16_le(0, (uint8_t *) json_buffer, MB_BUFFER_SIZE)) == modbus_poller_last_crc16_v
          && modbus_poller_force_crc16_chk == false)
    {
      goto MB_POLLER_SKIP_MQTT;
    }

    modbus_poller_last_crc16_v = curr_crc16_v;
    modbus_poller_force_crc16_chk = false;

    if (mqtt_client.connected()) {
      String mqtt_topic = MQTT_TOPIC;
      mqtt_topic += "/" + String(HOSTNAME) + "/current";
      ESP_LOGI(TAG, "MQTT Publishing data to topic: %s", mqtt_topic.c_str());
      mqtt_client.publish(mqtt_topic.c_str(), 2, true, json_buffer, buff_size);
    }

MB_POLLER_SKIP_MQTT:
    if (eTaskGetState(modbus_main_task_handler) != eSuspended) {
      xSemaphoreGive(modbus_main_semaphore);
    } else {
      if (xTimerStart(modbus_poller_timer, 0) == pdFAIL) {
        ESP_LOGW(TAG, "Unable to start Modbus Poller timer");
      }
    }

    ESP_LOGV(TAG, "Suspending Modbus Poller task. Unused stack size: %d", uxHighWaterMark);
    vTaskSuspend(NULL);
  }
#endif  // MODBUS_DISABLED
}

void runModbusPollerTimer() {
  if (eTaskGetState(modbus_main_task_handler) == eSuspended) {
    if (xSemaphoreGive(modbus_poller_semaphore) != pdTRUE) {
      ESP_LOGW(TAG, "Unable to give semaphore for Modbus Poller Task");
    }
  }

  if (eTaskGetState(modbus_poller_task_handler) == eSuspended) {
    ESP_LOGV(TAG, "Time to resume Modbus Poller");
    vTaskResume(modbus_poller_task_handler);
  }
}

void runModbusMainTask(void *pvParameters) {
#ifndef MODBUS_DISABLED

  UBaseType_t __attribute__((__unused__)) uxHighWaterMark;
  /* Inspect our own high water mark on entering the task. */
  uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  ESP_LOGV(TAG, "Entering Modbus Main task. Unused stack size: %d", uxHighWaterMark);

  vTaskSuspend(modbus_main_task_handler);

  for (;;) {

    uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

    // максимальное ожидание команды алисы - 2.5 сек.
    if (xQueueReceive(json_data_queue, mb_main_json_buffer, 2500u / portTICK_PERIOD_MS) == pdFALSE) {
      ESP_LOGD(TAG, "End of Queue is reched in Modbus Main Task");

      if (eTaskGetState(modbus_poller_task_handler) != eSuspended) {
        xSemaphoreGive(modbus_poller_semaphore); // поллер занят и наверняка ждет, когда мы ему позволим работать
      } else {
        // иначе запускаем таймер поллера в работу
        if (xTimerStart(modbus_poller_timer, 10) == pdFAIL) {
          ESP_LOGW(TAG, "Unable to start Modbus Poller timer");
        }
      }

      ESP_LOGV(TAG, "Suspending Modbus Main task. Unused stack size: %d", uxHighWaterMark);
      vTaskSuspend(NULL);

      continue;
    } else {
      ESP_LOGD(TAG, "MAIN JSON BUFF: %s", mb_main_json_buffer);
    }

    if (xSemaphoreTake(modbus_main_semaphore, 10000 / portTICK_PERIOD_MS) != pdTRUE) {
      ESP_LOGW(TAG, "Unable to take semaphore for Modbus Main Task");
    }

    deserializeJson(json_doc_main, mb_main_json_buffer);
    parseJsonToModbus_single(json_doc_main.as<JsonVariant>());

    // получаем информацию о текущем состоянии целевого порта
    if (modbusGetState_target(json_doc_main.as<JsonVariant>())) {
      char buff_temp[MB_BUFFER_SIZE];
      size_t n = serializeJson(json_doc_main, buff_temp);

      if (mqtt_client.connected()) {
        String mqtt_topic = MQTT_TOPIC;
        mqtt_topic += "/" + String(HOSTNAME) + "/res";
        ESP_LOGI(TAG, "MQTT Publishing data to topic: %s", mqtt_topic.c_str());
        mqtt_client.publish(mqtt_topic.c_str(), 2, false, buff_temp, n);
      }
    } else {
      ESP_LOGE(TAG, "Error while getting target state!");
    }
  }
#endif  // MODBUS_DISABLED
}

void setup() {
  // debug comm
  Serial.begin(MONITOR_SPEED);
  while (!Serial) continue;
  Serial.setDebugOutput(true);
/*
// TODO(gmasse): fix esp_log_level_set
  esp_log_level_set("*", ESP_LOG_VERBOSE);
  esp_log_level_set("ESP_base", ESP_LOG_INFO);
  esp_log_level_set("Modbus_base", ESP_LOG_INFO);
  esp_log_level_set("Main", ESP_LOG_INFO);
  ESP_LOGE("Main", "Error");
  ESP_LOGW("Main", "Warning");
  ESP_LOGI("Main", "Info");
  ESP_LOGD("Main", "Debug");
  ESP_LOGV("Main", "Verbose");
  esp_log_level_set(TAG, ESP_LOG_INFO);
  ESP_LOGE(TAG, "Error");
  ESP_LOGW(TAG, "Warning");
  ESP_LOGI(TAG, "Info");
  ESP_LOGD(TAG, "Debug");
  ESP_LOGV(TAG, "Verbose");
*/

  snprintf(HOSTNAME, sizeof(HOSTNAME), "ESP-MM-%llX", ESP.getEfuseMac());  // setting hostname

  ESP_LOGI(TAG, "*********************************************************************");
  ESP_LOGI(TAG, "Firmware version %s (compiled at %s %s)", FIRMWARE_VERSION, __DATE__, __TIME__);
  ESP_LOGV(TAG, "Watchdog time-out: %ds", CONFIG_TASK_WDT_TIMEOUT_S);
  ESP_LOGI(TAG, "Hostname: %s", HOSTNAME);

  mqtt_reconnect_timer = xTimerCreate("mqtt_timer", pdMS_TO_TICKS(2000), pdFALSE,
    NULL, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifi_reconnect_timer = xTimerCreate("wifi_timer", pdMS_TO_TICKS(2000), pdFALSE,
    NULL, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(wiFiEvent);

  mqtt_client.onConnect(onMqttConnect);
  mqtt_client.onDisconnect(onMqttDisconnect);
  mqtt_client.onSubscribe(onMqttSubscribe);
  mqtt_client.onUnsubscribe(onMqttUnsubscribe);
  mqtt_client.onMessage(onMqttMessage);
  mqtt_client.onPublish(onMqttPublish);

/*
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  io_conf.pin_bit_mask = GPIO_PIN25_CONFIG_M;
  gpio_config(&io_conf);
*/
  IPAddress mqtt_ip;

  // внешний пин определяет режим подключения к MQTT серверу
/*  if (gpio_get_level(GPIO_NUM_25) == false) { // подключаемся к локальному брокеру
    
    mqtt_ip.fromString(MQTT_LOCAL_IP);
    mqtt_client.setServer(mqtt_ip, MQTT_LOCAL_PORT);
    mqtt_client.setCredentials(MQTT_LOCAL_USERNAME, MQTT_LOCAL_PASSWD);

  } else { // или к профилю по умолчанию
*/
    mqtt_ip.fromString(MQTT_HOST_IP);
    mqtt_client.setServer(mqtt_ip, MQTT_PORT);
    mqtt_client.setCredentials(MQTT_USERNAME, MQTT_PASSWD);
//    mqtt_client.setSecure(true);
//  }

  wifiManager.autoConnect();

#ifndef MODBUS_DISABLED
  modbus_poller_semaphore = xSemaphoreCreateBinary();
  modbus_main_semaphore = xSemaphoreCreateBinary();
  json_data_queue = xQueueCreate(5u, sizeof(mb_main_json_buffer));

  initModbus(MODBUS_UNIT);

  xTaskCreate(runModbusPollerTask, "modbus_poller", 5900u, NULL, 2, &modbus_poller_task_handler);
  configASSERT(modbus_poller_task_handler);

  xTaskCreate(runModbusMainTask, "modbus_main", 5900u, NULL, 2, &modbus_main_task_handler);
  configASSERT(modbus_main_task_handler);

  modbus_poller_timer = xTimerCreate("modbus_poller_timer", pdMS_TO_TICKS((MODBUS_SCANRATE)*5u), pdTRUE, NULL,
    reinterpret_cast<TimerCallbackFunction_t>(runModbusPollerTimer));
#endif  // MODBUS_DISABLED

  //xTaskCreate(runOtaUpdateTask, "ota_update", 4500u, NULL, 2, &ota_update_task_handler);
  //configASSERT(ota_update_task_handler);
}

void loop() {
  vTaskDelete(NULL);
}
