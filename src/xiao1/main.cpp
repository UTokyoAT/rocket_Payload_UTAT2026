#include <Arduino.h>
#include "shared.h"
#include "tasks/task_sensor.h"
#include "tasks/task_gps.h"
#include "tasks/task_wifi.h"
#include "tasks/task_spi_link.h"

static Shared shared;

void setup() {
    Serial.begin(115200);
    Serial.println("XIAO1 (sensor/GPS/WiFi) booting...");

    shared.mutex = xSemaphoreCreateMutex();

    // Core 0（WiFi/BT系）
    xTaskCreatePinnedToCore(taskGPS,  "taskGPS",  4096, &shared, 3, nullptr, 0);
    xTaskCreatePinnedToCore(taskWifi, "taskWifi", 4096, &shared, 2, nullptr, 0);

    // Core 1（リアルタイム系）
    xTaskCreatePinnedToCore(taskSensor,  "taskSensor",  4096, &shared, 5, nullptr, 1);
    xTaskCreatePinnedToCore(taskSpiLink, "taskSpiLink", 4096, &shared, 4, nullptr, 1);

    // TODO: taskStateMachine（ミッションステート遷移）は未実装。
    //       現状 shared.state は STANDBY のまま固定される。
}

void loop() {
    vTaskDelete(nullptr);  // すべてFreeRTOSタスクに委ねるため、Arduinoのloopタスクは不要
}
