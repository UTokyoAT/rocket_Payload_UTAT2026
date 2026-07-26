#include <Arduino.h>
#include "shared.h"
#include "tasks/task_sensor.h"
#include "tasks/task_gps.h"
#include "tasks/task_navigation.h"
#include "tasks/task_spi_link.h"

static Shared shared;

void setup() {
    Serial.begin(115200);
    Serial.println("XIAO1 (sensor/GPS/navigation/SPI master) booting...");

    shared.mutex = xSemaphoreCreateMutex();

    // Core 0
    xTaskCreatePinnedToCore(taskGPS, "taskGPS", 4096, &shared, 3, nullptr, 0);

    // Core 1（リアルタイム系。優先度はセンサー→誘導PID→SPI送信の依存順）
    xTaskCreatePinnedToCore(taskSensor,     "taskSensor",     4096, &shared, 5, nullptr, 1);
    xTaskCreatePinnedToCore(taskNavigation, "taskNavigation", 4096, &shared, 4, nullptr, 1);
    xTaskCreatePinnedToCore(taskSpiLink,    "taskSpiLink",    4096, &shared, 3, nullptr, 1);

    // TODO: taskStateMachine（ミッションステート遷移）は未実装。
    //       現状 shared.state は STANDBY のまま固定される。
    // WiFi/地上局とのテレメトリ・手動操作コマンドはXIAO2側（Radio）が担当する。
}

void loop() {
    vTaskDelete(nullptr);  // すべてFreeRTOSタスクに委ねるため、Arduinoのloopタスクは不要
}
