#pragma once

#if SK_Network

#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <HTTPClient.h>

#include "logger.h"
#include "motor_task.h"
#include "task.h"


class NetworkTask : public Task<NetworkTask> {
    friend class Task<NetworkTask>; // Allow base Task to invoke protected run()

    public:
        NetworkTask(const uint8_t task_core, MotorTask& motor_task, Logger& logger);

    protected:
        void run();

    private:
        MotorTask& motor_task_;
        Logger& logger_;
        WiFiClient wifi_client_;
        PubSubClient mqtt_client_;
        int mqtt_last_connect_time_ = 0;

        void connectWifi();
        void connectMQTT();
        void mqttCallback(char *topic, byte *payload, unsigned int length);
};

#endif