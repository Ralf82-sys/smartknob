#pragma once

#if SK_Network

#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "configuration.h"

#include "logger.h"
#include "motor_task.h"
#include "task.h"


class NetworkTask : public Task<NetworkTask> {
    friend class Task<NetworkTask>; // Allow base Task to invoke protected run()

    public:
        NetworkTask(const uint8_t task_core, MotorTask& motor_task, Logger& logger, Configuration& configuration);
        void addListener(QueueHandle_t queue);

        QueueHandle_t getKnobStateQueue();

    protected:
        void run();

    private:
        MotorTask& motor_task_;
        Logger& logger_;
        WiFiClient wifi_client_;
        PubSubClient mqtt_client_;
        HTTPClient http_client_;
        int mqtt_last_connect_time_ = 0;
        Configuration& configuration_;
        QueueHandle_t queue_;
        QueueHandle_t knob_state_queue_; // Add this line

        PB_SmartKnobState state_;

        void connectWifi();
        //void connectMQTT();
        void connectHttp();
        //void mqttCallback(char *topic, byte *payload, unsigned int length);
};

#endif