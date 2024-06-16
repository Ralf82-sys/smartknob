#if SK_Network
#include "network_task.h"

#include "motor_task.h"
#include "secrets.h"
//#include <ArduinoHttpClient.h>
#include <HTTPClient.h>



NetworkTask::NetworkTask(const uint8_t task_core, MotorTask& motor_task, Logger& logger, Configuration& configuration) :
    Task("Network", 4096, 1), motor_task_(motor_task), logger_(logger), configuration_(configuration) {
    knob_state_queue_ = xQueueCreate(1, sizeof(PB_SmartKnobState));
    assert(knob_state_queue_ != NULL);
    }
    //mqtt_client_(wifi_client_) {
    //auto callback = [this](char *topic, byte *payload, unsigned int length) { mqttCallback(topic, payload, length); };
    //mqtt_client_.setCallback(callback);
//}

void NetworkTask::connectWifi() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        logger_.log("Establishing connection to WiFi..");
    }

    char buf[256];
    snprintf(buf, sizeof(buf), "Connected to network %s", WIFI_SSID);
    logger_.log(buf);
}

//void NetworkTask::mqttCallback(char *topic, byte *payload, unsigned int length) {
//    char buf[256];
//    snprintf(buf, sizeof(buf), "Received mqtt callback for topic %s, length %u", topic, length);
//    logger_.log(buf);
//}

//void NetworkTask::connectMQTT() {
//    char buf[256];
//    mqtt_client_.setServer(MQTT_SERVER, 1883);
//    logger_.log("Attempting MQTT connection...");
//    if (mqtt_client_.connect(HOSTNAME "-" MQTT_USER, MQTT_USER, MQTT_PASSWORD)) {
//        logger_.log("MQTT connected");
//        mqtt_client_.subscribe(MQTT_COMMAND_TOPIC);
//    } else {
//        snprintf(buf, sizeof(buf), "MQTT failed rc=%d will try again in 5 seconds", mqtt_client_.state());
//        logger_.log(buf);
//    }
//}

void NetworkTask::connectHttp() {
    char buf[256];

    std::string serverName;
    serverName.append(Http_Server_Adress);
    serverName.append(":");
    serverName.append(Http_Server_Port);
    serverName.append(Http_Volume_Command);

    logger_.log(serverName.c_str());

    http_client_.begin(serverName.c_str());
    delay(1000);

    int httpResponseCode = http_client_.GET();

    std::string s = std::to_string(httpResponseCode);
    const char * status = s.c_str();
    logger_.log("Status: ");
    logger_.log(status);

    if (httpResponseCode>0) {
        String payload = http_client_.getString();
        const char * response = payload.c_str();
        logger_.log("Response: ");
        logger_.log(response);
    }

}

void NetworkTask::run() {
    connectWifi();
    //connectMQTT();
    connectHttp();

    //PB_PersistentConfiguration c = configuration_.get();
    //PB_SmartKnobConfig config = {
    //    .position = 0,
    //    .sub_position_unit = 0,
    //    .position_nonce = 0,
    //    .min_position = 0,
    //    .max_position = 1,
    //    .position_width_radians = 60 * _PI / 180,
    //    .detent_strength_unit = 0,
    //};


    //Command command;

    while(1) {

        //if (xQueueReceive(queue_, &command, 0) == pdTRUE) {
        //    logger_.log("Command received: ");
        //}

        long now = millis();
        //if (!mqtt_client_.connected() && (now - mqtt_last_connect_time_) > 5000) {
        //    logger_.log("Reconnecting MQTT");
        //    mqtt_last_connect_time_ = now;
        //    connectMQTT();
        //}
        //mqtt_client_.loop();

        std::string serverName;
        serverName.append(Http_Server_Adress);
        serverName.append(":");
        serverName.append(Http_Server_Port);
        serverName.append(Http_Volume_Command);

        http_client_.begin(serverName.c_str());
        delay(1000);
        int httpResponseCode = http_client_.GET();
        std::string s = std::to_string(httpResponseCode);
        const char * status = s.c_str();
        logger_.log("Status: ");
        logger_.log(status);

        if (httpResponseCode>0) {
            String payload = http_client_.getString();
            const char * response = payload.c_str();
            logger_.log("Response: ");
            logger_.log(response);
        }


        delay(5000);
    }
}
#endif