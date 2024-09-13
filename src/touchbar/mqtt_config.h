// config.h
#ifndef CONFIG_H
#define CONFIG_H

#define WIFI_SSID             "wifi-network-here"
#define WIFI_PASSWORD         "wifi-pw-here"
#define MQTT_SERVER           "mqtt-host"
#define MQTT_PORT             1883
#define MQTT_USER             "user"
#define MQTT_PASSWORD         "pass"

#define WIFI_CONNECT_WAIT_MAX (30 * 1000)

// mqtt topics to use
const char* TOPIC_BEDROOM_LIGHT = "home/zha/control/bedroom_night_lamp";
const char* TOPIC_LIVINGROOM_LIGHT = "home/zha/control/livingroom_light";
const char* TOPIC_COUCH_LIGHT = "home/zha/control/couch_light";
const char* TOPIC_COUCH_LED = "home/zha/control/couch_led";
const char* TOPIC_VOLUME = "home/zha/control/volume";

#endif // CONFIG_H