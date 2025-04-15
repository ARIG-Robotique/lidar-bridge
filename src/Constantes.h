//
// Created by gregorydepuille@sglk.local on 28/02/17.
//

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifndef RPLIDAR_BRIDGE_CONSTANTES_H
#define RPLIDAR_BRIDGE_CONSTANTES_H

#include <json.hpp>

using json = nlohmann::json;

#define DEVICE_INFO  "DEVICE_INFO"
#define HEALTH_INFO  "HEALTH_INFO"
#define START_SCAN   "START_SCAN"
#define STOP_SCAN    "STOP_SCAN"
#define SET_SPEED    "SET_SPEED"
#define SET_CONFIG   "SET_CONFIG"
#define GRAB_DATA    "GRAB_DATA"

#define EXIT            "EXIT"
#define DATA_INVALID    "DATA_INVALID"
#define DATA_UNPARSABLE "DATA_UNPARSABLE"

#define RESPONSE_OK                 "OK"
#define RESPONSE_ERROR              "ERROR"
#define RESPONSE_NOT_IMPLEMENTED    "NOT_IMPLEMENTED"

struct JsonResult {
    std::string status;
    std::string action;
    std::string errorMessage;
    json data;
};

struct JsonQuery {
    std::string action;
    json data;
};

#endif //RPLIDAR_BRIDGE_CONSTANTES_H
