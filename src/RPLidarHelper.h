//
// Created by gregorydepuille@sglk.local on 27/02/17.
//

#ifndef RPLIDAR_BRIDGE_RPLIDARHELPER_H
#define RPLIDAR_BRIDGE_RPLIDARHELPER_H

#include <string>
#include <rplidar.h>
#include "Constantes.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif


using namespace std;
using namespace rp::standalone::rplidar;

class RPLidarHelper {

public:
    RPLidarHelper(string comFile = "/dev/ttyUSB0", unsigned int baudrate = 115200);

    void init();

    JsonResult getDeviceInfo();
    JsonResult getHealth();

    JsonResult startScan(JsonQuery q);
    JsonResult setMotorSpeed(JsonQuery q);

    JsonResult grabScanData();

    void end();

private:
    string comFile;
    unsigned int baudrate;

    RPlidarDriver * driver;

    void setMotorSpeed(_u16 speed);
};


#endif //RPLIDAR_BRIDGE_RPLIDARHELPER_H
