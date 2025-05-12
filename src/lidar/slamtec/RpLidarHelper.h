//
// Created by gregorydepuille@sglk.local on 27/02/17.
//

#ifndef LIDAR_BRIDGE_RPLIDARHELPER_H
#define LIDAR_BRIDGE_RPLIDARHELPER_H

#include <rplidar.h>
#include "../AbstractLidarHelper.h"

using namespace rp::standalone::rplidar;

class RpLidarHelper final : public AbstractLidarHelper {

public:
    explicit RpLidarHelper(const string &comFile) : RpLidarHelper(comFile, 115200) { }

    RpLidarHelper(const string &comFile, unsigned int baudrate) : AbstractLidarHelper(comFile) {
        this->baudrate = baudrate;
        this->driver = nullptr;
    }

    void init() override;
    void end() override;

    JsonResult getDeviceInfo() override;
    JsonResult getHealth() override;

    JsonResult startScan(JsonQuery q) override;
    JsonResult stopScan() override;
    JsonResult setMotorSpeed(JsonQuery q) override;

    JsonResult grabScanData() override;

private:
    RPlidarDriver * driver;
    unsigned int baudrate;
    //vector<RplidarScanMode> scanModes;
    u_result setMotorSpeed(_u16 speed);
};

#endif //LIDAR_BRIDGE_RPLIDARHELPER_H
