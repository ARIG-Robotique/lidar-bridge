//
// Created by Gregory DEPUILLE on 21/02/2025.
//

#ifndef LIDAR_BRIDGE_LD19LIDARHELPER_H
#define LIDAR_BRIDGE_LD19LIDARHELPER_H
#include <lipkg.h>
#include "../AbstractLidarHelper.h"


class Ld19LidarHelper final : public AbstractLidarHelper {

public:
    explicit Ld19LidarHelper(const string &comFile) : AbstractLidarHelper(comFile) {
        this->driver = nullptr;
        this->cmd_port = nullptr;

        this->last_ignored = 0;
        this->last_scan = json::array();
        this->last_scan_read_time = std::chrono::steady_clock::now();
    }

    void init() override;
    void end() override;

    JsonResult getDeviceInfo() override;
    JsonResult getHealth() override;

    JsonResult grabScanData() override;

private:
    ldlidar::LiPkg *driver;
    ldlidar::CmdInterfaceLinux *cmd_port;

    unsigned int last_ignored;
    json last_scan;
    chrono::time_point<chrono::steady_clock> last_scan_read_time;
};

#endif //LIDAR_BRIDGE_LD19LIDARHELPER_H
