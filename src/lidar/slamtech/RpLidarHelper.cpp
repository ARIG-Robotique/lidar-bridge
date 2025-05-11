//
// Created by gregorydepuille@sglk.local on 27/02/17.
//

#include <iostream>
#include <sstream>
#include <thread>
#include "RpLidarHelper.h"

void RpLidarHelper::init() {
    // create the driver instance
#ifdef DEBUG_MODE
    cout << "RpLidarHelper::init()" << endl;
#endif

    if (!this->driver) {
        this->driver = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
        if (!this->driver) {
            cerr << "Insufficent memory, exit" << endl;
            exit(2);
        }
    }

    // try to connect
    if (IS_FAIL(this->driver->connect(this->comFile.c_str(), this->baudrate))) {
        cerr << "Error, cannot bind to the specified serial port " << this->comFile << endl;
        exit(3);
    }

    // NON SUPPORTER PAR LES VIEUX FIRMWARE
    // get scan modes
    /*
    if (IS_FAIL(this->driver->getAllSupportedScanModes(this->scanModes))) {
        cerr << "Error, cannot get all supported scan modes";
        exit(4);
    }
    */
}

void RpLidarHelper::reconnectLidarIfNeeded() {
    JsonResult r = getDeviceInfo();
    if (r.status == RESPONSE_ERROR) {
        cerr << "Connection RPLidar perdu, reconnection" << endl;
        if (this->driver) {
            RPlidarDriver::DisposeDriver(this->driver);
            this->driver = nullptr;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        init();
    }
}

void RpLidarHelper::end() {
#ifdef DEBUG_MODE
    cout << "RpLidarHelper::end()" << endl;
#endif
    this->stopScan();
    this->driver->disconnect();

    RPlidarDriver::DisposeDriver(this->driver);
    this->driver = nullptr;
}

JsonResult RpLidarHelper::getDeviceInfo() {
    rplidar_response_device_info_t deviceInfo;
    if (IS_FAIL(this->driver->getDeviceInfo(deviceInfo))) {
        JsonResult fail;
        fail.status = RESPONSE_ERROR;
        fail.action = DEVICE_INFO;
        fail.errorMessage = "Impossible de récupérer les infos";
        return fail;
    }

    // Récupération du numéro de série
    ostringstream serialStream;
    for (unsigned char v : deviceInfo.serialnum) {
        char buff[3];
        snprintf(buff, sizeof(buff), "%02X", v);
        string tmp = buff;
        serialStream << tmp;
    }

    // Récupération du firmware
    ostringstream firmStream;
    firmStream << (deviceInfo.firmware_version >> 8) << "." << (deviceInfo.firmware_version & 0xFF);
    if ((deviceInfo.firmware_version & 0xFF) <= 10) {
        firmStream << "0";
    }

    json data;
    data["driver"] = "rplidar";
    data["serialNumber"] = serialStream.str();
    data["hardwareVersion"] = deviceInfo.hardware_version;
    data["firmwareVersion"] = firmStream.str();
    //data["scanModes"] = this->scanModes;

    JsonResult r;
    r.status = RESPONSE_OK;
    r.action = DEVICE_INFO;
    r.data = data;
    return r;
}

JsonResult RpLidarHelper::getHealth() {
    this->reconnectLidarIfNeeded();

    rplidar_response_device_health_t healthinfo;
    if (IS_FAIL(this->driver->getHealth(healthinfo))) {
        JsonResult fail;
        fail.status = RESPONSE_ERROR;
        fail.action = HEALTH_INFO;
        fail.errorMessage = "Impossible de récupérer les infos de santé";
        return fail;
    }

    string state;
    if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
        state = "ERROR";
    } else if (healthinfo.status == RPLIDAR_STATUS_WARNING) {
        state = "WARNING";
    } else {
        state = "OK";
    }

    json data;
    data["value"] = healthinfo.status;
    data["state"] = state;
    data["errorCode"] = (unsigned int) healthinfo.error_code;

    JsonResult r;
    r.status = RESPONSE_OK;
    r.action = DEVICE_INFO;
    r.data = data;
    return r;
}

JsonResult RpLidarHelper::startScan(JsonQuery q) {
    this->reconnectLidarIfNeeded();

    if (IS_FAIL(this->driver->startMotor())) {
        JsonResult r;
        r.action = q.action;
        r.status = RESPONSE_ERROR;
        r.errorMessage = "Impossible de démarrer le moteur";
        return r;
    };
    if (IS_FAIL(this->driver->startScan(false, true))) {
        JsonResult r;
        r.action = q.action;
        r.status = RESPONSE_ERROR;
        r.errorMessage = "Impossible de démarrer le scan";
        return r;
    }
    return this->setMotorSpeed(q);
}

JsonResult RpLidarHelper::stopScan() {
    JsonResult r;
    r.action = STOP_SCAN;
    r.status = RESPONSE_OK;

    if (IS_FAIL(this->driver->stop())) {
        r.status = RESPONSE_ERROR;
        r.errorMessage = "Impossible d'arreter le scan";
    }
    if (IS_FAIL(this->driver->stopMotor())) {
        r.status = RESPONSE_ERROR;
        r.errorMessage = "Impossible d'arreter le moteur";
    }

    return r;
}

JsonResult RpLidarHelper::setMotorSpeed(JsonQuery q) {
    JsonResult r;
    r.action = q.action;
    r.data = q.data;
    u_result result = RESULT_OK;
    if (!q.data["speed"].is_null()) {
        result = this->setMotorSpeed(q.data["speed"]);
    }
    if (IS_FAIL(result)) {
        r.status = RESPONSE_ERROR;
    } else {
        r.status = RESPONSE_OK;
    }
    return r;
}

u_result RpLidarHelper::setMotorSpeed(_u16 speed) {
    if (speed > MAX_MOTOR_PWM) {
        speed = MAX_MOTOR_PWM;
    } else if (speed < 0) {
        speed = 0;
    }
    this->reconnectLidarIfNeeded();
    return this->driver->setMotorPWM(speed);
}

JsonResult RpLidarHelper::grabScanData() {
    JsonResult r;
    r.action = GRAB_DATA;

    this->reconnectLidarIfNeeded();
    rplidar_response_measurement_node_hq_t nodes[8192];
    size_t nodeCount = sizeof(nodes)/sizeof(rplidar_response_measurement_node_hq_t);

    u_result resScan = this->driver->grabScanDataHq(nodes, nodeCount);
    u_result resAscend = this->driver->ascendScanData(nodes, nodeCount);
    if (IS_OK(resScan) && IS_OK(resAscend)) {
        r.status = RESPONSE_OK;

        int ignored = 0;
        json scanData = json::array();
        for (auto node : nodes) {
            float distanceMm = node.dist_mm_q2 / 4.0f;
            if ((distanceMm < this->excludeLowerThanMm) || (distanceMm > this->excludeGreaterThanMm)) {
                ignored++;
                continue;
            }

            float angleDeg = this->adjustAngle((node.angle_z_q14 * 90.0f) / 16384.0f);

            json v;
            v["angleDeg"] = angleDeg;
            v["distanceMm"] = distanceMm;
            v["syncBit"] = node.flag;
            v["quality"] = node.quality;
            scanData.push_back(v);
        }

        r.data["ignored"] = ignored;
        r.data["scan"] = scanData;
    } else {
        r.status = RESPONSE_ERROR;
        r.errorMessage = "Erreur lors de la récupération des données du SCAN";
    }

    return r;
}
