//
// Created by Gregory DEPUILLE on 21/02/2025.
//

#include "Ld19LidarHelper.h"

void Ld19LidarHelper::init() {
  // Create driver instance
  this->driver = new ldlidar::LiPkg();

  this->cmd_port = new ldlidar::CmdInterfaceLinux();
  cmd_port->SetReadCallback(std::bind(&ldlidar::LiPkg::CommReadCallback, this->driver, std::placeholders::_1, std::placeholders::_2));
  if (cmd_port->Open(this->comFile)) {
    std::cout << "[LDRobot] Open LDLiDAR device " << this->comFile << " success!" << std::endl;
  } else {
    std::cerr << "[LDRobot] Open LDLiDAR device " << this->comFile << " fail!" << std::endl;
    exit(3);
  }
}

void Ld19LidarHelper::end() {
  this->cmd_port->Close();
  delete this->cmd_port;
  this->cmd_port = nullptr;

  delete this->driver;
  this->driver = nullptr;

  this->last_scan.clear();
}

JsonResult Ld19LidarHelper::getDeviceInfo() {
  json data;
  data["driver"] = "ldlidar";
  data["firmwareVersion"] = this->driver->GetSdkVersionNumber();
  data["speedHz"] = this->driver->GetSpeed();

  JsonResult r;
  r.status = RESPONSE_OK;
  r.action = DEVICE_INFO;
  r.data = data;
  return r;
}

JsonResult Ld19LidarHelper::getHealth() {
  json data;
  data["value"] = 0;
  data["state"] = "OK";
  data["errorCode"] = 0;

  JsonResult r;
  r.status = RESPONSE_OK;
  r.action = DEVICE_INFO;
  r.data = data;
  return r;
}

JsonResult Ld19LidarHelper::grabScanData() {
  JsonResult r;
  r.action = GRAB_DATA;

  if (this->driver->IsFrameReady()) {
    this->driver->ResetFrameReady();
    ldlidar::Points2D laser_scan = this->driver->GetLaserScanData();

    this->last_scan_read_time = std::chrono::steady_clock::now();
    this->last_ignored = 0;
    this->last_scan.clear();

    for (auto point : laser_scan) {
      float distanceMm = point.distance;
      if ((distanceMm < this->excludeLowerThanMm) || (distanceMm > this->excludeGreaterThanMm)) {
        this->last_ignored++;
        continue;
      }

      float angleDeg = this->adjustAngle(point.angle);

      json scanPoint;
      scanPoint["angle"] = angleDeg;
      scanPoint["distance"] = distanceMm;
      scanPoint["intensity"] = point.intensity;
      this->last_scan.push_back(scanPoint);
    }
  }

  if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - this->last_scan_read_time).count() > 150) {
    this->last_scan.clear();

    r.status = RESPONSE_ERROR;
    r.errorMessage = "Erreur de timeout sur la récupération des données du SCAN";

  } else if (!this->last_scan.empty()) {
    r.status = RESPONSE_OK;
    r.data["ignored"] = this->last_ignored;
    r.data["scan"] = this->last_scan;

  } else {
    r.status = RESPONSE_ERROR;
    r.errorMessage = "Erreur lors de la récupération des données du SCAN";
  }

  return r;
}
