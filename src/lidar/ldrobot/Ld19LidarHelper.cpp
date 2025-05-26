//
// Created by Gregory DEPUILLE on 21/02/2025.
//

#include "Ld19LidarHelper.h"

bool Ld19LidarHelper::connectIfNeeded() {
  // Create driver instance
  if (isConnected())
  {
    return true;
  }
  

  if (!driver)
  {
    this->driver = new ldlidar::LiPkg();
  }
  if (!cmd_port && driver)
  {
    this->cmd_port = new ldlidar::CmdInterfaceLinux();
    if (cmd_port)
    {
      cmd_port->SetReadCallback(std::bind(&ldlidar::LiPkg::CommReadCallback, this->driver, std::placeholders::_1, std::placeholders::_2));
      cout << "Read callback bound !" << endl;
    }
  }
  
  if (cmd_port && !cmd_port->IsOpened())
  {
    if (cmd_port->Open(this->comFile)) {
      std::cout << "[LDRobot] Open LDLiDAR device " << this->comFile << " success!" << std::endl;
      this->last_scan_read_time = std::chrono::steady_clock::now();
      return true;
    } else {
      std::cerr << "[LDRobot] Open LDLiDAR device " << this->comFile << " fail!" << std::endl;
      disconnect();
      return false;
    }
  }
  return true;
}

void Ld19LidarHelper::disconnect() {
  if (driver)
  {
    delete driver;
  }
  if (cmd_port)
  {
    this->cmd_port->Close();
    delete cmd_port;
  }
  this->cmd_port = nullptr;

  this->driver = nullptr;

  this->last_scan.clear();
}


bool Ld19LidarHelper::isConnected() 
{
  if (!cmd_port)
  {
    return false;
  }
  if (!driver)
  {
    return false;
  }
  
  return cmd_port->IsOpened();
}

JsonResult Ld19LidarHelper::getDeviceInfo() {
  if (!driver)
  {
    JsonResult r;
    r.status = RESPONSE_ERROR;
    r.action = DEVICE_INFO;
    return r;
  }
  
  JsonResult r;
  json &data = r.data;
  data["driver"] = "ldlidar";
  data["firmwareVersion"] = this->driver->GetSdkVersionNumber();
  data["speedHz"] = this->driver->GetSpeed();

  r.status = RESPONSE_OK;
  r.action = DEVICE_INFO;
  r.data = data;
  return r;
}

JsonResult Ld19LidarHelper::getHealth() {
  JsonResult r;
  json &data = r.data;
  data["value"] = 0;
  data["state"] = isConnected() ? "OK" : "KO";
  data["errorCode"] = 0;

  r.status = RESPONSE_OK;
  r.action = DEVICE_INFO;
  r.data = data;
  return r;
}

JsonResult Ld19LidarHelper::grabScanData() {
  connectIfNeeded();
  JsonResult r;
  r.action = GRAB_DATA;

  if (!isConnected())
  {
    r.status = RESPONSE_ERROR;
    r.errorMessage = "Pas connecté";
    return r;
  }
  

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
      scanPoint["angleDeg"] = angleDeg;
      scanPoint["distanceMm"] = distanceMm;
      scanPoint["intensity"] = point.intensity;
      this->last_scan.push_back(scanPoint);
    }
  }

  if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - this->last_scan_read_time).count() > 150) {
    this->last_scan.clear();

    r.status = RESPONSE_ERROR;
    r.errorMessage = "Erreur de timeout sur la récupération des données du SCAN";
    disconnect();
    

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
