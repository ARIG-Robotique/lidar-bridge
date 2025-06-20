//
// Created by Gregory DEPUILLE on 10/02/2025.
//


#ifndef ABSTRACTLIDARHELPER_H
#define ABSTRACTLIDARHELPER_H

#include "../Constantes.h"

using namespace std;

class AbstractLidarHelper {

public:

  virtual ~AbstractLidarHelper() = default;
  explicit AbstractLidarHelper(string comFile) {
    this->comFile = std::move(comFile);
    this->reversed = false;
    this->angleOffset = 0.0;
    this->excludeLowerThanMm = 150;
    this->excludeGreaterThanMm = 3600;
  }

  bool fileExists(); 

  virtual bool Autoconnect();
  virtual bool connectIfNeeded() = 0;
  virtual void disconnect() = 0;
  virtual bool isConnected() = 0;

  JsonResult getDisconnectedResponse();

  virtual JsonResult getDeviceInfo();
  virtual JsonResult getHealth();

  virtual JsonResult startScan(JsonQuery q);
  virtual JsonResult stopScan();
  virtual JsonResult setMotorSpeed(JsonQuery q);

  virtual JsonResult grabScanData();

  JsonResult setConfig(JsonQuery q);

protected:
  string comFile;

  bool reversed;
  float angleOffset;
  int16_t excludeLowerThanMm;
  int16_t excludeGreaterThanMm;

  JsonResult notImplemented(string action);
  float adjustAngle(float angle);
};

#endif //ABSTRACTLIDARHELPER_H
