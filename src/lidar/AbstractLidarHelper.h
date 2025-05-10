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
    this->angleOffset = 0;
    this->excludeLowerThanMm = 150;
    this->excludeGreaterThanMm = 3600;
  }

  virtual void init() = 0;
  virtual void end() = 0;

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
  int8_t angleOffset;
  int16_t excludeLowerThanMm;
  int16_t excludeGreaterThanMm;

  JsonResult notImplemented(string action);
  float adjustAngle(float angle);
};

#endif //ABSTRACTLIDARHELPER_H
