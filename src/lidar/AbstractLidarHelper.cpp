//
// Created by Gregory DEPUILLE on 10/02/2025.
//

#include "AbstractLidarHelper.h"
#include <string>

JsonResult AbstractLidarHelper::getDeviceInfo() {
  return notImplemented(DEVICE_INFO);
}
JsonResult AbstractLidarHelper::getHealth() {
  return notImplemented(HEALTH_INFO);
}

JsonResult AbstractLidarHelper::startScan(JsonQuery q) {
  return notImplemented(q.action);
}
JsonResult AbstractLidarHelper::stopScan(){
  return notImplemented(STOP_SCAN);
}
JsonResult AbstractLidarHelper::setMotorSpeed(JsonQuery q){
  return notImplemented(q.action);
}

JsonResult AbstractLidarHelper::grabScanData(){
  return notImplemented(GRAB_DATA);
}

JsonResult AbstractLidarHelper::notImplemented(string action){
	JsonResult notImplemented;
    notImplemented.status = RESPONSE_NOT_IMPLEMENTED;
    notImplemented.action = action;
    notImplemented.errorMessage = "Not yet implemented";
    return notImplemented;
}

float AbstractLidarHelper::adjustAngle(float angleDeg) {
  // Change angle with configured offset
  angleDeg += angleOffset;
  if (angleDeg > 360) {
    angleDeg -= 360;
  } else if (angleDeg < 0) {
    angleDeg += 360;
  }

  // Reverse the angle if needed
  if (reversed) {
    angleDeg = 360 - angleDeg;
  }

  // Transposition des angles dans le repère robot
  if (angleDeg < 180) {
    angleDeg *= -1;
  } else {
    angleDeg = 360 - angleDeg;
  }
  return angleDeg;
}

JsonResult AbstractLidarHelper::setConfig(JsonQuery q) {
  JsonResult r;
  r.status = RESPONSE_OK;
  r.action = q.action;
  r.data = q.data;

  if (q.data["reversed"].is_null() && !q.data["reversed"].is_boolean()) {
    r.status = RESPONSE_ERROR;
    r.errorMessage = "reversed is not a boolean";
    return r;
  }
  if (q.data["angleOffset"].is_null() && !q.data["angleOffset"].is_number()) {
    r.status = RESPONSE_ERROR;
    r.errorMessage = "angleOffset is not a number";
    return r;
  }

  this->reversed = q.data["reversed"];
  this->angleOffset = q.data["angleOffset"];

  return r;
}
