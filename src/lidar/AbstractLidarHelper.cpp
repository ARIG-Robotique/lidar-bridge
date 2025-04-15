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
  // Transposition des angles dans le repère robot
  if (angleDeg < 180) {
    angleDeg *= -1;
  } else {
    angleDeg = 360 - angleDeg;
  }
  return angleDeg;
}
