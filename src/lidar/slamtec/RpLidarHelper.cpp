//
// Created by gregorydepuille@sglk.local on 27/02/17.
//

#include <iostream>
#include <sstream>
#include <thread>
#include "RpLidarHelper.h"


bool RpLidarHelper::connectIfNeeded()
{
	//early exit if we're connected
	if (isConnected())
	{
		return true;
	}
	//create driver object
	if (driver == nullptr) {
		driver = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
		if (driver == nullptr) {
			cerr << "Insufficent memory, exit" << endl;
			return false;
		}
	}
	// try to connect
	if (IS_FAIL(driver->connect(this->comFile.c_str(), this->baudrate))) {
		cerr << "Error, cannot bind to the specified serial port " << this->comFile << endl;
		return false;
	}
	cout << "Lidar connected" << endl;
	return true;
}

void RpLidarHelper::disconnect() {
#ifdef DEBUG_MODE
	cout << "RpLidarHelper::disconnect()" << endl;
#endif
	if (scanStarted)
	{
		scanStarted = false;
		this->stopScan();
	}
	if (driver)
	{
		if (driver->isConnected())
		{
			this->driver->disconnect();

		}
		RPlidarDriver::DisposeDriver(this->driver);
	}
	this->driver = nullptr;
	this->scanStarted = false;
}

bool RpLidarHelper::isConnected() 
{
	if (driver == nullptr)
	{
		return false;
	}
	return driver->isConnected() && fileExists();
	
}

JsonResult RpLidarHelper::getDeviceInfo() {
	if (!Autoconnect())
	{
		auto fail = getDisconnectedResponse();
		fail.action = DEVICE_INFO;
		return fail;
	}
	
	
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

	if (!Autoconnect())
	{
		auto fail = getDisconnectedResponse();
		fail.action = HEALTH_INFO;
		return fail;
	}
	

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

	JsonResult r;
	json &data = r.data;
	data["value"] = healthinfo.status;
	data["state"] = state;
	data["errorCode"] = (unsigned int) healthinfo.error_code;

	r.status = RESPONSE_OK;
	r.action = DEVICE_INFO;
	r.data = data;
	return r;
}

JsonResult RpLidarHelper::startScan(JsonQuery q) {
	if (!Autoconnect())
	{
		auto fail = getDisconnectedResponse();
		fail.action = q.action;
		return fail;
	}

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
#ifdef DEBUG_MODE
	cout << "Démarrage du scan" <<endl;
#endif
	this->scanStarted = true;
	return this->setMotorSpeed(q);
}

JsonResult RpLidarHelper::stopScan() {
	if (!Autoconnect())
	{
		auto fail = getDisconnectedResponse();
		fail.action = STOP_SCAN;
		return fail;
	}
	JsonResult r;
	r.action = STOP_SCAN;
	r.status = RESPONSE_OK;

	if (this->driver) {
		if (IS_FAIL(this->driver->stop())) {
			r.status = RESPONSE_ERROR;
			r.errorMessage = "Impossible d'arreter le scan";
		}
		if (IS_FAIL(this->driver->stopMotor())) {
			r.status = RESPONSE_ERROR;
			r.errorMessage = "Impossible d'arreter le moteur";
		}
		this->scanStarted = false;
	} else {
		r.status = RESPONSE_ERROR;
		r.errorMessage = "Le driver n'est pas initialisé";
	}

	return r;
}

JsonResult RpLidarHelper::setMotorSpeed(JsonQuery q) {
	if (!Autoconnect())
	{
		auto fail = getDisconnectedResponse();
		fail.action = q.action;
		return fail;
	}
	JsonResult r;
	r.action = q.action;
	r.data = q.data;
	u_result result = RESULT_OK;
	if (!q.data["speed"].is_null()) {
		last_motor_speed = q.data["speed"];
		result = this->setMotorSpeed(last_motor_speed);
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
	if (!Autoconnect())
	{
		return RESULT_RECONNECTING;
	}
#ifdef DEBUG_MODE
	cout << "Démarrage du moteur, vitesse " << speed <<endl;
#endif
	return this->driver->setMotorPWM(speed);
}

JsonResult RpLidarHelper::grabScanData() {
	JsonResult r;
	r.action = GRAB_DATA;

	if (!Autoconnect())
	{
		auto fail = getDisconnectedResponse();
		fail.action = r.action;
		return fail;
	}
	if (!this->scanStarted) {
		JsonQuery q = JsonQuery();
		q.action = START_SCAN;
		q.data = json::object();
		q.data["speed"] = last_motor_speed;
		startScan(q);
	}

	rplidar_response_measurement_node_hq_t nodes[8192];
	size_t nodeCount = sizeof(nodes)/sizeof(rplidar_response_measurement_node_hq_t);

	u_result resScan = this->driver->grabScanDataHq(nodes, nodeCount);
	u_result resAscend = this->driver->ascendScanData(nodes, nodeCount);
	if (IS_OK(resScan) && IS_OK(resAscend)) {
		r.status = RESPONSE_OK;

		int ignored = 0;
		json scanData = json::array();
		for (size_t i = 0; i < nodeCount; i++)
		{
			auto &node = nodes[i];
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
		disconnect();
	}

	return r;
}
