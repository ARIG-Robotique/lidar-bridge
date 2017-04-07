/* RPLIDAR Bridge Application */
#include "SocketHelper.h"
#include "RPLidarHelper.h"

void printUsage();

int main(int argc, const char **argv) {

    if (argc != 3) {
        printUsage();
        return 1;
    }
    string socketType = argv[1];
    string socketConf = argv[2];

    // Initialisation de la socket
    SocketHelper socket(socketType);
    if (socket.isUnknown()) {
        printUsage();
        return 2;
    } else if (socket.isInet()) {
        socket.setPort(atoi(socketConf.c_str()));
    } else if (socket.isUnix()) {
        socket.setSocketFile(socketConf);
    }
    socket.init();

    // Initialisation RPLidar
    RPLidarHelper lidar;
    lidar.init();

    bool stop = false, waitConnection = true;
    while (!stop) {
        if (waitConnection) {
            cout << "Attente de connexion client ..." << endl;
            socket.waitConnection();
            waitConnection = false;
        }

        JsonQuery query = socket.getQuery();
        if (query.action == DATA_INVALID) {
            cout << "Données invalide, la socket client est fermé ?" << endl;
            waitConnection = true;
            continue;
        }

        JsonResult result;
        if (query.action == DEVICE_INFO) {
            result = lidar.getDeviceInfo();

        } else if (query.action == HEALTH_INFO) {
            result = lidar.getHealth();

        } else if (query.action == START_SCAN) {
            result = lidar.startScan(query);

        } else if (query.action == STOP_SCAN) {
            result = lidar.stopScan();

        } else if (query.action == SET_SPEED) {
            result = lidar.setMotorSpeed(query);

        } else if (query.action == GRAB_DATA) {
            result = lidar.grabScanData();

        } else if (query.action == EXIT) {
            cout << "Demande d'arret du programe" << endl;
            result.status = RESPONSE_OK;
            stop = true;

        } else {
            result.status = RESPONSE_ERROR;
            result.errorMessage = "Action non supporté";
        }

        result.action = query.action;
        socket.sendResponse(result);
    }

    // Fermeture de la socket
    socket.end();

    // Arret du lidar
    lidar.end();

    return 0;
}

void printUsage() {
    cerr << "Usage socket unix : rplidar_bridge unix /tmp/socket.sock" << endl;
    cerr << "Usage socket inet : rplidar_bridge inet 8686" << endl;
}