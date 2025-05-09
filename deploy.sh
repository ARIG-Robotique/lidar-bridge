#!/bin/sh

ROBOT_NAME=${1}
INSTALL_DIR=/home/pi

if [ "${ROBOT_NAME}" != "nerell" ] && [ "${ROBOT_NAME}" != "odin" ] ; then
  echo "[ ERROR ] Robot ${ROBOT_NAME} inconnu."
  exit 1
fi

echo "Compilation ..."
./build.sh raspi

echo "Déploiement Applicatif ..."
scp "./build-raspi/bin/lidar_bridge" "${ROBOT_NAME}.local:$INSTALL_DIR/"
