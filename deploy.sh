#!/bin/sh

ROBOT_NAME=${1}
INSTALL_DIR=/home/pi

if [ "${ROBOT_NAME}" != "nerell" ] && [ "${ROBOT_NAME}" != "odin" ] ; then
  echo "[ ERROR ] Robot ${ROBOT_NAME} inconnu."
  exit 1
fi

echo "Compilation ..."
./build.sh "${ROBOT_NAME}"

echo "Déploiement Applicatif ..."
scp "./build-${ROBOT_NAME}/bin/lidar_bridge" "${ROBOT_NAME}:$INSTALL_DIR/"
