#!/usr/bin/env bash
# Author : Gregory DEPUILLE
# Description : Wrapper pour la configuration et le build du projet
PROJECT="ARIG C++ Lidar Bridge"

echo "Build du projet ${PROJECT}"

ROOT_DIR=$(pwd)
if [ "${#}" -eq 1 ] ; then
  BUILD_NAME="${1}"
else
  echo "Le nombre d'argument doit etre definie (local ou raspi)"
  exit 1
fi
if [ "${BUILD_NAME}" != "local" ] && [ "${BUILD_NAME}" != "raspi" ] ; then
  echo "Le build name doit être local ou raspi"
  exit 2
fi
BUILD_DIR=${ROOT_DIR}/build-${BUILD_NAME}

cd "${ROOT_DIR}" || exit 1
if [ -d "${BUILD_DIR}" ] ; then
    echo "-- Nettoyage du répertoire de build ${BUILD_DIR}"
    rm -Rf "${BUILD_DIR}"
fi

echo "-- Création du répertoire de build ${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"
echo "-- Build directory : ${BUILD_DIR}"
sh download.sh

echo "-- Build du projet ${PROJECT}"
cd "${BUILD_DIR}" || exit 1
cmake .. -DBUILD_NAME="${BUILD_NAME}" || exit ${?}
cmake --build . || exit $?
echo "Build terminé"
