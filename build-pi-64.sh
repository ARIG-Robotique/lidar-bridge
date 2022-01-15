#!/usr/bin/env bash
# Author : Gregory DEPUILLE
# Description : Wrapper pour la configuration et le build du projet
PROJECT="ARIG C++ RPLidar Bridge (cross compile Raspberry PI 64bits)"

echo "Build du projet $PROJECT"

ROOT_DIR=$(pwd)
BUILD_NAME=build-pi-64
BUILD_DIR=${ROOT_DIR}/${BUILD_NAME}

cd ${ROOT_DIR}
if [ -d "${BUILD_DIR}" ] ; then
    echo "-- Nettoyage du répertoire de build ${BUILD_DIR}"
    rm -Rf ${BUILD_DIR}
fi

echo "-- Création du répertoire de build ${BUILD_DIR}"
mkdir -p ${BUILD_DIR}

sh download.sh ${BUILD_NAME}

echo "-- Build du projet ${PROJECT}"
cd ${BUILD_DIR}
cmake -DBUILD_PI_64=true .. || exit $?
cmake --build . || exit $?
echo "Build terminé"
