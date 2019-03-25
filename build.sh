#!/usr/bin/env bash
# Author : Gregory DEPUILLE
# Description : Wrapper pour la configuration et le build du projet
PROJECT="ARIG C++ RPLidar Bridge"

ROOT_DIR=$(pwd)
BUILD_NAME=build
BUILD_DIR=${ROOT_DIR}/${BUILD_NAME}

echo "Build du projet $PROJECT"
if [ -d "${BUILD_DIR}" ] ; then
    echo "-- Nettoyage du répertoire de build ${BUILD_DIR}"
    rm -Rf ${BUILD_DIR}
fi

echo "-- Création du répertoire de build ${BUILD_DIR}"
mkdir -p ${BUILD_DIR}

echo "-- Build du projet $PROJECT"
cd ${BUILD_DIR}
cmake .. || exit $?
cmake --build . || exit $?
echo "Build terminé"
