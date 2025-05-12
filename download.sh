#!/usr/bin/env bash

ROOT_DIR=$(pwd)
DOWNLOAD_DIR=${ROOT_DIR}/downloads

RPLIDAR_SDK_VERSION=v1.12.0
RPLIDAR_SDK_DIR=rplidar_sdk
RPLIDAR_FILENAME=${RPLIDAR_SDK_DIR}_${RPLIDAR_SDK_VERSION}
RPLIDAR_DOWNLOAD_URL=https://github.com/Slamtec/rplidar_sdk/archive/refs/tags/release/${RPLIDAR_SDK_VERSION}.zip

LD19LIDAR_SDK_VERSION=v2.3.1
LD19LIDAR_SDK_DIR=ldlidar_sdk
LD19LIDAR_FILENAME=${LD19LIDAR_SDK_DIR}_${LD19LIDAR_SDK_VERSION}
LD19LIDAR_DOWNLOAD_URL=https://raw.githubusercontent.com/May-DFRobot/DFRobot/refs/heads/master/LD19_SDK.zip

JSON_VERSION=v3.7.3
JSON_DIR=json-${JSON_VERSION}
JSON_FILENAME=json.hpp
JSON_DOWNLOAD_URL=https://github.com/nlohmann/json/releases/download/${JSON_VERSION}/${JSON_FILENAME}

SPDLOG_VERSION=1.3.1
SPDLOG_FILENAME=spdlog-${SPDLOG_VERSION}
SPDLOG_DOWNLOAD_URL=https://github.com/gabime/spdlog/archive/v${SPDLOG_VERSION}.zip

echo "-- Download external dependencies"
if [ ! -d "${DOWNLOAD_DIR}" ] ; then
    echo "---- Make download directory : $DOWNLOAD_DIR"
    mkdir -p ${DOWNLOAD_DIR}
fi

cd ${DOWNLOAD_DIR}
if [ ! -f "${RPLIDAR_FILENAME}.zip" ] ; then
  echo "---- Download RPLidar SDK ${RPLIDAR_SDK_VERSION} ..."
  curl -L ${RPLIDAR_DOWNLOAD_URL} -o ${RPLIDAR_FILENAME}.zip
  unzip ${RPLIDAR_FILENAME}.zip
  #mv rplidar_sdk-master rplidar_sdk-release-${RPLIDAR_SDK_VERSION}
  #rm -f ${RPLIDAR_SDK_DIR}
  ln -s ${RPLIDAR_SDK_DIR}-release-${RPLIDAR_SDK_VERSION} ${RPLIDAR_SDK_DIR}

  cd ${DOWNLOAD_DIR}/${RPLIDAR_SDK_DIR}
  for f in $(ls ${ROOT_DIR}/patch/${RPLIDAR_SDK_DIR}-${RPLIDAR_SDK_VERSION}/*.patch); do
    echo "---- Apply patch $f"
    patch --batch -p0 < $f
  done
fi

cd ${DOWNLOAD_DIR}
if [ ! -f "${LD19LIDAR_FILENAME}.zip" ] ; then
  echo "---- Download LD19 Lidar SDK ${LD19LIDAR_SDK_VERSION} ..."
  curl -L ${LD19LIDAR_DOWNLOAD_URL} -o ${LD19LIDAR_FILENAME}.zip
  unzip ${LD19LIDAR_FILENAME}.zip
  mv LD19_SDK/Linux/ldlidar_stl_sdk.zip .
  rm -Rf LD19_SDK
  unzip ldlidar_stl_sdk.zip
  rm -f ldlidar_stl_sdk.zip
  mv ldlidar_stl_sdk ldlidar_sdk-release-${LD19LIDAR_SDK_VERSION}
  rm -f ${LD19LIDAR_SDK_DIR}
  ln -s ldlidar_sdk-release-${LD19LIDAR_SDK_VERSION} ${LD19LIDAR_SDK_DIR}
fi

cd ${DOWNLOAD_DIR}
if [ ! -f "${JSON_DIR}/${JSON_FILENAME}" ] ; then
    echo "---- Download JSON ${JSON_VERSION} ..."
    mkdir -p ${JSON_DIR} json
    curl -L ${JSON_DOWNLOAD_URL} -o ${JSON_DIR}/${JSON_FILENAME}
    cd json
    rm -f ${JSON_FILENAME}
    ln -s ../${JSON_DIR}/${JSON_FILENAME}
fi

cd ${DOWNLOAD_DIR}
if [ ! -f "${SPDLOG_FILENAME}.zip" ] ; then
  echo "---- Download SPDLOG ${SPDLOG_VERSION} ..."
  curl -L ${SPDLOG_DOWNLOAD_URL} -o ${SPDLOG_FILENAME}.zip
  unzip ${SPDLOG_FILENAME}.zip
  rm -f spdlog
  ln -s ${SPDLOG_FILENAME}/include spdlog
fi
