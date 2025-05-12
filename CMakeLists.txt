#=============================================================================#
# Author: Gregory DEPUILLE                                                    #
# Date:   18/02/2017                                                          #
# Description: Build du bridge par socket pour le RPLidar                     #
#=============================================================================#
# Contrôle de la version minimale de CMake
cmake_minimum_required(VERSION 3.11.0)

# Check program requis
function (checkProgram name)
    message("-- Detecting ${name} program")
    find_program(PROGRAM_PRESENT ${name})
    if (NOT PROGRAM_PRESENT)
        message(FATAL_ERROR "${name} must be installed")
    else()
        message("-- Detecting ${name} program - done")
    endif()
endfunction(checkProgram)

# Variable pour les dépendances externes
set(DOWNLOAD_DIR "downloads")

# Pour faire de la cross compilation
if (DEFINED BUILD_NAME)
    if (${BUILD_NAME} STREQUAL "raspi")
        if (UNIX AND NOT APPLE)
            set(ARM_CROSS_COMPILER_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../raspi-crosstools/build-tools/gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu)
            set(ARM_BIN_PREFIX bin/aarch64-linux-gnu-)
        elseif (UNIX AND APPLE)
            set(ARM_CROSS_COMPILER_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../raspi-crosstools/build-tools/gcc-arm-11.2-2022.02-darwin-x86_64-aarch64-none-elf)
            set(ARM_BIN_PREFIX bin/aarch64-none-elf-)
        else()
            message(FATAL_ERROR "Arch OS is not valid for cross compile")
        endif()

        set(ARM_SYSROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../raspi-crosstools/build-tools/sysroot-2021-05-07-raspios-buster-aarch64)
        set(CMAKE_TOOLCHAIN_FILE ./raspberry.cmake)
    endif()
endif()

message(STATUS "Build Name : ${BUILD_NAME}")

# Variable de build
set(CMAKE_CXX_STANDARD 11)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if (NOT DEFINED CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    message(STATUS "Debug mode ON")
    add_definitions(-DDEBUG_MODE)
endif()

# Détection des fichiers par OS
if (UNIX AND NOT APPLE)
    set(RPLIDAR_ARCH_DIR "linux")
elseif (UNIX AND APPLE)
    set(RPLIDAR_ARCH_DIR "macOS")
    add_definitions(-D_MACOS)
else()
    message(FATAL_ERROR "Arch OS is not valid")
endif()

# Check program
checkProgram(git)
checkProgram(curl)
checkProgram(unzip)

#====================================================================#
#  Setup Project                                                     #
#====================================================================#
project(lidar_bridge)

# Récupération des dépendances par le script de download
execute_process(
        COMMAND sh download.sh
        WORKING_DIRECTORY ..
)

# Compilation de la lib externe RPLidar #
#########################################
set(RPLIDAR_LIB_NAME   "_rplidar_sdk")
set(RPLIDAR_SDK_DIR    "${DOWNLOAD_DIR}/rplidar_sdk/sdk/sdk")

file(GLOB RPLIDAR_SRCS
        "${RPLIDAR_SDK_DIR}/src/*.cpp"
        "${RPLIDAR_SDK_DIR}/src/hal/*.cpp"
        "${RPLIDAR_SDK_DIR}/src/arch/${RPLIDAR_ARCH_DIR}/*.cpp"
)

include_directories(
        "${RPLIDAR_SDK_DIR}/include/"
        "${RPLIDAR_SDK_DIR}/src/"
)
add_library(${RPLIDAR_LIB_NAME} STATIC ${RPLIDAR_SRCS})

# Compilation de la lib externe LD19 Lidar #
############################################
set(LDLIDAR_LIB_NAME   "_ldlidar_sdk")
set(LDLIDAR_SDK_DIR    "${DOWNLOAD_DIR}/ldlidar_sdk/include/ldlidar_driver")

file(GLOB LDLIDAR_SRCS "${LDLIDAR_SDK_DIR}/*.cpp")
include_directories("${LDLIDAR_SDK_DIR}/include/")
add_library(${LDLIDAR_LIB_NAME} STATIC ${LDLIDAR_SRCS})

# Récupération de la définition pour JSON #
###########################################
set(JSON_LIB_DIR "${DOWNLOAD_DIR}/json")
include_directories(${JSON_LIB_DIR})

# Récupération de la définition pour SPDLOG #
#############################################
set(SPDLOG_LIB_DIR "${DOWNLOAD_DIR}/spdlog")
include_directories(${SPDLOG_LIB_DIR})

# Récupération des sources du projet principale #
#################################################
file(GLOB_RECURSE SRCS "src/*.cpp")
file(GLOB_RECURSE HEADERS "src/*.h")

add_executable(${PROJECT_NAME} ${SRCS} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${RPLIDAR_LIB_NAME} ${LDLIDAR_LIB_NAME} pthread)
