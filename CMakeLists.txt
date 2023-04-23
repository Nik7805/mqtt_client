set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(TOOLCHAIN_PATH /opt/gcc-arm-10.3-2021.07-x86_64-arm-none-linux-gnueabihf/bin)
set(SYSROOT_PATH ${TOOLCHAIN_PATH}/../arm-none-linux-gnueabihf/libc)
set(TOOLCHAIN_PREFIX arm-none-linux-gnueabihf-)

set(CMAKE_C_COMPILER ${TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}g++)
set(CMAKE_LD_COMPILER ${TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}ld)
set(CMAKE_AR_COMPILER ${TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}ar)

set(CMAKE_SYSROOT ${SYSROOT_PATH})
set(CMAKE_EXE_LINKER_FLAGS "--sysroot=${SYSROOT_PATH}")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

cmake_minimum_required(VERSION 3.0.0)
project(mqtt_client VERSION 0.1.0)

set(SOURCE_EXE 
src/main.cpp
lib/BMP280_driver/bmp280.c
src/BMP280/bmp280_app.cpp
)

set(BUILD_SHARED_LIBS NO)
set(CMAKE_BUILD_TYPE "Debug")
set (CMAKE_CXX_STANDARD 17)

include_directories(
${PROJECT_SOURCE_DIR}/lib/c-periphery/src
${PROJECT_SOURCE_DIR}/src/
${PROJECT_SOURCE_DIR}/src/BMP280
${PROJECT_SOURCE_DIR}/lib/mosquitto/include
${PROJECT_SOURCE_DIR}/lib/BMP280_driver
)

add_executable(mqtt_client ${SOURCE_EXE})

add_subdirectory(${CMAKE_SOURCE_DIR}/lib/mosquitto)
target_link_libraries(${PROJECT_NAME} libmosquitto_static)

add_subdirectory(${CMAKE_SOURCE_DIR}/lib/c-periphery)
target_link_libraries(${PROJECT_NAME} periphery)

