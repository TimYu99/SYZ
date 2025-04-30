cmake_minimum_required(VERSION 3.5)

project(sdkExample LANGUAGES CXX)

include_directories( ${PROJECT_SOURCE_DIR}/islSdk/src)
include_directories( ${PROJECT_SOURCE_DIR}/src)

##set(OpenCV_DIR "E:/lib/opencv/opencv4.6.0/opencv/build/x64/vc15/lib") # OpenCVConfig.cmake 所在的目录//根据实际情况修改
set(OpenCV_DIR "E:/RuanJian/Files/opencv/build") # OpenCVConfig.cmake 所在的目录//根据实际情况修改
# 查找 OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
# 添加 OpenCV 的库路径
link_directories(${OpenCV_LIBRARY_DIRS})


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

#set(X32_BIT TRUE)

if (X32_BIT)
    add_definitions(-DSYSTEM_32BIT)
    message(STATUS "App build config 32 bit")  
else()
    add_definitions(-DSYSTEM_64BIT)
    message(STATUS "App build config 64 bit") 
endif()

if (WIN32)
    add_definitions(-DOS_WINDOWS)
    message(STATUS "App build config Windows")
elseif (UNIX)
    add_definitions(-DOS_UNIX)
    message(STATUS "App build config Unix")
endif()

set(HEADERS
    src/app.h
    src/imuManager.h
    src/isa500App.h
    src/isd4000App.h
    src/ism3dApp.h
    src/sonarApp.h
    src/platform.h
    src/gpsApp.h
"src/array_data.h" "src/data_logger.h" "src/global.h")

set(SOURCES
    src/main.cpp
    src/app.cpp
    src/imuManager.cpp
    src/isa500App.cpp
    src/isd4000App.cpp
    src/ism3dApp.cpp
    src/sonarApp.cpp
    src/platform.cpp
    src/gpsApp.cpp
 "src/array_data.cpp" "src/data_logger.cpp" "src/global.cpp" )

add_subdirectory(islSdk)
add_executable (${PROJECT_NAME} ${SOURCES} ${HEADERS} "src/array_data.cpp" "src/data_logger.cpp" "src/global.cpp" "src/array_data.h" "src/data_logger.h" "src/global.h")
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} islSdk)