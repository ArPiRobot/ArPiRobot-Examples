cmake_minimum_required (VERSION 3.20)
project (ArPiRobot-UserRobot)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(CMAKE_HOST_WIN32)
	set(HOMEDIR $ENV{UserProfile})
else()
	set(HOMEDIR $ENV{HOME})
endif()

include_directories(${HOMEDIR}/.arpirobot/corelib/include ${PROJECT_SOURCE_DIR}/include)

if("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "arm")
     link_directories(${HOMEDIR}/.arpirobot/corelib/lib/armv6)
elseif("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
     link_directories(${HOMEDIR}/.arpirobot/corelib/lib/aarch64)
else()
     message(FATAL_ERROR "Unknown architecture for linking libraries.")
endif()

file (GLOB_RECURSE SOURCES
     ${PROJECT_SOURCE_DIR}/src/*.c
     ${PROJECT_SOURCE_DIR}/src/*.cpp
     ${PROJECT_SOURCE_DIR}/include/*.h
     ${PROJECT_SOURCE_DIR}/include/*.hpp
)

find_package(OpenCV REQUIRED core)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET gstreamer-1.0)
add_executable(robot ${SOURCES})
target_link_libraries(robot arpirobot-core pthread opencv_core PkgConfig::GSTREAMER)