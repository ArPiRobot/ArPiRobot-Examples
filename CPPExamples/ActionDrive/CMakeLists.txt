cmake_minimum_required (VERSION 3.0)
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

add_executable(robot ${SOURCES})
target_link_libraries(robot arpirobot-core pthread)

if(("${CMAKE_SYSTEM_NAME}" STREQUAL "Linux") OR ("${CMAKE_SYSTEM_NAME}" STREQUAL "Windows") OR ("${CMAKE_SYSTEM_NAME}" STREQUAL "Darwin"))
     target_link_libraries(robot serial)
endif()

if(("${CMAKE_SYSTEM_NAME}" STREQUAL "Linux") AND (("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "arm") OR ("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "aarch64")))
     target_link_libraries(robot pigpio)
endif()

if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Linux")
     target_link_libraries(robot libsoc)
endif()
