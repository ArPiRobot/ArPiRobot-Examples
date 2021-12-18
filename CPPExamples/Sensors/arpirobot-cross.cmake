if(CMAKE_HOST_WIN32)
    set(HOMEDIR $ENV{UserProfile})
    set(EXTENSION ".exe")
else()
    set(HOMEDIR $ENV{HOME})
    set(EXTENSION "")
endif()

set(POSSIBLE_PREFIXES
    # Software float, 32-bit
    arm-linux-gnueabi
    armv6-rpi-linux-gnueabi
    armv7-rpi2-linux-gnueabi
    armv8-rpi3-linux-gnueabi
    armv8-rpi4-linux-gnueabi

    # Hardware float, 32-bit
    arm-linux-gnueabihf
    armv6-rpi-linux-gnueabihf
    armv7-rpi2-linux-gnueabihf
    armv8-rpi3-linux-gnueabihf
    armv8-rpi4-linux-gnueabihf

    # Software float, 64-bit
    aarch64-linux-gnueabi
    aarch64-rpi3-linux-gnueabi
    aarch64-rpi4-linux-gnueabi

    # Hardware float, 64-bit
    aarch64-linux-gnueabihf
    aarch64-rpi3-linux-gnueabihf
    aarch64-rpi4-linux-gnueabihf
)

set(target_host "")
foreach(POSSIBLE_PREFIX ${POSSIBLE_PREFIXES})
    if(EXISTS "${HOMEDIR}/.arpirobot/toolchain/${POSSIBLE_PREFIX}")
        set(target_host ${POSSIBLE_PREFIX})
        break()
    endif()
endforeach()

if("${target_host}" STREQUAL "")
    message(FATAL_ERROR "ArPiRobot toolchain not found!")
endif()

message(STATUS "Found toolchain with prefix ${target_host}")


set(standalone_toolchain ${HOMEDIR}/.arpirobot/toolchain)
set(cc_compiler gcc)
set(cxx_compiler g++)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER ${standalone_toolchain}/bin/${target_host}-${cc_compiler}${EXTENSION} CACHE PATH "C Compiler")
set(CMAKE_CXX_COMPILER ${standalone_toolchain}/bin/${target_host}-${cxx_compiler}${EXTENSION} CACHE PATH "C++ Compiler")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)