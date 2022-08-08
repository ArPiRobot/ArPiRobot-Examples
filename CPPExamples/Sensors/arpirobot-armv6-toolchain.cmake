if(CMAKE_HOST_WIN32)
    set(HOMEDIR "$ENV{UserProfile}")
    set(EXTENSION ".exe")
else()
    set(HOMEDIR "$ENV{HOME}")
    set(EXTENSION "")
endif()

set(target_host "armv6-unknown-linux-gnueabihf")
set(standalone_toolchain "${HOMEDIR}/.arpirobot/toolchain/armv6")
set(cc_compiler gcc)
set(cxx_compiler g++)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER "${standalone_toolchain}/bin/${target_host}-${cc_compiler}${EXTENSION}" CACHE PATH "C Compiler")
set(CMAKE_CXX_COMPILER "${standalone_toolchain}/bin/${target_host}-${cxx_compiler}${EXTENSION}" CACHE PATH "C++ Compiler")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
