if(CMAKE_HOST_WIN32)
    set(HOMEDIR "$ENV{UserProfile}")
    string(REPLACE "\\" "/" HOMEDIR "${HOMEDIR}")
    set(EXTENSION ".exe")
else()
    set(HOMEDIR "$ENV{HOME}")
    set(EXTENSION "")
endif()

if(CMAKE_HOST_APPLE)
    execute_process(COMMAND clang --version OUTPUT_VARIABLE CLANGOUT)
    string(TOLOWER ${CLANGOUT} CLANGOUT)
    string(FIND ${CLANGOUT} "apple clang" APPLECLANGPOS)
    if(${APPLECLANGPOS} STREQUAL "-1")
        # Clang in path is not apple clang. Assume user modified the path
        set(CLANG clang)
        set(CLANGPP clang++)
    else()
        # MacOS includes Apple Clang, which doesn't support lld
        # Which is needed for cross compiling
        # Need to use llvm clang instead, which is assumed to be installed by
        # brew at /usr/local/opt/llvm/
        set(CLANG /usr/local/opt/llvm/bin/clang)
        set(CLANGPP /usr/local/opt/llvm/bin/clang++)
    endif()
else()
    # Just use whatever is in the path elsewhere
    set(CLANG clang)
    set(CLANGPP clang++)
endif()

set(CMAKE_CROSSCOMPILING TRUE)
SET(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

SET(TARGET arm-linux-gnueabihf)
set(CMAKE_SYSROOT "${HOMEDIR}/.arpirobot/sysroot/armv6")

# Make pkg-config work properly with sysroot
set(ENV{PKG_CONFIG_DIR} "")
set(ENV{PKG_CONFIG_LIBDIR} "${CMAKE_SYSROOT}/usr/lib/arm-linux-gnueabihf/pkgconfig")
set(ENV{PKG_CONFIG_SYSROOT_DIR} ${CMAKE_SYSROOT})

if(NOT EXISTS "${CMAKE_SYSROOT}")
    message(FATAL_ERROR "Sysroot directory is missing.")
endif()

SET(CMAKE_C_COMPILER ${CLANG})
SET(CMAKE_C_COMPILER_TARGET arm-linux-gnueabihf)

SET(CMAKE_CXX_COMPILER ${CLANGPP})
SET(CMAKE_CXX_COMPILER_TARGET arm-linux-gnueabihf)

SET(CMAKE_ASM_COMPILER ${CLANG})
SET(CMAKE_ASM_COMPILER_TARGET arm-linux-gnueabihf)

# Find C++ include path for whatever version of libstdc++ is used by the sysroot
file(GLOB CPP_INCLUDES "${CMAKE_SYSROOT}/usr/include/c++/*/")
file(GLOB CPP_INCLUDES_2 "${CMAKE_SYSROOT}/usr/include/arm-linux-gnueabihf/c++/*/")
list(JOIN CPP_INCLUDES "-isystem" CPP_INCLUDES_STR)
list(JOIN CPP_INCLUDES_2 "-isystem" CPP_INCLUDES_STR_2)

# Note: --sysroot automatically passed if CMAKE_SYSROOT is set
# Note: -target is automatically passed by cmake as set above
# Note: Linking with lld since it is cross linker natively
#       Avoids needing gcc for target system just to link
#       Thus, using clang and lld, no cross GNU toolchain is needed. Only sysroot.
SET(SHARED_FLAGS "-fuse-ld=lld -Qunused-arguments -march=armv6z -mtune=arm1176jzf-s -mfpu=vfp -mfloat-abi=hard")
SET(CMAKE_C_FLAGS "${SHARED_FLAGS}" CACHE STRING "C compiler flags")
SET(CMAKE_CXX_FLAGS "${SHARED_FLAGS} -isystem ${CPP_INCLUDES_STR} -isystem ${CPP_INCLUDES_STR_2}" CACHE STRING "C++ compiler flags")
set(LINK_FLAGS "${SHARED_FLAGS} -fuse-ld=lld" CACHE STRING "Linker flags")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
