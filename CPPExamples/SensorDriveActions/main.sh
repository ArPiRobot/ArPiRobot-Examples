#!/usr/bin/env bash

function binarch(){
    printf "$(readelf -h $1 | grep Machine: | sed -r 's/\s+Machine:\s+//g')"
}

DIR="$(realpath "$(dirname "${BASH_SOURCE[0]}")")"
cd $DIR



arch=$(binarch ./robot)
if [ "$arch" = "ARM" ]; then
    LD_LIB_PATH_NEW=./armv6:$LD_LIBRARY_PATH
elif [ "$arch" = "AArch64" ]; then
    LD_LIB_PATH_NEW=./aarch64:$LD_LIBRARY_PATH
else
    echo "Unknown architecture. Cannot run robot program!"
fi

if [ "$1" = "--debug" ]; then
    sudo -E LD_LIBRARY_PATH=$LD_LIB_PATH_NEW lldb-server g 0.0.0.0:2000 ./robot
else
    sudo -E LD_LIBRARY_PATH=$LD_LIB_PATH_NEW ./robot
fi
