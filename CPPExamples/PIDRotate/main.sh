#!/usr/bin/env bash

function binarch(){
    printf "$(readelf -h $1 | grep Machine: | sed -r 's/\s+Machine:\s+//g')"
}

DIR="$(realpath "$(dirname "${BASH_SOURCE[0]}")")"
cd $DIR


arch=$(binarch ./robot)
if [ "$arch" = "ARM" ]; then
    sudo LD_LIBRARY_PATH=./armv6:$LD_LIBRARY_PATH ./robot
elif [ "$arch" = "AArch64" ]; then
    sudo LD_LIBRARY_PATH=./aarch64:$LD_LIBRARY_PATH ./robot
else
    echo "Unknown architecture. Cannot run robot program!"
fi
