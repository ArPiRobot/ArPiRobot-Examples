#!/usr/bin/env bash

function binarch(){
    printf "$(readelf -h $1 | grep Machine: | sed -r 's/\s+Machine:\s+//g')"
}

DIR="$(realpath "$(dirname "${BASH_SOURCE[0]}")")"
cd $DIR



arch=$(binarch $(which python3))
if [ "$arch" = "ARM" ]; then
    LD_LIB_PATH_NEW=./armv6:$LD_LIBRARY_PATH
elif [ "$arch" = "AArch64" ]; then
    LD_LIB_PATH_NEW=./aarch64:$LD_LIBRARY_PATH
else
    echo "Unknown architecture. Cannot run robot program!"
fi

if [ "$1" = "--debug" ]; then
    sudo -E LD_LIBRARY_PATH=$LD_LIB_PATH_NEW python3 -u -Xfrozen_modules=off -m debugpy --listen 0.0.0.0:2000 --wait-for-client main.py
else
    sudo -E LD_LIBRARY_PATH=$LD_LIB_PATH_NEW PYTHONPATH=. python3 -u main.py
fi