#!/bin/bash

DIR="$(realpath "$(dirname "${BASH_SOURCE[0]}")")"
cd $DIR

sudo LD_LIBRARY_PATH=.:$LD_LIBRARY_PATH ./robot