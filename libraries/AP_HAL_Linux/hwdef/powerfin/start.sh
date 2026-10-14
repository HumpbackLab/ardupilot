#!/bin/sh
set -eu

app_dir=$(unset CDPATH; cd "$(dirname "$0")" && pwd)
cd "$app_dir"
mkdir -p logs terrain

modprobe sc7u22

exec ./arducopter \
    --serial0 udpin:0.0.0.0:14550 \
    --serial1 /dev/ttyS1 \
    --serial2 /dev/ttyS2 \
    --serial3 /dev/ttyS3 \
    --serial4 /dev/ttyS4 \
    --serial5 /dev/ttyS5 \
    "$@"
