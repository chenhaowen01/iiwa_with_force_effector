#!/bin/bash

device=${1-can0}
shift

bitrate=${1-1000000}
shift

sudo ip link set $device down
sudo ip link set $device up type can bitrate $bitrate