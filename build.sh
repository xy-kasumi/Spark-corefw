#!/bin/bash
# Build script for Spark firmware using Windows Zephyr installation

WEST_EXE="/mnt/c/Users/xyx/Documents/zephyrproject/.venv/Scripts/west.exe"

# Pass all arguments to west
$WEST_EXE build "$@"