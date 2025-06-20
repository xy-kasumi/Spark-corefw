#!/bin/bash

# Check if west command is available
if ! command -v west &> /dev/null; then
    echo "Error: 'west' command not found."
    echo "Please activate the Zephyr virtual environment first."
    exit 1
fi

# Check if we're in the correct directory
if [ ! -f "app/src/main.c" ]; then
    echo "Error: Not in the correct directory."
    echo "Please run this script from the Spark-corefw repository root directory."
    exit 1
fi

west flash
