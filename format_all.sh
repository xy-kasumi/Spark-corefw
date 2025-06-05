#!/bin/bash

# Format all C source and header files in the spark directory
# Uses clang-format with Chromium style and ensures proper file-end newlines

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Check if clang-format is available
if ! command -v clang-format &> /dev/null; then
    echo -e "${RED}Error: clang-format not found. Please install it:${NC}"
    echo "  sudo apt update && sudo apt install clang-format"
    exit 1
fi

echo "Formatting C/C++ files in spark directory..."

# Find all .c and .h files in specific directories
files=$(find spark/app spark/drivers spark/tests spark/include -type f \( -name "*.c" -o -name "*.h" \) 2>/dev/null | sort)

if [ -z "$files" ]; then
    echo "No C/C++ files found in spark directory"
    exit 0
fi

# Count total files
total=$(echo "$files" | wc -l)
formatted=0
fixed_newlines=0

for file in $files; do
    # Format with clang-format using Chromium style
    clang-format -i -style=Chromium "$file"
    
    # Check if file ends with newline
    if [ -n "$(tail -c 1 "$file")" ]; then
        # Add newline at end of file
        echo >> "$file"
        fixed_newlines=$((fixed_newlines + 1))
        echo -e "${YELLOW}Fixed missing newline:${NC} $file"
    fi
    
    formatted=$((formatted + 1))
done

echo -e "\n${GREEN}Formatting complete!${NC}"
echo "Formatted $formatted/$total files"
if [ $fixed_newlines -gt 0 ]; then
    echo -e "${YELLOW}Added missing newlines to $fixed_newlines files${NC}"
fi
