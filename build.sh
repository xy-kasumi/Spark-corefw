#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/firmware"
cargo build --release "$@"
cargo size --quiet --release "$@"
