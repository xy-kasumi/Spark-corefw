#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

cargo fmt --all --manifest-path Cargo.toml "$@"
cargo fmt --all --manifest-path firmware/Cargo.toml "$@"
