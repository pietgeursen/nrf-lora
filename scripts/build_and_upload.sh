#! /bin/bash
cargo build --release
./scripts/create_hex.sh
./scripts/create_debug_pkg.sh
./scripts/upload_pkg.sh
