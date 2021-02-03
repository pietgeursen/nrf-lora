#! /bin/bash
cargo objcopy --release -- -O ihex ./target/app.hex
