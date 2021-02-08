#! /bin/bash
cargo objcopy --release -- -O ihex ./target/app.hex
nrfjprog --program ./target/app.hex --sectorerase --verify
nrfjprog --reset
