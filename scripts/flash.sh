#! /bin/bash
cargo objcopy --release -- -O ihex ./target/app.hex
nrfjprog --program ./target/app.hex --sectorerase
nrfjprog --reset
