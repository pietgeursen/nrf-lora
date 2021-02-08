#! /bin/bash
#adafruit-nrfutil dfu genpkg --dev-type 0x0052 --application ./target/app.hex  out.zip --softdevice ../nrf52_sdk/s140nrf52720/s140_nrf52_7.2.0_softdevice.hex
adafruit-nrfutil dfu genpkg --dev-type 0x0052 --application ./target/app.hex  out.zip
