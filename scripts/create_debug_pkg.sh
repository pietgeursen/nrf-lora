#! /bin/bash
adafruit-nrfutil dfu genpkg --dev-type 0x0052  --application target/app.hex out.zip
