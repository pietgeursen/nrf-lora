#! /bin/bash
adafruit-nrfutil dfu serial -p /dev/ttyACM0 -pkg out.zip -b 115200
