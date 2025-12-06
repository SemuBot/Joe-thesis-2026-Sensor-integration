#!/bin/bash

# Setup automatic removal of build files and make project
# easier to build again.
rm -rf build/*
cd build
cmake ..
make
cd ..
cp build/ultrasonic_publisher.uf2 /media/aleks/RP2350/