#!/bin/bash
#tells nix shell what kind of interpreter to run
set -e
#Set allows you to change the values of shell options and set positional parameters
#Exit immediately if a command exits with a non-zero status.
#git clone https://github.com/chobitsfan/VINS-Fusion.git
#cd VINS-Fusion
#git checkout -t origin/apm_wiki
#sudo apt install -y libceres-dev
#cd vins_estimator
#cmake .
#make -j2
#cd ../../
#
#git clone https://github.com/luxonis/depthai-core.git --branch v2.25.0 --recursive
#cd depthai-core
#cmake -S. -Bbuild
#cmake --build build --target install
#cd ../
#
#git clone https://github.com/chobitsfan/oak_d_vins_cpp.git
#cd oak_d_vins_cpp
#git checkout -t origin/apm_wiki
#cmake -D'depthai_DIR=../depthai-core/build/install/lib/cmake/depthai' .
#make
#cd ..
#
#git clone https://github.com/chobitsfan/mavlink-udp-proxy.git
#cd mavlink-udp-proxy
#git checkout -t origin/apm_wiki
git submodule update --init --recursive
./build_it
cd ..
