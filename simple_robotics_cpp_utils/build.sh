#!/bin/bash

# exit in case of error. Otherwise, the script could go on.
set -e

# # Check running as root using $(id -u)
# TODO: how to detect when need to run this script as sudo
# if [ "$(id -u)" -ne 0 ]; then echo "Please run this script as sudo" 1>&2; exit 1; fi
# want to mkdir anywhere this script is called.
THIS_FILE_DIR=$(dirname "$0")
BUILD_PATH="$THIS_FILE_DIR/build"
if [ -d $BUILD_PATH ]; then rm -rf $BUILD_PATH; fi
mkdir $BUILD_PATH
cd $BUILD_PATH
# This is assuming the docker environment already has env var CUSTOM_INSTALL_PATH
cmake -DCMAKE_INSTALL_PREFIX=${CUSTOM_INSTALL_PATH} ..
make -j7
# make test runs ctest, which by default does not capture stdout
# make test
ctest -V
# TODO: and this one
# sudo make install
make install
cd $THIS_FILE_DIR
rm -rf $BUILD_PATH
echo "Done"



