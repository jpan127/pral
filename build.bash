#!/bin/bash
clear

set -e

####################
#       Build      #
####################

# Make build directory
mkdir -p build

# Default
if [[ ! -v CXX ]]; then
    export CXX=/usr/bin/clang++-8
fi

if [[ ! -v STDLIB ]]; then
    export STDLIB=-stdlib=libc++
fi

# CMake
cd build
cmake ..
make -j
cd ..

echo "--------------------"
echo "| Build Successful |"
echo "--------------------"

####################
#     Run Tests    #
####################
set +e
./build/src/astar/hot_lava $1 2>/dev/null 1>/dev/null

# If it failed, then rerun with stdout stderr
return_value=$?
if [ $return_value != "0" ]; then
    ./build/src/astar/hot_lava $1
    exit 1
fi

echo "--------------------"
echo "| Tests Successful |"
echo "--------------------"
