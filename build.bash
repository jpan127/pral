# set -e

# clear

# clang++ -std=c++17 -Wall -Werror -ggdb3 -Isrc src/sim1.cpp -o sim1

# time ./sim1

# time /c/Python35/python.exe plot.py
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

# ####################
# #     Run Tests    #
# ####################
# set +e
# ./build/tests $1 2>/dev/null 1>/dev/null

# # If it failed, then rerun with stdout stderr
# return_value=$?
# if [ $return_value != "0" ]; then
#     ./build/tests $1
#     exit 1
# fi

# echo "--------------------"
# echo "| Tests Successful |"
# echo "--------------------"
