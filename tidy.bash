#!/bin/bash

set -e

SUPRESSED_CHECKS="\
-cppcoreguidelines-avoid-c-arrays,\
-cppcoreguidelines-pro-bounds-array-to-pointer-decay,\
-cppcoreguidelines-pro-bounds-constant-array-index,\
-cppcoreguidelines-pro-bounds-pointer-arithmetic,\
-cppcoreguidelines-pro-type-union-access,\
-cppcoreguidelines-pro-type-vararg,\
-fuchsia-default-arguments,\
-fuchsia-overloaded-operator,\
-google-build-using-namespace,\
-google-runtime-references,\
-hicpp-avoid-c-arrays,\
-hicpp-no-array-decay,\
-hicpp-vararg,\
-llvm-header-guard,\
-misc-definitions-in-headers,\
-modernize-avoid-c-arrays,\
-readability-avoid-const-params-in-decls,\
-readability-named-parameter\
"

if [[ -z "${CLANG_TIDY}" ]]; then
    # Default
    CLANG_TIDY="clang-tidy-8"
    # If clang doesn't exist, exit
    if ! command -v $CLANG_TIDY &> /dev/null ; then
        exit 0
    fi
fi

# Compiler flags
FLAGS="-std=c++14 -stdlib=libc++ -Imodules/optional -Iargparse/include -Imodules/catch2"

# Run clang-tidy
python3 runtidy.py                    \
    --clang-tidy-binary=${CLANG_TIDY} \
    --checks=*,$SUPRESSED_CHECKS      \
    --header-filter=argparse/include/ \
    --cxxflags="$FLAGS"               \
    argparse/src/*.cpp

echo "--------------------"
echo "|    All Tidied    |"
echo "--------------------"
