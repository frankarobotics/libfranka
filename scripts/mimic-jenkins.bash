#!/usr/bin/env bash
set -x  # Print commands and their arguments as they are executed
set -e  # Exit immediately on error
trap 'echo "Error: Script failed at line $LINENO" >&2; exit 1' ERR

# This script performs the same actions that the Jenkins job will perform.
# EXCEPT for publishing
# It's done in a script so that you don't have to do this manually.

# IMPORTANT !
# In this script STRICT is set to OFF (-werror is not on),
# so that the build does not fail on warnings.
# The real Jenkins Job has STRICT=ON and will fail on warnings

echo 'Clean Workspace build dirs'
rm -rf build-*

echo 'Build'
## Building libfranka will use all your cores, virtual or otherwise
## Also, will consume about 16GB RAM - if available.
## This helps you maintain some control while the build progresses.
TASKS=$(( $(nproc) > 1 ? $(nproc) - 1 : 1 ))
echo "Using $TASKS build tasks"

echo 'Build debug'
mkdir build-debug && pushd build-debug
cmake -DCMAKE_BUILD_TYPE=Debug -DSTRICT=OFF -DBUILD_COVERAGE=OFF \
      -DBUILD_DOCUMENTATION=OFF -DBUILD_EXAMPLES=ON -DBUILD_TESTS=ON ..
make -j${TASKS}
popd

echo 'Build release'
mkdir build-release && pushd build-release
cmake -DCMAKE_BUILD_TYPE=Release -DSTRICT=OFF -DBUILD_COVERAGE=OFF \
      -DBUILD_DOCUMENTATION=ON -DBUILD_EXAMPLES=ON -DBUILD_TESTS=ON ..
make -j${TASKS}
popd

echo 'Build examples (debug)'
mkdir build-debug-examples && pushd build-debug-examples
cmake -DFranka_DIR:PATH=../build-debug.${DISTRO} ../examples
make -j${TASKS}
popd

echo 'Build examples (release)'
mkdir build-release-examples && pushd build-release-examples
cmake -DFranka_DIR:PATH=../build-release.${DISTRO} ../examples
make -j${TASKS}
popd

echo 'Build coverage'
mkdir build-coverage && pushd build-coverage
cmake -DCMAKE_BUILD_TYPE=Debug -DBUILD_COVERAGE=ON \
      -DBUILD_DOCUMENTATION=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=ON ..
make -j${TASKS}
popd

echo 'Lint'
mkdir build-lint && pushd build-lint
cmake -DBUILD_COVERAGE=OFF -DBUILD_DOCUMENTATION=OFF -DBUILD_EXAMPLES=ON -DBUILD_TESTS=ON ..
make check-tidy -j${TASKS}
popd

echo 'Format'
mkdir build-format && pushd build-format
cmake -DBUILD_COVERAGE=OFF -DBUILD_DOCUMENTATION=OFF -DBUILD_EXAMPLES=ON -DBUILD_TESTS=ON ..
make check-format -j${TASKS}
popd

echo 'Coverage'
mkdir build-coverage && pushd build-coverage
cmake -DBUILD_COVERAGE=ON -DBUILD_DOCUMENTATION=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=ON ..
make coverage -j${TASKS}
popd

echo 'Test Debug'
mkdir build-debug && pushd build-debug
ctest -V
popd

pushd build-release
ctest -V
popd
