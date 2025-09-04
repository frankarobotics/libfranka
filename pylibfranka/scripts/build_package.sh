#!/bin/bash
set -e

echo "Building package..."

# Ensure user-level Python packages are in PATH
export PATH=$HOME/.local/bin:$PATH

# Set up CMake for pybind11
export pybind11_DIR=/usr/lib/cmake/pybind11

# Install the package in development mode
echo "Installing package..."
pip install .

# Build the wheel
echo "Building wheel..."
python3 -m build --wheel

# Create wheelhouse directory
mkdir -p wheelhouse

echo "Package build complete!"
echo "Built wheels:"
ls -la dist/*.whl
