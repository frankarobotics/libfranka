#!/bin/bash
set -e

echo "Setting up dependencies..."

# Update package lists (allow failure)
sudo apt-get update || true

# Install system dependencies
sudo apt-get install -y libeigen3-dev libpoco-dev pybind11-dev || true

# Install Python dependencies
echo "Installing Python dependencies..."
python3 -m pip install --user auditwheel setuptools build wheel patchelf pybind11 numpy cmake

# Add user-level Python bin to PATH
export PATH=$HOME/.local/bin:$PATH

# Verify auditwheel is available
if command -v auditwheel &> /dev/null; then
    echo "auditwheel found at: $(which auditwheel)"
else
    echo "WARNING: auditwheel not found in PATH"
fi

# Set up CMake for pybind11
export pybind11_DIR=/usr/lib/cmake/pybind11

echo "Dependencies setup complete!"
