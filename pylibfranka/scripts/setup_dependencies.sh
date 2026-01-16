#!/bin/bash
set -e

echo "Setting up dependencies for pylibfranka..."

# Get script and project directories
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Update package lists (allow failure for non-root users)
echo "Updating package lists..."
sudo apt-get update 2>/dev/null || apt-get update 2>/dev/null || true

# Install system dependencies
echo "Installing system dependencies..."
sudo apt-get install -y \
    build-essential \
    cmake \
    libeigen3-dev \
    libpoco-dev \
    libfmt-dev \
    pybind11-dev \
    python3-dev \
    python3-pip \
    patchelf \
    2>/dev/null || \
apt-get install -y \
    build-essential \
    cmake \
    libeigen3-dev \
    libpoco-dev \
    libfmt-dev \
    pybind11-dev \
    python3-dev \
    python3-pip \
    patchelf \
    2>/dev/null || true

# Install Python dependencies
echo "Installing Python dependencies..."
# Use --user only if not in a virtualenv (virtualenv doesn't support --user)
PIP_USER_FLAG=""
if [ -z "$VIRTUAL_ENV" ]; then
    PIP_USER_FLAG="--user"
fi
python3 -m pip install $PIP_USER_FLAG --upgrade \
    pip \
    setuptools \
    wheel \
    build \
    auditwheel \
    patchelf \
    pybind11 \
    numpy \
    cmake \
    twine

# Add user-level Python bin to PATH (only needed for --user installs)
export PATH="$HOME/.local/bin:$PATH"

# Verify tools are available
echo ""
echo "Verifying installed tools..."

check_tool() {
    local tool="$1"
    if command -v "$tool" &> /dev/null; then
        echo "✓ $tool found at: $(which $tool)"
        return 0
    elif [ -f "$HOME/.local/bin/$tool" ]; then
        echo "✓ $tool found at: $HOME/.local/bin/$tool"
        return 0
    else
        echo "✗ $tool not found"
        return 1
    fi
}

check_tool cmake
check_tool python3
check_tool pip
check_tool auditwheel
check_tool patchelf

# Set up CMake for pybind11
export pybind11_DIR=/usr/lib/cmake/pybind11

echo ""
echo "Dependencies setup complete!"
echo ""
echo "To build the wheel, run:"
echo "  cd $PROJECT_ROOT"
echo "  ./pylibfranka/scripts/build_package.sh"
echo "  ./pylibfranka/scripts/repair_wheels.sh"
echo "  ./pylibfranka/scripts/test_installation.sh"