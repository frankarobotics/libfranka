#!/bin/bash
set -e

echo "Repairing wheels with auditwheel..."

# Get script and project directories
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

cd "$PROJECT_ROOT"

# Ensure user-level Python packages are in PATH
export PATH=$HOME/.local/bin:$PATH

# Create wheelhouse directory if it doesn't exist
mkdir -p wheelhouse

# Target platform for manylinux wheels
PLATFORM="${1:-manylinux_2_17_x86_64}"

# Function to run auditwheel
run_auditwheel() {
    local wheel="$1"
    local platform="$2"
    
    echo "Repairing: $wheel for $platform"
    
    # Try with specified platform first
    if auditwheel repair "$wheel" -w wheelhouse/ --plat "$platform" 2>/dev/null; then
        return 0
    fi
    
    # Fallback to auto-detection
    echo "Trying auto-detection..."
    if auditwheel repair "$wheel" -w wheelhouse/ 2>/dev/null; then
        return 0
    fi
    
    # Final fallback: copy as-is
    echo "Warning: auditwheel failed, copying wheel as-is"
    cp "$wheel" wheelhouse/
    return 0
}

# Find auditwheel
if command -v auditwheel &> /dev/null; then
    AUDITWHEEL="auditwheel"
elif [ -f "$HOME/.local/bin/auditwheel" ]; then
    AUDITWHEEL="$HOME/.local/bin/auditwheel"
else
    echo "Installing auditwheel..."
    PIP_USER_FLAG=""
    if [ -z "$VIRTUAL_ENV" ]; then
        PIP_USER_FLAG="--user"
    fi
    python3 -m pip install $PIP_USER_FLAG auditwheel patchelf
    AUDITWHEEL="$HOME/.local/bin/auditwheel"
fi

# Check for wheels in dist directory
if [ ! -d dist ] || [ -z "$(ls -A dist/*.whl 2>/dev/null)" ]; then
    echo "Error: No wheels found in dist/ directory"
    echo "Run ./scripts/build_package.sh first"
    exit 1
fi

# Repair each wheel
for whl in dist/*.whl; do
    if [ -f "$whl" ]; then
        run_auditwheel "$whl" "$PLATFORM"
    fi
done

# List the final wheels
echo ""
echo "Final wheels in wheelhouse:"
ls -la wheelhouse/

# Show wheel contents for verification
echo ""
echo "Wheel contents (showing bundled libraries):"
for whl in wheelhouse/*.whl; do
    if [ -f "$whl" ]; then
        echo "--- $whl ---"
        unzip -l "$whl" | grep -E "\.so|\.libs" | head -20 || true
    fi
done
