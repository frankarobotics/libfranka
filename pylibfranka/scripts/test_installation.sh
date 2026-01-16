#!/bin/bash
set -e

# Test pylibfranka wheel installation
# Usage: ./test_installation.sh [wheel_pattern]
#
# Examples:
#   ./test_installation.sh                           # Install any wheel from wheelhouse/
#   ./test_installation.sh manylinux_2_17_x86_64.whl # Install specific platform wheel
#   ./test_installation.sh cp310                     # Install Python 3.10 wheel

# Accept wheel pattern as first argument, default to all wheels
WHEEL_PATTERN=${1:-"*.whl"}

echo "==========================================="
echo "Testing pylibfranka installation"
echo "==========================================="
echo ""

# Get script and project directories
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

WHEELHOUSE="$PROJECT_ROOT/wheelhouse"

if [ ! -d "$WHEELHOUSE" ]; then
    echo "Error: wheelhouse directory not found at $WHEELHOUSE"
    exit 1
fi

echo "Installing wheel with pattern: $WHEEL_PATTERN"
echo ""

# Find wheels matching the pattern
if [[ "$WHEEL_PATTERN" == "*.whl" ]]; then
    # Install first available wheel
    MATCHING_WHEEL=$(ls -1 "$WHEELHOUSE"/*.whl 2>/dev/null | head -1)
else
    # Find specific wheel matching pattern
    echo "Available wheels:"
    ls -la "$WHEELHOUSE"/*.whl
    echo ""
    echo "Filtering for wheels matching: $WHEEL_PATTERN"
    MATCHING_WHEEL=$(find "$WHEELHOUSE" -name "*${WHEEL_PATTERN}*" -name "*.whl" | head -1)
fi

if [ -z "$MATCHING_WHEEL" ] || [ ! -f "$MATCHING_WHEEL" ]; then
    echo "Error: No wheel found matching pattern: $WHEEL_PATTERN"
    echo "Available wheels in $WHEELHOUSE:"
    ls -la "$WHEELHOUSE"/*.whl 2>/dev/null || echo "  (none)"
    exit 1
fi

echo "Installing: $MATCHING_WHEEL"
pip install --force-reinstall "$MATCHING_WHEEL"

echo ""
echo "==========================================="
echo "Testing pylibfranka import..."
echo "==========================================="

# IMPORTANT: Change to a neutral directory to avoid importing from local pylibfranka folder
# The project has a pylibfranka/ folder which would shadow the installed package
cd /tmp
echo "Testing from directory: $(pwd)"
echo ""

python3 << 'EOF'
import sys
import os

print(f"Python version: {sys.version}")
print(f"Python executable: {sys.executable}")
print(f"Current directory: {os.getcwd()}")
print()

# Test basic import
print("Testing basic import...")
import pylibfranka
print(f"✓ pylibfranka imported successfully")
print(f"  Version: {pylibfranka.__version__}")
print(f"  Location: {pylibfranka.__file__}")

# Verify we're NOT importing from the local project folder
if "/workspace" in pylibfranka.__file__ or "/workspaces" in pylibfranka.__file__:
    print(f"✗ ERROR: Importing from project folder instead of installed package!")
    print(f"  This means the test is not testing the wheel correctly.")
    sys.exit(1)
print(f"✓ Confirmed: importing from installed package (not local folder)")
print()

# Test core classes
print("Testing core classes...")
from pylibfranka import (
    Robot,
    Gripper,
    Model,
    RobotState,
    GripperState,
    Duration,
    Errors,
    RobotMode,
    ControllerMode,
    RealtimeConfig,
)
print("✓ All core classes imported successfully")
print()

# Test control types
print("Testing control types...")
from pylibfranka import (
    JointPositions,
    JointVelocities,
    CartesianPose,
    CartesianVelocities,
    Torques,
)
print("✓ All control types imported successfully")
print()

# Test exceptions
print("Testing exceptions...")
from pylibfranka import (
    FrankaException,
    CommandException,
    ControlException,
    NetworkException,
    RealtimeException,
    InvalidOperationException,
)
print("✓ All exception types imported successfully")
print()

# Test creating objects (without connecting to robot)
print("Testing object creation...")
jp = JointPositions([0.0] * 7)
print(f"✓ JointPositions created: {jp.q}")

jv = JointVelocities([0.0] * 7)
print(f"✓ JointVelocities created: {jv.dq}")

t = Torques([0.0] * 7)
print(f"✓ Torques created: {t.tau_J}")

# Duration class exists (constructor may vary by version)
print(f"✓ Duration class available: {Duration}")
print()

# Verify version matches expected format
import re
if re.match(r'^\d+\.\d+\.\d+', pylibfranka.__version__):
    print(f"✓ Version format valid: {pylibfranka.__version__}")
else:
    print(f"✗ Warning: Unexpected version format: {pylibfranka.__version__}")
    sys.exit(1)

print()
print("=========================================")
print("All tests passed!")
print("=========================================")
EOF

echo ""
echo "Installation test complete!"
