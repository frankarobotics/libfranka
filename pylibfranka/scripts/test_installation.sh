#!/bin/bash
set -e

# Accept wheel pattern as first argument, default to all wheels
WHEEL_PATTERN=${1:-"*.whl"}

echo "Installing wheel with pattern: $WHEEL_PATTERN"

# Find wheels matching the pattern
if [[ "$WHEEL_PATTERN" == "*.whl" ]]; then
    # Install all wheels (default behavior)
    pip install wheelhouse/*.whl
else
    # Find specific wheel matching pattern
    echo "Available wheels:"
    ls -la wheelhouse/
    echo "Filtering for wheels matching: $WHEEL_PATTERN"
    MATCHING_WHEEL=$(find wheelhouse/ -name "*${WHEEL_PATTERN}" | head -1)
    if [ -z "$MATCHING_WHEEL" ]; then
        echo "No wheel found matching pattern: $WHEEL_PATTERN"
        exit 1
    fi
    echo "Found wheel: $MATCHING_WHEEL"
    pip install "$MATCHING_WHEEL"
fi

# Test the installation
echo "Testing pylibfranka import..."
cd /
python -c "
import pylibfranka
print('pylibfranka imported successfully')
try:
    print(f'Version: {pylibfranka.__version__}')
except AttributeError:
    print('No version attribute found')
"

echo "Installation test complete!"
