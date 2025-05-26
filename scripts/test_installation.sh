#!/bin/bash
set -e


echo "Installing wheel..."
pip install wheelhouse/*.whl

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
