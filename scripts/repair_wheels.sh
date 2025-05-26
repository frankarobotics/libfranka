#!/bin/bash
set -e

echo "Repairing wheels with auditwheel..."

# Ensure user-level Python packages are in PATH
export PATH=$HOME/.local/bin:$PATH

# Create wheelhouse directory if it doesn't exist
mkdir -p wheelhouse

# Try different approaches to run auditwheel
if command -v auditwheel &> /dev/null; then
    echo "Using system auditwheel"
    auditwheel repair dist/*.whl -w wheelhouse/
elif [ -f $HOME/.local/bin/auditwheel ]; then
    echo "Using user-local auditwheel"
    $HOME/.local/bin/auditwheel repair dist/*.whl -w wheelhouse/
else
    echo "Installing auditwheel and running repair"
    python3 -m pip install --user auditwheel
    $HOME/.local/bin/auditwheel repair dist/*.whl -w wheelhouse/
fi

# If auditwheel fails, copy the wheel directly as fallback
if [ ! -f wheelhouse/*.whl ]; then
    echo "auditwheel failed, copying wheel directly"
    cp dist/*.whl wheelhouse/
fi

# List the final wheels
echo "Final wheels in wheelhouse:"
ls -la wheelhouse/
