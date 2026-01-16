#!/bin/bash
set -e

echo "Building pylibfranka package..."

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

cd "$PROJECT_ROOT"

export PATH=$HOME/.local/bin:$PATH
export pybind11_DIR=/usr/lib/cmake/pybind11

echo "Installing package..."
pip install . --no-build-isolation || pip install .

echo "Building wheel..."
python3 -m build --wheel

mkdir -p wheelhouse

echo "Package build complete!"
echo "Built wheels:"
ls -la dist/*.whl
