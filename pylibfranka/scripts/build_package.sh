#!/bin/bash
set -e

echo "Building pylibfranka package..."

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

cd "$PROJECT_ROOT"

export PATH=$HOME/.local/bin:$PATH
export pybind11_DIR=/usr/lib/cmake/pybind11

echo "Building wheel..."
# Builds in place (no source copy), so setup.py can reach the repo root to drive
# the cmake build. Requires the build deps from setup_dependencies.sh
# (incl. pybind11-stubgen) to be present.
python3 -m build --wheel --no-isolation

mkdir -p wheelhouse

echo "Package build complete!"
echo "Built wheels:"
ls -la dist/*.whl
