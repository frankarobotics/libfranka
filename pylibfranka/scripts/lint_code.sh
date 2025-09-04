#!/bin/bash
set -e

echo "Running code linting..."

# Ensure user-level Python packages are in PATH
export PATH=$HOME/.local/bin:$PATH

# Install flake8 if not already installed
if ! command -v flake8 &> /dev/null; then
    echo "Installing flake8..."
    python3 -m pip install --user flake8
fi

# Run linting (allow it to fail without stopping the build)
echo "Running flake8 on pylibfranka and examples..."
flake8 pylibfranka examples || echo "Linting issues found but continuing"

echo "Linting complete!"
