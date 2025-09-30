#!/bin/bash

set -e

DOCS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Set locale to avoid locale errors
export LC_ALL=C.UTF-8
export LANG=C.UTF-8

# Add sphinx to PATH
export PATH="$HOME/.local/bin:$PATH"

# Navigate to docs directory
cd "$DOCS_DIR"

# Build documentation
echo "Building documentation..."
make clean
make html

echo ""
echo "✓ Documentation built successfully!"
echo "  Location: $DOCS_DIR/_build/html/index.html"
echo ""
echo "To view:"
echo "  ./view_docs.sh"
echo "  or open file://$DOCS_DIR/_build/html/index.html in your browser"

