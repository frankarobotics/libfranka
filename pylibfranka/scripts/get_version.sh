#!/bin/bash
set -e

# Get Package Version from CMakeLists.txt (single source of truth)
echo "Getting package version from CMakeLists.txt..."
PACKAGE_VERSION=$(grep -oP 'set\(libfranka_VERSION\s+\K[\d.]+' CMakeLists.txt)

if [ -z "$PACKAGE_VERSION" ]; then
    echo "Error: Could not extract version from CMakeLists.txt"
    exit 1
fi

echo "Package version: $PACKAGE_VERSION"

# Export for use in pipelines
echo "PACKAGE_VERSION=$PACKAGE_VERSION" >> version_info.properties
export PACKAGE_VERSION
echo "PACKAGE_VERSION=$PACKAGE_VERSION"
