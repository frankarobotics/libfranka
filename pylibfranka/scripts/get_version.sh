#!/bin/bash
set -e

# Get Package Version from setup.py
echo "Getting package version..."
PACKAGE_VERSION=$(grep -oP '(?<=version=")[^"]+' setup.py)
echo "Package version: $PACKAGE_VERSION"

# Export for use in pipelines
echo "PACKAGE_VERSION=$PACKAGE_VERSION" >> version_info.properties
export PACKAGE_VERSION
echo "PACKAGE_VERSION=$PACKAGE_VERSION"
