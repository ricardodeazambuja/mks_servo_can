#!/bin/bash

# Script to recursively find and delete __pycache__, *.egg-info, and .pytest_cache directories

# Check if a directory path is provided as an argument
if [ -z "$1" ]; then
  echo "Usage: $0 <directory_path>"
  exit 1
fi

TARGET_DIR="$1"

# Check if the target directory exists
if [ ! -d "$TARGET_DIR" ]; then
  echo "Error: Directory '$TARGET_DIR' not found."
  exit 1
fi

echo "Searching for and deleting specified directories in '$TARGET_DIR'..."

# Find and delete __pycache__ directories
echo "Looking for __pycache__ directories..."
find "$TARGET_DIR" -type d -name "__pycache__" -exec echo "Deleting: {}" \; -exec rm -rf {} \;

# Find and delete *.egg-info directories
echo "Looking for *.egg-info directories..."
find "$TARGET_DIR" -type d -name "*.egg-info" -exec echo "Deleting: {}" \; -exec rm -rf {} \;

# Find and delete .pytest_cache directories
echo "Looking for .pytest_cache directories..."
find "$TARGET_DIR" -type d -name ".pytest_cache" -exec echo "Deleting: {}" \; -exec rm -rf {} \;

echo "Cleanup complete."