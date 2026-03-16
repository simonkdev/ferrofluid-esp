#!/bin/bash

# Get the absolute path of the script itself
SCRIPT_PATH="$(realpath "$0")"

# Ensure we're in a git repo
if [ ! -d ".git" ]; then
  echo "Not a git repository. Exiting."
  exit 1
fi

# Find and commit each file individually, excluding this script
find . -type f ! -path "./.git/*" | while read -r file; do
  FULL_PATH="$(realpath "$file")"
  if [ "$FULL_PATH" != "$SCRIPT_PATH" ]; then
    git add "$file"
    git commit -m "Add $(basename "$file")"
  fi
done

echo "All files committed individually (excluding this script)."
