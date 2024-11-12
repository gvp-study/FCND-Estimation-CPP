#!/bin/bash

# Directory containing the .txt files
CONFIG_DIR="config/traj"

# Loop through each .txt file in the directory
for file in "$CONFIG_DIR"/*.txt; do
    # Use sed to remove carriage return characters and save changes in-place
    sed -i '' 's/\r$//' "$file"
    echo "Converted $file to Unix line endings."
done
