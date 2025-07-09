#!/usr/bin/env python3

import os
import argparse

# Define folder
folder = "RIO-T"

# List all .txt files in the folder
files = [f for f in os.listdir(folder) if f.endswith(".txt")]

# Process each file
for filename in files:
    input_path = os.path.join(folder, filename)
    output_filename = filename.replace(".txt", "_rio-t.txt")
    output_path = os.path.join(folder, output_filename)

    # Delete output file if it exists
    if os.path.exists(output_path):
        os.remove(output_path)

    # Process the file
    with open(input_path, 'r') as infile, open(output_path, 'w') as outfile:
        count = 0
        for line in infile:
            count += 1
            if count == 1:
                continue  # Skip header or first line

            parts = line.strip().split(',')
            try:
                timestamp = float(parts[0]) / 1e9  # Convert timestamp to seconds
                data = " ".join(parts[5:12])
                outfile.write(f"{timestamp} {data}\n")
            except ValueError:
                continue  # Skip lines with invalid data

    print(f"Processed {filename} -> {output_filename}")

print("All files processed!")




