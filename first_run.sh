#!/bin/bash

# https://buildkite.com/docs/pipelines/configure/writing-build-scripts#configuring-bash
set -euo pipefail  # Exit immediately if any command fails

# Detect OS
OS="$(uname -s)"

if [[ "$OS" == "Linux" ]]; then
    echo "Detected Linux. Installing system dependencies..."

    sudo apt-get update
    sudo apt-get install -y --no-install-recommends \
        ca-certificates \
        build-essential \
        sudo \
        libnlopt-dev \
        curl

    echo "Dependencies installed successfully."
elif [[ "$OS" == "Darwin" ]]; then
    echo "Detected Darwin. Installing system dependencies..."
    brew tap viamrobotics/brews
    brew install nlopt-static
else
    echo "Non-Linux system detected ($OS). Skipping Linux-specific dependency installation."
fi
