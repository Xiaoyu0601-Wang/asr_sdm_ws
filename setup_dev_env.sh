#!/usr/bin/env bash
# Set up development environment for Autoware Core/Universe.
# Usage: setup-dev-env.sh <ros2_installation_type('core' or 'universe')> [-y] [-v] [--no-nvidia]
# Note: -y option is only for CI.

set -e

# Function to print help message
print_help() {
    echo "Usage: setup-dev-env.sh [OPTIONS]"
    echo "Options:"
    echo "  --help                  Display this help message"
    echo "  -h                      Display this help message"
    echo "  -y                      Use non-interactive mode"
    echo "  -v                      Enable debug outputs"
    echo "  --no-nvidia             Disable installation of the NVIDIA-related roles ('cuda' and 'tensorrt')"
    echo "  --no-cuda-drivers       Disable installation of 'cuda-drivers' in the role 'cuda'"
    echo "  --runtime               Disable installation dev package of role 'cuda' and 'tensorrt'"
    echo "  --data-dir              Set data directory (default: $HOME/asr_sdm_data)"
    echo "  --download-artifacts    Download artifacts"
    echo "  --module                Specify the module (default: all)"
    echo "  --ros-distro            Specify ROS distribution (rolling or jazzy, default: jazzy)"
    echo ""
}
