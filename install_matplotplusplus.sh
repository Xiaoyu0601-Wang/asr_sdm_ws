#!/usr/bin/env bash
#
# Install matplot++ (matplotplusplus) from source.
# Usage: ./install_matplotplusplus.sh [install_prefix]
#   install_prefix defaults to /usr/local
#
set -euo pipefail

PREFIX="${1:-/usr/local}"
JOBS="$(nproc 2>/dev/null || echo 4)"
REPO="https://github.com/alandefreitas/matplotplusplus.git"
BUILD_DIR="/tmp/matplotplusplus_build"

echo "=== Installing matplot++ dependencies ==="
sudo apt-get update -qq
sudo apt-get install -y --no-install-recommends \
    gnuplot \
    cmake \
    g++ \
    git \
    pkg-config \
    libpng-dev \
    libjpeg-dev \
    libtiff-dev \
    libfftw3-dev \
    libblas-dev \
    liblapack-dev \
    zlib1g-dev

LATEST_TAG="$(git ls-remote --tags --sort=-v:refname "${REPO}" 'v*' | head -1 | sed 's|.*/||')"
echo "=== Cloning matplot++ ${LATEST_TAG} ==="
rm -rf "${BUILD_DIR}"
git clone --depth 1 --branch "${LATEST_TAG}" "${REPO}" "${BUILD_DIR}"

echo "=== Building matplot++ ==="
cmake -S "${BUILD_DIR}" -B "${BUILD_DIR}/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17 \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    -DMATPLOTPP_BUILD_EXAMPLES=OFF \
    -DMATPLOTPP_BUILD_TESTS=OFF \
    -DMATPLOTPP_BUILD_EXPERIMENTAL_OPENGL_BACKEND=OFF \
    -DBUILD_SHARED_LIBS=ON

cmake --build "${BUILD_DIR}/build" --parallel "${JOBS}" --config Release

echo "=== Installing matplot++ to ${PREFIX} ==="
sudo cmake --install "${BUILD_DIR}/build"

echo "=== Updating linker cache ==="
sudo ldconfig

echo "=== Cleaning up ==="
rm -rf "${BUILD_DIR}"

echo "=== matplot++ installed successfully ==="
echo "  Headers:   ${PREFIX}/include/matplot/"
echo "  Libraries: ${PREFIX}/lib/"
echo "  Use in CMake: find_package(Matplot++ REQUIRED)"
