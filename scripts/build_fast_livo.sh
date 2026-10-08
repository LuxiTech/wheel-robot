#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
revision=a09599a288c56b832114d7ad9e898248a4111d9e
source_dir=third_party/FAST-LIVO2-ROS2
if [[ ! -d "$source_dir/.git" ]]; then
  mkdir -p third_party
  git clone --no-checkout https://github.com/U-AMC/FAST-LIVO2-ROS2.git "$source_dir"
  git -C "$source_dir" checkout --detach "$revision"
fi
if [[ "$(git -C "$source_dir" rev-parse HEAD)" != "$revision" ]]; then
  echo "Expected FAST-LIVO2 revision $revision; use a separate clean checkout at that revision." >&2
  exit 1
fi
docker build --build-arg "BASE_IMAGE=${FAST_LIVO_BASE_IMAGE:-ros:humble-ros-base-jammy}" \
  -f docker/Dockerfile.fast-livo2 -t wheel-robot-fast-livo2:humble "$source_dir"
