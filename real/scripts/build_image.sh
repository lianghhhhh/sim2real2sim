#!/bin/bash
# Builds kros-car-new:latest from the car's driver image plus workspaces/src.
# Run it again after changing anything under workspaces/src.
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

docker build -t kros-car-new:latest "$REPO_ROOT" "$@"
