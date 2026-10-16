#!/bin/bash

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_DIR="$REPO_ROOT/docker/compose"

# All compose files in this repo intentionally share one project (so they
# share my_bridge_network). Suppress the orphan-container warnings that
# arise when each script only declares a subset of services.
export COMPOSE_IGNORE_ORPHANS=True

cleanup() {
    local scripts=("$@")
    echo "Shutting down docker compose services..."
    for script in "${scripts[@]}"; do
        echo "Stopping services for $script..."
        $DOCKER_COMPOSE_COMMAND -f "$script" down --timeout 0
    done
    exit 0
}

main() {
    SCRIPTS=()

    if docker compose version &> /dev/null; then
        DOCKER_COMPOSE_COMMAND="docker compose"
    elif command -v docker-compose &> /dev/null; then
        DOCKER_COMPOSE_COMMAND="docker-compose"
    else
        echo "Neither 'docker compose' nor 'docker-compose' is installed."
        exit 1
    fi

    for script in "$@"; do
        if [[ ! -f "$script" ]]; then
            echo "Error: Compose file not found: $script" >&2
            exit 1
        fi
        echo "Starting services for $script..."
        $DOCKER_COMPOSE_COMMAND -f "$script" up -d
        SCRIPTS+=("$script")
        $DOCKER_COMPOSE_COMMAND -f "$script" logs -f &
    done

    trap 'cleanup "${SCRIPTS[@]}"' SIGINT

    wait
}
