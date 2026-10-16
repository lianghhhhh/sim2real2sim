#!/bin/bash
source "$(dirname "${BASH_SOURCE[0]}")/utils.sh"
main "$COMPOSE_DIR/docker-compose_rosbridge_server.yml"
