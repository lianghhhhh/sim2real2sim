#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_DIR="${ROOT_DIR}"
SKETCH_FILE="${SKETCH_DIR}/wildbot_wheelctrl.ino"

PORT="${PORT:-/dev/usb_wheel}"
ARDUINO_FQBN="${ARDUINO_FQBN:-esp32:esp32:esp32}"
ESP32_INDEX_URL="${ESP32_INDEX_URL:-https://espressif.github.io/arduino-esp32/package_esp32_index.json}"
INSTALLER_URL="${INSTALLER_URL:-https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh}"

ARDUINO_DATA_DIR="${ARDUINO_DATA_DIR:-${ROOT_DIR}/.arduino-data}"
ARDUINO_DOWNLOADS_DIR="${ARDUINO_DOWNLOADS_DIR:-${ROOT_DIR}/.arduino-downloads}"
ARDUINO_USER_DIR="${ARDUINO_USER_DIR:-${ROOT_DIR}/.arduino-user}"
BUILD_DIR="${BUILD_DIR:-${ROOT_DIR}/.arduino-build}"
CONFIG_FILE="${CONFIG_FILE:-${ROOT_DIR}/.arduino-cli.yaml}"
LOCAL_CLI_DIR="${LOCAL_CLI_DIR:-${ROOT_DIR}/bin}"

usage() {
  cat <<'EOF'
Usage: ./flash_esp32.sh

Builds and uploads wildbot_wheelctrl.ino with arduino-cli.

Environment overrides:
  PORT=/dev/usb_wheel              Upload port
  ARDUINO_FQBN=esp32:esp32:esp32   Board FQBN
  LOCAL_CLI_DIR=...                Where a repo-local arduino-cli binary is stored
  BUILD_DIR=...                    Build output directory

Example:
  PORT=/dev/ttyUSB0 ARDUINO_FQBN=esp32:esp32:esp32 ./flash_esp32.sh
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ ! -f "${SKETCH_FILE}" ]]; then
  echo "Sketch not found: ${SKETCH_FILE}" >&2
  exit 1
fi

ensure_prereqs() {
  local missing=()
  local tool
  for tool in curl grep mktemp rm sh; do
    if ! command -v "${tool}" >/dev/null 2>&1; then
      missing+=("${tool}")
    fi
  done

  if [[ ${#missing[@]} -gt 0 ]]; then
    echo "Missing required tools: ${missing[*]}" >&2
    exit 1
  fi
}

ensure_config() {
  mkdir -p "${ARDUINO_DATA_DIR}" "${ARDUINO_DOWNLOADS_DIR}" "${ARDUINO_USER_DIR}" "${BUILD_DIR}"

  cat > "${CONFIG_FILE}" <<EOF
board_manager:
  additional_urls:
    - ${ESP32_INDEX_URL}
directories:
  data: ${ARDUINO_DATA_DIR}
  downloads: ${ARDUINO_DOWNLOADS_DIR}
  user: ${ARDUINO_USER_DIR}
library:
  enable_unsafe_install: false
EOF
}

download_arduino_cli() {
  local tmp_dir

  tmp_dir="$(mktemp -d)"
  trap 'rm -rf "${tmp_dir}"' RETURN
  echo "Installing arduino-cli into ${LOCAL_CLI_DIR}"
  mkdir -p "${LOCAL_CLI_DIR}"
  curl -fsSL "${INSTALLER_URL}" -o "${tmp_dir}/install.sh"
  (
    cd "${ROOT_DIR}"
    BINDIR="${LOCAL_CLI_DIR}" sh "${tmp_dir}/install.sh"
  )
}

tmp_dir=$(mktemp -d)

resolve_arduino_cli() {
  if command -v arduino-cli >/dev/null 2>&1; then
    ARDUINO_CLI="$(command -v arduino-cli)"
    return
  fi

  ARDUINO_CLI="${LOCAL_CLI_DIR}/arduino-cli"
  if [[ ! -x "${ARDUINO_CLI}" ]]; then
    download_arduino_cli
  fi
}

install_dependencies() {
  "${ARDUINO_CLI}" core update-index --config-file "${CONFIG_FILE}"

  if ! "${ARDUINO_CLI}" core list --config-file "${CONFIG_FILE}" | grep -q '^esp32:esp32'; then
    "${ARDUINO_CLI}" core install esp32:esp32 --config-file "${CONFIG_FILE}"
  fi

  if ! "${ARDUINO_CLI}" lib list --config-file "${CONFIG_FILE}" | grep -q '^ArduinoJson'; then
    "${ARDUINO_CLI}" lib install ArduinoJson --config-file "${CONFIG_FILE}"
  fi
}

compile_sketch() {
  rm -rf "${BUILD_DIR}"
  mkdir -p "${BUILD_DIR}"
  "${ARDUINO_CLI}" compile \
    --fqbn "${ARDUINO_FQBN}" \
    --build-path "${BUILD_DIR}" \
    --config-file "${CONFIG_FILE}" \
    "${SKETCH_DIR}"
}

upload_sketch() {
  if [[ ! -e "${PORT}" ]]; then
    echo "Upload port not found: ${PORT}" >&2
    exit 1
  fi

  "${ARDUINO_CLI}" upload \
    --port "${PORT}" \
    --fqbn "${ARDUINO_FQBN}" \
    --input-dir "${BUILD_DIR}" \
    --config-file "${CONFIG_FILE}" \
    "${SKETCH_DIR}"
}

main() {
  ensure_prereqs
  ensure_config
  resolve_arduino_cli

  echo "Using arduino-cli: ${ARDUINO_CLI}"
  echo "Board FQBN: ${ARDUINO_FQBN}"
  echo "Upload port: ${PORT}"

  install_dependencies
  compile_sketch
  upload_sketch
}

main "$@"
