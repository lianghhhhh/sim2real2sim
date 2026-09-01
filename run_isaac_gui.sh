#!/bin/bash
set -e

DOCKER_IMAGE="registry.screamtrumpet.csie.ncku.edu.tw/screamlab/sim2real2sim:v1"
DOCKER_NETWORK="compose_my_bridge_network"
XAUTH_FILE="/tmp/.docker.xauth"

if [ -z "$(docker network ls --filter name=$DOCKER_NETWORK --quiet)" ]; then
    docker network create "$DOCKER_NETWORK"
fi

DISPLAY_NUM=$(echo "$DISPLAY" | sed -E 's/.*:([0-9]+).*/\1/')
PORT=$((6000 + DISPLAY_NUM))
GATEWAY_IP=$(docker network inspect "$DOCKER_NETWORK" -f '{{range .IPAM.Config}}{{.Gateway}}{{end}}')

echo "Host DISPLAY: $DISPLAY"
echo "Container DISPLAY: ${GATEWAY_IP}:${DISPLAY_NUM}.0"
echo "Forwarding Host 127.0.0.1:${PORT} -> ${GATEWAY_IP}:${PORT}"

# 建立給 container 用的獨立 Xauthority
rm -f "$XAUTH_FILE"
touch "$XAUTH_FILE"
chmod 600 "$XAUTH_FILE"

# 先把目前 SSH X11 的 cookie 匯出到暫存檔
xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH_FILE" nmerge -

# 再補一份給 container 會用的 gateway 位址
COOKIE=$(xauth list "$DISPLAY" | awk 'NR==1 {print $3}')
xauth -f "$XAUTH_FILE" add "${GATEWAY_IP}:${DISPLAY_NUM}" MIT-MAGIC-COOKIE-1 "$COOKIE"
xauth -f "$XAUTH_FILE" add "${GATEWAY_IP}:${DISPLAY_NUM}.0" MIT-MAGIC-COOKIE-1 "$COOKIE"

# 清掉舊 socat
pkill -f "socat TCP4-LISTEN:${PORT}" 2>/dev/null || true

# 開 socat 背景轉發
socat TCP4-LISTEN:${PORT},fork,reuseaddr,bind=${GATEWAY_IP} TCP4:127.0.0.1:${PORT} &
SOCAT_PID=$!

cleanup() {
    kill $SOCAT_PID 2>/dev/null || true
    rm -f "$XAUTH_FILE"
}
trap cleanup EXIT

docker run -it --rm \
    --name ros2_node \
    --gpus all \
    --network "$DOCKER_NETWORK" \
    -v "$(pwd)/src/:/workspaces/src" \
    -v "$(pwd)/scripts/:/workspaces/scripts" \
    -v "$(pwd)/car_run_data/:/workspaces/car_run_data" \
    -v "$(pwd)/../cameracalibration:/cameracalibration" \
    -v "$XAUTH_FILE:/tmp/.docker.xauth:ro" \
    --shm-size=2048m \
    -e ROS_DOMAIN_ID=82 \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e DISPLAY="${GATEWAY_IP}:${DISPLAY_NUM}.0" \
    -e XAUTHORITY=/tmp/.docker.xauth \
    -e QT_X11_NO_MITSHM=1 \
    "$DOCKER_IMAGE" \
    /bin/bash
