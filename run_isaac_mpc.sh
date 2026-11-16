#!/bin/bash
# 單一 container: registry.screamtrumpet.csie.ncku.edu.tw/liang/sim2real2sim:v2
#   (= sim2real2sim:v1 + MPC 工具 + pandas, 見根目錄的 Dockerfile; 公開, 不用登入就能 pull)
# 取代 old_docker/run_isaac_gui.sh (ros2_node) + old_docker/docker/scripts/mpc.sh (mpc_isaac) 兩個 container。
#
# 用法 (可在任何目錄執行):
#   ./run_isaac_mpc.sh          啟動 container 並進入 bash (已在跑就直接進入)
#   ./run_isaac_mpc.sh shell    另開一個 bash 到正在跑的 container
#   IMAGE=... ./run_isaac_mpc.sh   換 image (例如本機剛 build 的 sim2real2sim:combined)
#
# 第一個終端機離開時 container 會被移除 (--rm), X11 轉發 (socat) 也會關掉。
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_IMAGE="${IMAGE:-registry.screamtrumpet.csie.ncku.edu.tw/liang/sim2real2sim:v2}"
DOCKER_NETWORK="compose_my_bridge_network"
XAUTH_FILE="/tmp/.docker.xauth"
NAME="ros2_node"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-82}"

running() {
    [ -n "$(docker ps -q --filter "name=^${NAME}$")" ]
}

if [ "$1" = "shell" ]; then
    if running; then
        exec docker exec -it "$NAME" bash
    fi
    echo "$NAME 沒在跑, 先執行 $0" >&2
    exit 1
elif [ -n "$1" ]; then
    echo "用法: $0 [shell]" >&2
    exit 1
fi

if running; then
    echo "$NAME 已經在跑, 直接進入它的 bash"
    exec docker exec -it "$NAME" bash
fi

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
    --name "$NAME" \
    --gpus all \
    --network "$DOCKER_NETWORK" \
    -v "$ROOT_DIR/src/:/workspaces/src" \
    -v "$ROOT_DIR/scripts/:/workspaces/scripts" \
    -v "$ROOT_DIR/car_run_data/:/workspaces/car_run_data" \
    -v "$ROOT_DIR/model/:/workspaces/model" \
    -v "$ROOT_DIR/../cameracalibration:/cameracalibration" \
    -v "$XAUTH_FILE:/tmp/.docker.xauth:ro" \
    --shm-size=2048m \
    -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e MPC_CONFIG_PATH=/workspaces/model/mpc_config.json \
    -e DISPLAY="${GATEWAY_IP}:${DISPLAY_NUM}.0" \
    -e XAUTHORITY=/tmp/.docker.xauth \
    -e QT_X11_NO_MITSHM=1 \
    "$DOCKER_IMAGE" \
    /bin/bash
