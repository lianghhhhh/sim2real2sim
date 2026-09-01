#!/bin/bash
set -e

# ─────────────────────────────────────────────────────────────
# 預設值（都可以用參數覆蓋，見下方 usage）
# ─────────────────────────────────────────────────────────────
ISAAC_SIM_PATH="/home/liang/isaac-sim/isaac-sim.streaming.sh"   # 請改成你的路徑
SIM_USD_PATH="/home/liang/sim2real2sim/car_sim.usd"
REAL_USD_PATH="/home/liang/sim2real2sim/car_real.usd"
OUTPUT_DIR="/workspaces/car_run_data/"
SIM_CSV_NAME="sim_data.csv"
REAL_CSV_NAME="real_data.csv"

# 執行 ROS 2 指令的 docker container
CONTAINER_NAME="e7724391f8ad"    # 可用 docker ps 查看目前的 CONTAINER ID 或 NAMES 欄位，兩者都能用
# 進 container 後、跑 ros2 run 前的初始化：直接用你在 container 裡設定好的 alias `r`
# （定義在 container 的 ~/.bashrc 裡，通常是 source ROS + workspace 的 setup.bash）

# 判斷 Isaac Sim 是否載入完成的關鍵字（不看時間，只看這行有沒有出現）
READY_PATTERN="Isaac Sim Full Streaming App is loaded"
LOAD_TIMEOUT=120        # 秒。超過這個時間還沒看到 READY_PATTERN，判定啟動失敗，中止整個程式
POLL_INTERVAL=2         # 秒。多久檢查一次 log
SIM_LOG="/tmp/isaac_sim_launch_$$.log"
PYTHON_SCRIPT_PATH="/home/liang/sim2real2sim/load_isaac.py"

usage() {
    echo "用法: $0 [-u|--usd <usd路徑>] [-c|--csv <csv檔名>] [-o|--outdir <輸出資料夾>]"
    echo "         [-n|--container <container id 或名稱>]"
    echo "         [-s|--skip-sim-launch]"
    echo ""
    echo "範例:"
    echo "  $0 --usd /home/liang/envs/gravel.usd --csv gravel_run.csv"
    echo "  $0 -u /home/liang/envs/ice.usd -c ice_run.csv"
    exit 1
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -u|--usd)
            USD_PATH="$2"; shift 2 ;;
        -c|--csv)
            CSV_NAME="$2"; shift 2 ;;
        -o|--outdir)
            OUTPUT_DIR="$2"; shift 2 ;;
        -n|--container)
            CONTAINER_NAME="$2"; shift 2 ;;
        -h|--help)
            usage ;;
        *)
            echo "未知參數: $1"
            usage ;;
    esac
done


# 檢查 container 是否存在且正在執行中，避免 docker exec 直接噴一堆看不懂的錯誤
if ! docker ps --format '{{.ID}} {{.Names}}' | grep -qE "(^| )${CONTAINER_NAME}( |$)"; then
    echo "錯誤：找不到執行中的 container「$CONTAINER_NAME」，請確認 docker ps 裡有這個 ID/名稱。"
    exit 1
fi

echo "輸出 CSV:       $OUTPUT_DIR/$CSV_NAME"
echo "目標 container: $CONTAINER_NAME"

SIM_PID=""

echo "USD 環境檔:     $USD_PATH"


# 1. 將獨立腳本傳給 --exec
SIM_ARGS=(--exec "$PYTHON_SCRIPT_PATH")

# 將你要執行的順序 (先 Real 再 Sim) 串成字串傳給 Python
export ISAAC_USD_LIST="${REAL_USD_PATH},${SIM_USD_PATH}"

SIM_ARGS=(--exec "$PYTHON_SCRIPT_PATH")
echo "啟動 Isaac Sim (log: $SIM_LOG)..."
"$ISAAC_SIM_PATH" "${SIM_ARGS[@]}" > "$SIM_LOG" 2>&1 &
SIM_PID=$!

# 2. 輪詢 log，找到 READY_PATTERN 才算真正載入完成；
#    process 提前掛掉，或超過 LOAD_TIMEOUT 秒還沒看到，都視為啟動失敗並中止整個程式。
echo "等待 Isaac Sim 載入中（最多等 ${LOAD_TIMEOUT} 秒）..."
elapsed=0
while true; do
    if grep -qF "$READY_PATTERN" "$SIM_LOG" 2>/dev/null; then
        echo "偵測到「$READY_PATTERN」，Isaac Sim 已就緒。"
        break
    fi

    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "錯誤：Isaac Sim process 提前結束，請檢查 log：$SIM_LOG"
        exit 1
    fi

    if (( elapsed >= LOAD_TIMEOUT )); then
        echo "錯誤：等待 Isaac Sim 載入超過 ${LOAD_TIMEOUT} 秒仍未看到就緒訊息，判定啟動失敗，中止整個程式。"
        echo "      可檢查 log：$SIM_LOG"
        kill "$SIM_PID" 2>/dev/null || true
        exit 1
    fi

    sleep "$POLL_INTERVAL"
    elapsed=$((elapsed + POLL_INTERVAL))
done


# ═════════════════════════════════════════════════════════════
# 第一階段：載入 Real USD 並收集資料
# ═════════════════════════════════════════════════════════════
echo -e "\n=== 準備執行第一階段 (Real): $REAL_USD_PATH ==="
echo "等待載入並 Play..."
while [ ! -f /tmp/isaac_ready_flag ]; do sleep 1; done
rm -f /tmp/isaac_ready_flag

echo "開始收集資料 ($REAL_CSV_NAME)..."
docker exec "$CONTAINER_NAME" bash -ic "
    r
    ros2 run calibrate_env_pkg calibrate_env_node --ros-args \
        -p csv_filename:=${REAL_CSV_NAME} \
        -p output_dir:=${OUTPUT_DIR}
"

echo "第一階段 (Real) 完畢，發送 Stop 訊號..."
touch /tmp/isaac_stop_flag
sleep 2 # 給模擬器一點緩衝時間去完全停止物理運算


# ═════════════════════════════════════════════════════════════
# 第二階段：載入 Sim USD 並收集資料
# ═════════════════════════════════════════════════════════════
echo -e "\n=== 準備執行第二階段 (Sim): $SIM_USD_PATH ==="
echo "等待載入並 Play..."
while [ ! -f /tmp/isaac_ready_flag ]; do sleep 1; done
rm -f /tmp/isaac_ready_flag

echo "開始收集資料 ($SIM_CSV_NAME)..."
docker exec "$CONTAINER_NAME" bash -ic "
    r
    ros2 run calibrate_env_pkg calibrate_env_node --ros-args \
        -p csv_filename:=${SIM_CSV_NAME} \
        -p output_dir:=${OUTPUT_DIR}
"

echo "第二階段 (Sim) 完畢，發送 Stop 訊號..."
touch /tmp/isaac_stop_flag

echo -e "\n所有腳本皆已執行完畢！"
echo "真實環境資料儲存於: $OUTPUT_DIR/$REAL_CSV_NAME"
echo "模擬環境資料儲存於: $OUTPUT_DIR/$SIM_CSV_NAME"
echo "Isaac Sim 仍保持開啟狀態待命。"