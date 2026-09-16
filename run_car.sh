#!/bin/bash
# ═════════════════════════════════════════════════════════════════════════════
#  sim2real 地面摩擦力自動校正
#
#    1. 量 real 一次 (或用 --reuse-real 沿用之前量好的)
#    2. 迴圈:  用目前的摩擦係數量 sim
#              -> scripts/friction_calib_step.py 比對 sim 與 real
#              -> 兩邊量不出差別就停; 否則換算出新的摩擦係數, 再量一輪 sim
#    3. 把收斂的摩擦係數另存成新的 USD (不動原本的 car_sim.usd)
#
#  每一輪的 CSV、比對結果 JSON、history.json 都在 car_run_data/<run 名稱>/。
#  Isaac 端是 load_isaac_usd.py 的工作佇列模式 (見那支檔案的說明)。
# ═════════════════════════════════════════════════════════════════════════════
set -e

# ─────────────────────────────────────────────────────────────
# 預設值（都可以用參數覆蓋，見下方 usage）
# ─────────────────────────────────────────────────────────────
ISAAC_SIM_PATH="${ISAAC_SIM_PATH:-/home/liang/isaac-sim/isaac-sim.streaming.sh}"   # 請改成你的路徑
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_USD_PATH="$REPO_DIR/car_sim.usd"
REAL_USD_PATH="$REPO_DIR/car_real.usd"
CALIBRATED_USD="$REPO_DIR/car_sim_calibrated.usd"
RUN_NAME="calib_$(date +%Y%m%d_%H%M%S)"
# 測試腳本用什麼狀態控制車子: sensor = 融合定位 + IMU (接近真車, 會一起開定位節點)
#                            gt     = Isaac /odom (舊流程)
STATE_SOURCE="sensor"

# ---- 校正迴圈 ----
MAX_ITER=6              # 最多量幾輪 sim
MAX_RETRY=1             # 同一組值資料不能用 (定位壞掉等) 時最多重量幾次
INIT_STATIC=""          # sim 第 0 輪的地面 staticFriction; 空 = 用 USD 裡原本的值
INIT_DYNAMIC=""
GAIN=1.0                # 更新步長, 1 = 直接跳到估計值; 震盪的話調小
TOL=0.05                # 收斂容許誤差: 預測的地面摩擦修正量 <= 這個值就停
WHEEL_MU=0.5            # 輪子材質 mu (car_*.usd 的輪子沒綁物理材質 -> PhysX 預設 0.5)
COMBINE="average"       # PhysX friction combine mode
# tied = 只解一個 mu, staticFriction 跟著 dynamicFriction (預設; 見 friction_calib_step.py
#        檔頭: 靜摩擦只有 B2 量得到, 敏感度只有 0.3, 分開解會生出假的靜動差)
# free = 用 B2 起轉 effort 分開解靜摩擦
STATIC_MODE="tied"
GROUND_PRIM="/Environment/groundCollider/PhysicsMaterial"
REUSE_REAL=""           # 已經量好的 real CSV; 給了就不重量 real
SKIP_ISAAC_LAUNCH=0     # 1 = Isaac 已經用 load_isaac_usd.py 開著了, 不要再開一個
PYTHON="${PYTHON:-python3}"   # 跑分析用的 python (要有 numpy / pandas)

# 執行 ROS 2 指令的 docker container
# run_isaac_gui.sh 裡是 `docker run --name ros2_node ...`，所以名稱固定是 ros2_node
CONTAINER_NAME="ros2_node"
DOCKER_IMAGE="registry.screamtrumpet.csie.ncku.edu.tw/screamlab/sim2real2sim:v1"
DOCKER_NETWORK="compose_my_bridge_network"
# 進 container 後、跑 ros2 launch 前的初始化：直接用你在 container 裡設定好的 alias `r`
# （定義在 container 的 ~/.bashrc 裡：source /workspaces/rebuild_colcon.rc）

# 判斷 Isaac Sim 是否載入完成的關鍵字（不看時間，只看這行有沒有出現）
READY_PATTERN="Isaac Sim Full Streaming App is loaded"
LOAD_TIMEOUT=120        # 秒。超過這個時間還沒看到 READY_PATTERN，判定啟動失敗
JOB_TIMEOUT=300         # 秒。Isaac 載入 / 設定一個場景的上限
POLL_INTERVAL=2
SIM_LOG="/tmp/isaac_sim_launch_$$.log"
PYTHON_SCRIPT_PATH="$REPO_DIR/load_isaac_usd.py"

# 跟 load_isaac_usd.py 交接用的檔案 (Isaac 由這支腳本啟動, 會繼承 ISAAC_FLAG_DIR)
export ISAAC_FLAG_DIR="${ISAAC_FLAG_DIR:-/tmp}"
READY_FLAG="$ISAAC_FLAG_DIR/isaac_ready_flag"
STOP_FLAG="$ISAAC_FLAG_DIR/isaac_stop_flag"
JOB_FILE="$ISAAC_FLAG_DIR/isaac_job.json"

usage() {
    cat <<EOF
用法: $0 [選項]

場景
  --sim-usd PATH          要校正的 sim USD            (預設 $SIM_USD_PATH)
  --real-usd PATH         當作 real 的 USD            (預設 $REAL_USD_PATH)
  --out-usd PATH          校正後另存的 USD            (預設 $CALIBRATED_USD)
  --reuse-real CSV        沿用量好的 real CSV (同目錄要有 *_imu.csv), 不重量 real
  --run-name NAME         輸出到 car_run_data/NAME/   (預設 calib_<時間>)

校正
  --max-iter N            最多量幾輪 sim              (預設 $MAX_ITER)
  --init-static MU        sim 第 0 輪的 staticFriction (預設 USD 原本的值)
  --init-dynamic MU       sim 第 0 輪的 dynamicFriction
  --tol T                 收斂容許誤差 (地面 mu)      (預設 $TOL)
  --gain G                更新步長                    (預設 $GAIN)
  --wheel-mu MU           輪子材質 mu                 (預設 $WHEEL_MU)
  --combine MODE          average|min|multiply|max    (預設 $COMBINE)

其他
  --state-source sensor|gt                            (預設 $STATE_SOURCE)
  --static-mode tied|free 靜摩擦跟著動摩擦 / 分開解    (預設 $STATIC_MODE)
  -n, --container NAME    ROS 2 container             (預設 $CONTAINER_NAME)
  --skip-isaac-launch     Isaac 已經開著 (用 load_isaac_usd.py), 不要再開

範例
  $0                                        # real 量一次, sim 從 USD 原本的值開始迭代
  $0 --reuse-real car_run_data/real_data.csv --init-dynamic 1.0 --init-static 1.0
EOF
    exit 1
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --sim-usd)          SIM_USD_PATH="$2"; shift 2 ;;
        --real-usd)         REAL_USD_PATH="$2"; shift 2 ;;
        --out-usd)          CALIBRATED_USD="$2"; shift 2 ;;
        --reuse-real)       REUSE_REAL="$2"; shift 2 ;;
        --run-name)         RUN_NAME="$2"; shift 2 ;;
        --max-iter)         MAX_ITER="$2"; shift 2 ;;
        --init-static)      INIT_STATIC="$2"; shift 2 ;;
        --init-dynamic)     INIT_DYNAMIC="$2"; shift 2 ;;
        --gain)             GAIN="$2"; shift 2 ;;
        --tol)              TOL="$2"; shift 2 ;;
        --wheel-mu)         WHEEL_MU="$2"; shift 2 ;;
        --combine)          COMBINE="$2"; shift 2 ;;
        --state-source)     STATE_SOURCE="$2"; shift 2 ;;
        --static-mode)      STATIC_MODE="$2"; shift 2 ;;
        -n|--container)     CONTAINER_NAME="$2"; shift 2 ;;
        --skip-isaac-launch) SKIP_ISAAC_LAUNCH=1; shift ;;
        -h|--help)          usage ;;
        *)  echo "未知參數: $1"; usage ;;
    esac
done

# 只給了靜 / 動其中一個 -> 另一個用同樣的值
[[ -n "$INIT_STATIC" && -z "$INIT_DYNAMIC" ]] && INIT_DYNAMIC="$INIT_STATIC"
[[ -n "$INIT_DYNAMIC" && -z "$INIT_STATIC" ]] && INIT_STATIC="$INIT_DYNAMIC"

# host 上的輸出目錄 <-> container 裡的同一個目錄 (car_run_data 掛在 /workspaces/car_run_data)
HOST_RUN_DIR="$REPO_DIR/car_run_data/$RUN_NAME"
CONT_RUN_DIR="/workspaces/car_run_data/$RUN_NAME"
HISTORY="$HOST_RUN_DIR/history.json"
mkdir -p "$HOST_RUN_DIR"

"$PYTHON" -c "import numpy, pandas" 2>/dev/null || {
    echo "錯誤：$PYTHON 沒有 numpy / pandas (分析要用)。用 PYTHON=/path/to/python3 $0 指定。"
    exit 1
}


# ─────────────────────────────────────────────────────────────
# ROS 2 container
# - 已經有人手動跑了 run_isaac_gui.sh → 直接沿用那個 container
# - 沒有 → 自己用背景模式 (-d) 起一個，跑完再自動關掉
# ─────────────────────────────────────────────────────────────
container_running() {
    docker ps --format '{{.ID}} {{.Names}}' | grep -qE "(^| )${CONTAINER_NAME}( |$)"
}

STARTED_CONTAINER=0
if ! container_running; then
    echo "找不到執行中的 container「$CONTAINER_NAME」，自動以背景模式啟動..."
    docker network inspect "$DOCKER_NETWORK" >/dev/null 2>&1 || docker network create "$DOCKER_NETWORK"
    docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true   # 清掉同名但已停止的殘留
    docker run -d --rm \
        --name "$CONTAINER_NAME" \
        --gpus all \
        --network "$DOCKER_NETWORK" \
        -v "$REPO_DIR/src/:/workspaces/src" \
        -v "$REPO_DIR/scripts/:/workspaces/scripts" \
        -v "$REPO_DIR/car_run_data/:/workspaces/car_run_data" \
        -v "$REPO_DIR/../cameracalibration:/cameracalibration" \
        --shm-size=2048m \
        -e ROS_DOMAIN_ID=82 \
        -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
        "$DOCKER_IMAGE" \
        sleep infinity >/dev/null
    STARTED_CONTAINER=1
    trap 'if (( STARTED_CONTAINER )); then docker stop "$CONTAINER_NAME" >/dev/null 2>&1 || true; fi' EXIT
    echo "container「$CONTAINER_NAME」已啟動（腳本結束時會自動關閉）。"
else
    echo "沿用已在執行中的 container「$CONTAINER_NAME」。"
fi


# ─────────────────────────────────────────────────────────────
# Isaac Sim (工作佇列模式)
# ─────────────────────────────────────────────────────────────
SIM_PID=""
if (( ! SKIP_ISAAC_LAUNCH )); then
    # 已經有 Isaac 開著的話再開一個, 兩個會在同一個 ROS domain 發同樣的 topic
    # (/clock /imu /odom ...), 收到的資料是兩台車混在一起。
    if [[ -z "${ISAAC_ALLOW_MULTIPLE:-}" ]] && pgrep -f -- "--exec .*load_isaac_usd.py" >/dev/null; then
        echo "錯誤：已經有 Isaac Sim 在跑 load_isaac_usd.py:"
        pgrep -fa -- "--exec .*load_isaac_usd.py" | sed 's/^/    /'
        echo "      先關掉它；或它是用新版 load_isaac_usd.py 開的話，加 --skip-isaac-launch 沿用。"
        exit 1
    fi
    rm -f "$READY_FLAG" "$STOP_FLAG" "$JOB_FILE"
    echo "啟動 Isaac Sim (log: $SIM_LOG)..."
    # ISAAC_USD_LIST 有設的話 load_isaac_usd.py 會跑舊的清單模式, 這裡一定要清掉
    env -u ISAAC_USD_LIST "$ISAAC_SIM_PATH" --exec "$PYTHON_SCRIPT_PATH" > "$SIM_LOG" 2>&1 &
    SIM_PID=$!

    echo "等待 Isaac Sim 載入中（最多等 ${LOAD_TIMEOUT} 秒）..."
    elapsed=0
    until grep -qF "$READY_PATTERN" "$SIM_LOG" 2>/dev/null; do
        if ! kill -0 "$SIM_PID" 2>/dev/null; then
            echo "錯誤：Isaac Sim process 提前結束，請檢查 log：$SIM_LOG"; exit 1
        fi
        if (( elapsed >= LOAD_TIMEOUT )); then
            echo "錯誤：等待 Isaac Sim 載入超過 ${LOAD_TIMEOUT} 秒，中止。log：$SIM_LOG"
            kill "$SIM_PID" 2>/dev/null || true; exit 1
        fi
        sleep "$POLL_INTERVAL"; elapsed=$((elapsed + POLL_INTERVAL))
    done
    echo "Isaac Sim 已就緒。"
fi

JOB_ID=0
# isaac_job <run|save> <usd> <static|""> <dynamic|""> [save_as]
# 成功後設定 REPLY_STATIC / REPLY_DYNAMIC = Isaac 回報實際生效的地面摩擦係數
isaac_job() {
    local action="$1" usd="$2" st="$3" dy="$4" save_as="${5:-}"
    JOB_ID=$((JOB_ID + 1))
    rm -f "$READY_FLAG"
    "$PYTHON" - "$JOB_FILE" "$JOB_ID" "$action" "$usd" "$st" "$dy" "$save_as" "$GROUND_PRIM" <<'PY'
import json, os, sys
path, jid, action, usd, st, dy, save_as, prim = sys.argv[1:9]
job = {"id": int(jid), "action": action, "usd": usd}
if st or dy:
    job["friction"] = {"prim": prim,
                       "static": float(st) if st else None,
                       "dynamic": float(dy) if dy else None}
if save_as:
    job["save_as"] = save_as
with open(path + ".tmp", "w") as f:
    json.dump(job, f)
os.replace(path + ".tmp", path)
PY
    local waited=0
    while true; do
        if [[ -f "$READY_FLAG" ]]; then
            local reply
            reply=$("$PYTHON" - "$READY_FLAG" "$JOB_ID" <<'PY'
import json, sys
r = json.load(open(sys.argv[1]))
if r.get("id") != int(sys.argv[2]):
    print("STALE")
elif not r.get("ok"):
    print("ERROR " + str(r.get("error")))
else:
    print(f"OK {r.get('static')} {r.get('dynamic')}")
PY
)
            case "$reply" in
                OK*)    read -r _ REPLY_STATIC REPLY_DYNAMIC <<< "$reply"
                        rm -f "$READY_FLAG"; return 0 ;;
                ERROR*) echo "錯誤：Isaac 工作失敗: ${reply#ERROR }"; rm -f "$READY_FLAG"; return 1 ;;
            esac
        fi
        if [[ -n "$SIM_PID" ]] && ! kill -0 "$SIM_PID" 2>/dev/null; then
            echo "錯誤：Isaac Sim process 結束了，log：$SIM_LOG"; return 1
        fi
        if (( waited >= JOB_TIMEOUT )); then
            echo "錯誤：Isaac 工作 $JOB_ID 超過 ${JOB_TIMEOUT} 秒沒回應"
            echo "      (Isaac 有沒有用 load_isaac_usd.py 的工作佇列模式開著?)"; return 1
        fi
        sleep 1; waited=$((waited + 1))
    done
}

isaac_stop() {
    touch "$STOP_FLAG"
    local waited=0
    while [[ -f "$STOP_FLAG" ]] && (( waited < 30 )); do sleep 1; waited=$((waited + 1)); done
    sleep 2   # 給模擬器一點緩衝時間去完全停止物理運算
}

# measure <csv 檔名>: 在目前 Play 的場景跑一次摩擦力測試腳本
measure() {
    local csv="$1"
    echo "開始收集資料 ($HOST_RUN_DIR/$csv)..."
    docker exec "$CONTAINER_NAME" bash -ic "
        r
        ros2 launch bringup_pkg friction_test.launch.py \
            state_source:=${STATE_SOURCE} \
            csv_filename:=${csv} \
            output_dir:=${CONT_RUN_DIR}
    " || { echo "警告：測試腳本以非 0 結束"; }
    local lines=0
    [[ -f "$HOST_RUN_DIR/$csv" ]] && lines=$(wc -l < "$HOST_RUN_DIR/$csv")
    if (( lines < 200 )); then
        echo "錯誤：$csv 只有 $lines 列 (測試腳本可能在開始前就放棄了, 看上面 control_car_node 的 log)"
        return 1
    fi
}


# ═════════════════════════════════════════════════════════════
# 第一階段：Real
# ═════════════════════════════════════════════════════════════
REAL_CSV="$HOST_RUN_DIR/real_data.csv"
if [[ -n "$REUSE_REAL" ]]; then
    echo -e "\n=== Real: 沿用 $REUSE_REAL ==="
    cp "$REUSE_REAL" "$REAL_CSV"
    cp "${REUSE_REAL%.csv}_imu.csv" "$HOST_RUN_DIR/real_data_imu.csv" 2>/dev/null \
        || echo "警告：找不到 ${REUSE_REAL%.csv}_imu.csv, 分析會退回 GT / 位姿 (準度較差)"
else
    echo -e "\n=== Real: $REAL_USD_PATH ==="
    isaac_job run "$REAL_USD_PATH" "" ""
    echo "Real USD 地面摩擦 (只是記錄, 校正不會用到): static $REPLY_STATIC, dynamic $REPLY_DYNAMIC"
    measure real_data.csv
    isaac_stop
fi


# ═════════════════════════════════════════════════════════════
# 第二階段：Sim 校正迴圈
# ═════════════════════════════════════════════════════════════
CUR_STATIC="$INIT_STATIC"
CUR_DYNAMIC="$INIT_DYNAMIC"
ITER=0
RETRY=0
STATUS=""
MAX_ROUNDS=$(( MAX_ITER * (MAX_RETRY + 1) ))
ROUND=0

while (( ROUND < MAX_ROUNDS )); do
    ROUND=$((ROUND + 1))
    CSV="sim_iter${ITER}.csv"
    (( RETRY > 0 )) && CSV="sim_iter${ITER}_retry${RETRY}.csv"

    echo -e "\n=== Sim 第 ${ITER} 輪$( (( RETRY > 0 )) && echo " (重量 $RETRY)"): " \
            "static ${CUR_STATIC:-<USD 原值>}, dynamic ${CUR_DYNAMIC:-<USD 原值>} ==="
    isaac_job run "$SIM_USD_PATH" "$CUR_STATIC" "$CUR_DYNAMIC"
    CUR_STATIC="$REPLY_STATIC"; CUR_DYNAMIC="$REPLY_DYNAMIC"
    echo "Isaac 回報實際生效: static $CUR_STATIC, dynamic $CUR_DYNAMIC"
    if ! [[ "$CUR_STATIC" =~ ^[0-9.eE+-]+$ && "$CUR_DYNAMIC" =~ ^[0-9.eE+-]+$ ]]; then
        echo "錯誤：讀不到 $GROUND_PRIM 的摩擦係數 (USD 沒設?)，用 --init-static / --init-dynamic 指定。"
        isaac_stop; STATUS="failed"; break
    fi

    if ! measure "$CSV"; then
        isaac_stop
        if (( RETRY < MAX_RETRY )); then RETRY=$((RETRY + 1)); continue; fi
        echo "錯誤：同一組值連續量測失敗，中止校正。"; STATUS="failed"; break
    fi
    isaac_stop

    STEP_OUT=$("$PYTHON" "$REPO_DIR/scripts/friction_calib_step.py" \
        --real "$REAL_CSV" --sim "$HOST_RUN_DIR/$CSV" \
        --static "$CUR_STATIC" --dynamic "$CUR_DYNAMIC" \
        --iter "$ITER" --max-iter "$MAX_ITER" --max-retry "$MAX_RETRY" \
        --history "$HISTORY" --gain "$GAIN" --tol "$TOL" \
        --wheel-mu "$WHEEL_MU" --combine "$COMBINE" --static-mode "$STATIC_MODE" \
        | tee /dev/stderr | grep '^STATUS=' | tail -1)
    if [[ -z "$STEP_OUT" ]]; then
        echo "錯誤：friction_calib_step.py 沒有輸出結果，中止。"; STATUS="failed"; break
    fi
    eval "$STEP_OUT"      # -> STATUS NEXT_STATIC NEXT_DYNAMIC

    case "$STATUS" in
        continue)  CUR_STATIC="$NEXT_STATIC"; CUR_DYNAMIC="$NEXT_DYNAMIC"
                   ITER=$((ITER + 1)); RETRY=0 ;;
        retry)     RETRY=$((RETRY + 1)) ;;
        converged|max_iter)
                   CUR_STATIC="$NEXT_STATIC"; CUR_DYNAMIC="$NEXT_DYNAMIC"; break ;;
        *)         break ;;
    esac
done


# ═════════════════════════════════════════════════════════════
# 收尾
# ═════════════════════════════════════════════════════════════
echo -e "\n══════════════════════════════════════════════════════════════════"
"$PYTHON" - "$HISTORY" <<'PY'
import json, os, sys
if not os.path.exists(sys.argv[1]):
    print("(沒有任何一輪完成比對)"); sys.exit()
h = json.load(open(sys.argv[1]))
print(f"{'輪':>3} {'static':>8} {'dynamic':>8} {'比值':>7}  {'估計 static':>14} {'估計 dynamic':>14}  狀態")
for it in h["iterations"]:
    r = it.get("ratio")
    e = it.get("estimate") or {}
    fmt = lambda t: (f"{e[t]['value']:.3f}±{e[t]['sigma']:.3f}" if t in e else "-")
    print(f"{it['iter']:>3} {it['static']:>8.3f} {it['dynamic']:>8.3f} "
          f"{(f'{r:.3f}' if r else '-'):>7}  {fmt('static'):>14} {fmt('dynamic'):>14}  {it['status']}")
    print(f"      {it.get('reason', '')}")
PY
echo "資料與比對結果: $HOST_RUN_DIR"

case "$STATUS" in
    converged|max_iter)
        [[ "$STATUS" == "max_iter" ]] && echo "注意：沒有收斂，用的是最接近 real 的那一輪。"
        echo "sim 地面摩擦係數: static $CUR_STATIC, dynamic $CUR_DYNAMIC"
        if isaac_job save "$SIM_USD_PATH" "$CUR_STATIC" "$CUR_DYNAMIC" "$CALIBRATED_USD"; then
            echo "已另存校正後的 USD: $CALIBRATED_USD"
        else
            echo "另存 USD 失敗；請手動把 $GROUND_PRIM 的 staticFriction / dynamicFriction 設成上面的值。"
        fi
        ;;
    *)
        echo "校正沒有完成 (STATUS=${STATUS:-未開始})，沒有另存 USD。看上面的訊息與 $HISTORY。"
        exit 1 ;;
esac
echo "Isaac Sim 仍保持開啟狀態待命。"
