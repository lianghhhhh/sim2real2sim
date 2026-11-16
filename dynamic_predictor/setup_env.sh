#!/usr/bin/env bash
# 建立訓練 dynamic_predictor 用的 conda 環境 (python 3.12 + PyTorch + requirements.txt), 最後自我檢查。
#
#   ./setup_env.sh                       # 環境名稱 dynamic_predictor, CUDA 版本自動偵測
#   ./setup_env.sh -n my_env             # 換環境名稱
#   ./setup_env.sh --cuda cpu            # 指定 PyTorch 版本: cpu | cu126 | cu128 | cu130
#   ./setup_env.sh --force               # 環境已經存在時先刪掉重建 (預設: 沿用, 只補裝/更新套件)
#
# 之後:  conda activate dynamic_predictor && python main.py
#
# 所有 pip 都用「環境裡的 python -m pip」執行: 環境沒有 python 時, 直接打 pip 會用到系統的 pip
# (Ubuntu 會回 externally-managed-environment 錯誤)。
set -euo pipefail

ENV_NAME=dynamic_predictor
PY_VER=3.12
CUDA=auto
FORCE=0
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() { sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }
while [[ $# -gt 0 ]]; do
  case "$1" in
    -n|--name) ENV_NAME="$2"; shift 2 ;;
    --cuda)    CUDA="$2"; shift 2 ;;
    --python)  PY_VER="$2"; shift 2 ;;
    --force)   FORCE=1; shift ;;
    -h|--help) usage 0 ;;
    *) echo "不認得的參數: $1"; usage 1 ;;
  esac
done

# ── conda
if ! command -v conda >/dev/null 2>&1; then
  for c in "$HOME/anaconda3" "$HOME/miniconda3" "$HOME/miniforge3"; do
    [[ -x "$c/bin/conda" ]] && export PATH="$c/bin:$PATH" && break
  done
fi
command -v conda >/dev/null 2>&1 || { echo "找不到 conda, 先安裝 Anaconda / Miniconda"; exit 1; }
CONDA_BASE="$(conda info --base)"

# ── PyTorch 要哪個 CUDA 版本
if [[ "$CUDA" == auto ]]; then
  if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
    drv=$(nvidia-smi | sed -n 's/.*CUDA Version: \([0-9]*\)\.\([0-9]*\).*/\1 \2/p' | head -1)
    read -r maj min <<<"$drv"
    if   (( maj >= 13 ));               then CUDA=cu130
    elif (( maj == 12 && min >= 8 ));   then CUDA=cu128
    elif (( maj == 12 && min >= 6 ));   then CUDA=cu126
    else echo "驅動支援的 CUDA $maj.$min 太舊, 改裝 CPU 版"; CUDA=cpu
    fi
    echo "偵測到 GPU 驅動支援 CUDA $maj.$min -> PyTorch $CUDA"
  else
    echo "沒有可用的 NVIDIA GPU -> PyTorch CPU 版"; CUDA=cpu
  fi
fi
case "$CUDA" in cpu|cu126|cu128|cu130) ;; *) echo "--cuda 要是 cpu|cu126|cu128|cu130, 收到 $CUDA"; exit 1 ;; esac
TORCH_INDEX="https://download.pytorch.org/whl/$CUDA"

# ── 建環境
ENV_PREFIX="$CONDA_BASE/envs/$ENV_NAME"
if [[ -d "$ENV_PREFIX" && $FORCE == 1 ]]; then
  echo "刪掉舊的環境 $ENV_NAME"
  conda env remove -y -n "$ENV_NAME"
fi
if [[ ! -x "$ENV_PREFIX/bin/python" ]]; then
  # 資料夾在但沒有 python (例如當初 conda create 沒指定 python) 也要重建
  [[ -d "$ENV_PREFIX" ]] && conda env remove -y -n "$ENV_NAME"
  echo "建立環境 $ENV_NAME (python $PY_VER)"
  conda create -y -n "$ENV_NAME" "python=$PY_VER" pip
else
  echo "環境 $ENV_NAME 已經存在, 沿用 (要重建加 --force)"
fi
PY="$ENV_PREFIX/bin/python"

# ── 套件
echo "安裝 PyTorch ($CUDA)"
"$PY" -m pip install --upgrade pip
"$PY" -m pip install --upgrade torch --index-url "$TORCH_INDEX"
echo "安裝 requirements.txt"
"$PY" -m pip install --upgrade -r "$HERE/requirements.txt"

# ── 自我檢查: 套件、GPU、模型能不能跑一步
echo "檢查環境"
cd "$HERE"
"$PY" - <<'EOF'
import sys
import joblib, matplotlib, numpy, pandas, sklearn, tensorboard, torch
from carDynamicModel import CarDynamicModel
from utils import SEQ_LENGTH, STATE_COLS, CONTROL_COLS, N_ABS_STATE

n_in = SEQ_LENGTH * (len(CONTROL_COLS) + N_ABS_STATE + len(STATE_COLS))
dev = 'cuda' if torch.cuda.is_available() else 'cpu'
m = CarDynamicModel(input_size=n_in, output_size=len(STATE_COLS)).to(dev)
y = m(torch.zeros(4, n_in, device=dev))
assert y.shape == (4, len(STATE_COLS))
print(f"python {sys.version.split()[0]} | torch {torch.__version__} (CUDA {torch.version.cuda}) | "
      f"numpy {numpy.__version__} | pandas {pandas.__version__} | scikit-learn {sklearn.__version__}")
print(f"GPU: {torch.cuda.get_device_name(0) if dev == 'cuda' else '沒有 (用 CPU 訓練會很慢)'}")
print(f"模型 {n_in} -> {len(STATE_COLS)} 在 {dev} 上跑一步 OK")
EOF

cat <<EOF

完成。使用:
  conda activate $ENV_NAME
  cd $HERE && python main.py
EOF
