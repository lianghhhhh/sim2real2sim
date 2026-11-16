# dynamic_predictor

學 kros_car (四輪 skid-steer、輪速控制) 的**車輛動態模型**: 給最近 0.5 s 的指令與狀態, 預測下一個
0.05 s 的狀態變化。訓練好的模型給 `src/mlp_control_pkg` 的 MPC 用 (見「部署到 MPC」)。

```
car_run_data/*.csv ──load_segments()──> 0.05 s 等間隔的段落 ──> MLP (190 -> 10) ──> models/<名稱>.pth
 (collect_car_pkg 收的)                                                         └─ export_for_mpc.py ─> model/ (MPC)
```

## 模型格式

時間步 `DT = 0.05 s` (模擬時間), 看 `SEQ_LENGTH = 10` 步 (0.5 s) 的歷史。

**狀態** (`STATE_COLS`, 10 維): `[x, y, θ, vx, vy, wz, ωFL, ωFR, ωRL, ωRR]`

| | 單位 | 座標 / 來源 |
| --- | --- | --- |
| x, y | m | 世界座標, `/odom` (Isaac GT) |
| θ | rad | yaw, 由 `/odom` 的四元數算, **不取負號** (kros_car 車頭朝 +x) |
| vx, vy, wz | m/s, rad/s | **車體座標**: x 往前、y 往左、wz 逆時針為正, `/odom` twist |
| ωFL ... ωRR | rad/s | `/joint_states` 的輪速 |

**控制** (`CONTROL_COLS`, 2 維): 左、右輪速指令 (rad/s, `/joint_command`, actuator_model **之前**的值 =
實體車收到的指令)。前後同側的指令永遠一樣, 所以只用前輪那兩欄。

**輸入** = 10 步 × 19 個特徵 = 190, 每一步是:

| 區塊 | 維度 | 內容 |
| --- | --- | --- |
| 控制 | 2 | 這一步的左右輪速指令 |
| 絕對狀態 | 7 | vx, vy, wz, 4 個輪速 (`ABS_STATE_MASK`: 拿掉 x, y, θ —— 平地上動態跟在哪裡、朝哪個方向無關) |
| 跟前一步的差 | 10 | 位置的差轉到**前一步的車體座標** (前進, 側向); θ 的差 wrap 到 [−π, π); 其他是直接相減 |

**輸出** = 10: 從 window 最後一步到下一步的差, 格式跟上面「跟前一步的差」一樣。

要注意的細節 (自己寫部署端程式時很容易漏):
* 車體座標慣例 `BODY_FRAME_CONVENTION = 'A'`: `forward = cosθ·dx + sinθ·dy`, `lateral = −sinθ·dx + cosθ·dy`
  (`calibrate_body_frame.py` 可以用資料檢查)。
* **window 第一步的差 = 0** (`make_delta_buffer`, 沒有更早的資料)。rollout 評估與 MPC 都照這樣做。
* 輸入、輸出各自用 StandardScaler 正規化 (只用訓練資料 fit), 存成 `scaler_in_*.pkl` / `scaler_out_*.pkl`。

網路 (`carDynamicModel.py`): Linear 190→256 → LayerNorm → Softplus → 256→64 → LayerNorm → Softplus → 64→10。

## 資料

用 `collect_car_pkg` 在 Isaac 收 (見 `src/collect_car_pkg/README.md`), 存在 `../car_run_data/`。
`utils.load_segments()` 讀的時候:
* 只取 `phase == measure` 的列
* 狀態用各自的 stamp (`odom_stamp` / `wheel_stamp`) 線性內插, 指令「沿用前一筆」, 重新取樣到固定 0.05 s
  (Isaac 的 ROS 時鐘 60 Hz, 記錄 50 Hz, 直接把列當時間步會錯)
* 時間空檔 > 0.1 s、時間倒退 (Isaac Stop / Play)、位置跳 > 0.14 m 都會切成新的段落

檔名慣例: `<模式>_mu<μ×100>_k<k_lat×100>_s<seed>.csv`, 例如 `ou_mu035_k100_s2.csv`、`hr_mu035_k100_s0.csv`
(`hr` = hold_ramp)。同一個 seed 的指令序列完全一樣, 所以**同一組 train / val / test 裡 seed 不要重複**。
OU 範圍廣但幾乎不停車、很少直行; hold_ramp 有停車、直行和階躍 —— 兩種都要放。

**資料的模擬條件** (換條件就要重收、重訓):

| 項目 | mu035_k100 用的值 |
| --- | --- |
| USD | kros_car.usd, 開放場地 |
| 輪胎摩擦 | `scripts/isaac_tire_friction.py`, μ 0.35, LAT_RATIO 1.0 (**沒開這個 PhysX 原生摩擦幾乎轉不了彎**) |
| 致動器 | `car_teleop actuator_model`: 延遲 0.05 s, 斜坡 30 rad/s², 停車 τ 0.09 s |
| 輪速上限 | 10 rad/s (≈ 0.51 m/s) |


## 環境與執行

第一次先建環境 (python 3.12 + PyTorch + `requirements.txt`, CUDA 版本依驅動自動選, 最後會自我檢查):

```bash
./setup_env.sh                          # 環境名稱 dynamic_predictor; -n 換名稱, --cuda cpu|cu126|cu128|cu130, --force 重建
```

```bash
conda activate dynamic_predictor
cd dynamic_predictor
python main.py
#   1 = 訓練    2 = 閉迴路評估 (在 test_data_source 上 rollout, 存圖)
tensorboard --logdir logs               # 看訓練曲線
python export_for_mpc.py                # 匯出給 MPC (見下面)
```

## config.json

| 欄位 | 說明 |
| --- | --- |
| `data_dir` | 資料資料夾 (預設 `../car_run_data`) |
| `data_source` / `validation_source` | 訓練 / 驗證檔案清單。驗證用別的檔案 (別的 seed); 沒給 `validation_source` 才用每段最後 `validation_split` 的比例 |
| `test_data_source` | mode 2 評估用的檔案 (一個) |
| `log_dir`, `model_save_path`, `scaler_in_path`, `scaler_out_path`, `result_png` | 輸出的名稱, 換資料記得一起改, 不然會蓋掉舊的 |
| `batch_size`, `epochs`, `learning_rate` | 一般訓練參數。學習率在 val loss 5 個 epoch 沒進步時減半; 閉迴路驗證誤差 30 個 epoch (`patience`) 沒進步就提早停 |
| `use_multistep_training`, `multistep_k`, `multistep_k_start`, `multistep_k_warmup_epochs` | 多步訓練: 模型拿自己的預測往前推 k 步再算 loss。k 在前 `warmup` 個 epoch 從 `k_start` 線性加到 `multistep_k` |
| `noise_std_max`, `noise_warmup_epochs` | 訓練時在輸入的狀態加高斯雜訊 (正規化後的單位), 模擬自己預測的誤差 |
| `weight_xy`, `weight_theta` | 多步訓練裡的運動學一致性 loss: 預測的位移 / 角度變化要等於 (目前 + 預測的) 速度平均 × dt (梯形法) |
| `weight_slip`, `slip_wz_max` | 幾乎直行 (\|wz\| < `slip_wz_max`) 時懲罰預測的側向速度 vy (目前 0 = 關) |
| (選用) `patience`, `rollout_horizon`, `rollout_validation_horizon`, `rollout_validation_starts_per_segment`, `checkpoint_dir` | 預設 30, 50, = multistep_k, 3, `checkpoints` |

## 輸出

| 檔案 | 內容 |
| --- | --- |
| `models/<名稱>.pth` | 最後的模型 = 閉迴路驗證誤差最小的那個 epoch (不是最後一個 epoch) |
| `checkpoints/best.pth`, `epoch_N.pth` | 訓練中的 best 與每 10 epoch 的快照 (**每次訓練共用這個資料夾, 會被蓋掉**) |
| `scaler_in_*.pkl`, `scaler_out_*.pkl` | 正規化參數, 跟模型一起用 |
| `logs/<log_dir>/` | TensorBoard: Train / Validation (一步 loss), Multistep, RolloutValidation (驗證集推 `multistep_k` 步的平均位置誤差, m) |
| `<result_png>`, `*_rollout_trajectories.png` | mode 2 的誤差 vs 步數圖、軌跡圖 |

## 目前的結果: mu035_k100 (2026-11-12)

* 資料: OU seed 0–3 + hold_ramp seed 0–3, 共約 3.2 小時; train 6 檔、val 2 檔 (ou_s1、hr_s0)、test 另收的 s10
* 第 77 epoch 提早停, 用第 47 epoch (驗證集推 40 步的位置誤差 4.1 mm)
* 測試 (推 50 步 = 2.5 s): 位置誤差 hr_s10 1.0 cm / ou_s10 0.9 cm, 航向誤差約 0.024 rad,
  R² (50 步) vx 0.996–0.997, vy 0.987–0.993, wz 0.998
* MPC 在 Isaac 循跡 (輪胎模型開著, v 0.3 m/s): 平均橫向誤差 5 cm, 直線 ~3.7 cm, 急彎最大 22 cm

跟舊的 `car_data` 模型 (50 步誤差 10 cm 以上) **不能直接比**: 舊車速度 1.8 m/s (最高 5 m/s)、2.5 s 走 4.5 m,
新車 0.2 m/s、2.5 s 走 0.4 m。用相對誤差 (誤差 ÷ 2.5 s 位移) 兩者都約 2–2.5%。

## 部署到 MPC

```bash
python export_for_mpc.py           # -> ../model/<名稱>.pth, scaler_<名稱>.npz, 並更新 ../model/mpc_config.json
```
scaler 轉成 npz 是因為 MPC 容器 (`compose-mpc_isaac`) 的 sklearn 版本跟訓練環境不同。之後在 MPC 容器裡先跑
`ros2 run mlp_control_pkg mpc_offline_check` (方向、CasADi 與 PyTorch 是否一致、閉迴路) 再上 Isaac。

**改了模型格式** (`STATE_COLS`、`CONTROL_COLS`、`SEQ_LENGTH`、`ABS_STATE_MASK`、`BODY_FRAME_CONVENTION`、`DT`)
就要同步改 `src/mlp_control_pkg/mlp_control_pkg/utils.py` 開頭的常數與 `compute_body_delta` /
`apply_body_delta`, 還有 `car_state_node.py` 組狀態的順序。只重新訓練、格式不變就只要跑 export。

## 已知限制

* **μ 在模型的狀態裡幾乎看不出來**: actuator 的 30 rad/s² 斜坡讓縱向加速度 (< 0.7 m/s²) 遠小於 μg, 不會打滑;
  Coulomb 摩擦下轉向又幾乎跟 μ 無關。μ 只反映在輪子扭矩 (effort, CSV 有、模型輸入沒有)。
  sim2real2sim 要用模型誤差反推 μ 之前, 先用同 seed、不同 μ 的資料確認看得出差別。LAT_RATIO 則會反映在轉向上。
* 只在資料涵蓋的範圍內可靠: |vx| ≲ 0.45 m/s, |wz| ≲ 1.7 rad/s, 平地。
* 學的是模擬裡的車 (含 actuator_model 與輪胎模型), 不是實體車。
