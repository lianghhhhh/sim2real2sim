"""把訓練好的模型交給 MPC (src/mlp_control_pkg): 複製權重、scaler 轉成 npz、更新 mpc_config.json。

    python export_for_mpc.py                      # 用 config.json 裡的 model_save_path / scaler_*_path
    python export_for_mpc.py --name mu05_k117     # 輸出檔名用這個 (預設 = model_save_path 的檔名)
    python export_for_mpc.py --no-update-config   # 只匯出檔案, 不改 mpc_config.json

輸出到 ../model/ (MPC 容器掛在 /workspaces/model):
    <name>.pth, scaler_<name>.npz (in_mean, in_scale, out_mean, out_scale)
scaler 存成 npz 而不是 pkl: MPC 容器裡的 sklearn 版本跟訓練環境不同, pickle 不保證讀得回來。
"""
import argparse
import json
import os
import shutil

import joblib
import numpy as np
import torch

from utils import SEQ_LENGTH, STATE_COLS, CONTROL_COLS, N_ABS_STATE

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--config', default=os.path.join(HERE, 'config.json'))
    ap.add_argument('--out-dir', default=os.path.join(HERE, '..', 'model'))
    ap.add_argument('--name', default=None, help='輸出檔名 (預設 = model_save_path 的檔名)')
    ap.add_argument('--no-update-config', action='store_true', help='不改 <out-dir>/mpc_config.json')
    args = ap.parse_args()

    with open(args.config) as f:
        cfg = json.load(f)
    rel = lambda p: p if os.path.isabs(p) else os.path.join(HERE, p)
    model_path = rel(cfg['model_save_path'])
    name = args.name or os.path.splitext(os.path.basename(model_path))[0]

    si = joblib.load(rel(cfg['scaler_in_path']))
    so = joblib.load(rel(cfg['scaler_out_path']))
    n_feat = len(CONTROL_COLS) + N_ABS_STATE + len(STATE_COLS)
    if si.mean_.shape != (n_feat,) or so.mean_.shape != (len(STATE_COLS),):
        raise SystemExit(f'scaler 維度 {si.mean_.shape}/{so.mean_.shape} 跟 utils.py 的格式 '
                         f'({n_feat},)/({len(STATE_COLS)},) 不合')
    sd = torch.load(model_path, map_location='cpu')
    w0 = sd['net.0.weight'].shape[1]
    if w0 != SEQ_LENGTH * n_feat:
        raise SystemExit(f'模型輸入 {w0} != SEQ_LENGTH*特徵 {SEQ_LENGTH * n_feat}')

    os.makedirs(args.out_dir, exist_ok=True)
    pth_name, npz_name = f'{name}.pth', f'scaler_{name}.npz'
    shutil.copy(model_path, os.path.join(args.out_dir, pth_name))
    np.savez(os.path.join(args.out_dir, npz_name), in_mean=si.mean_, in_scale=si.scale_,
             out_mean=so.mean_, out_scale=so.scale_)
    print(f'匯出 {model_path} -> {args.out_dir}/{pth_name}, {npz_name}')

    mpc_cfg = os.path.join(args.out_dir, 'mpc_config.json')
    if args.no_update_config:
        return
    if not os.path.isfile(mpc_cfg):
        print(f'{mpc_cfg} 不存在, 沒有更新 (參考 src/mlp_control_pkg 的說明建一個)')
        return
    with open(mpc_cfg) as f:
        mc = json.load(f)
    mc['model_path'], mc['scaler_path'] = pth_name, npz_name
    with open(mpc_cfg, 'w') as f:
        json.dump(mc, f, indent=4, ensure_ascii=False)
        f.write('\n')
    print(f'更新 {mpc_cfg}: model_path={pth_name}, scaler_path={npz_name}')


if __name__ == '__main__':
    main()
