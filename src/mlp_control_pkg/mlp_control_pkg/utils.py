"""MPC 用的動態模型 (dynamic_predictor 訓練出來的 MLP) 與 acados solver。

模型格式 (要跟 dynamic_predictor/utils.py 一模一樣, 重新訓練改了這些就要一起改):
  * 狀態 STATE (NS = 10):  [x, y, theta, vx, vy, wz, wFL, wFR, wRL, wRR]
      x, y, theta = /odom 的世界座標位姿 (theta 不取負號, 在節點裡 unwrap 成連續的)
      vx, vy, wz  = /odom 的車體速度 (kros_car 車頭朝 +x, 不翻號)
      wXX         = /joint_states 的輪速 (rad/s)
  * 控制 (NU = 2): [左輪, 右輪] 輪速指令 (rad/s), 前後同側相同
  * 模型輸入: SEQ = 10 步, 每步 NF = 19 個特徵
        [控制 2, 絕對狀態 7 (vx, vy, wz, 4 輪速; 不含 x, y, theta), 跟前一步的差 10]
    「差」的位置分量是前一步車體座標下的 (前進, 左側) —— BODY_FRAME_CONVENTION 'A':
        forward = cos(th)*dx + sin(th)*dy,  lateral = -sin(th)*dx + cos(th)*dy
    window 第一步的差 = 0 (跟 make_delta_buffer 一樣)
  * 模型輸出: 下一步的差 (10), 同樣是車體座標
  * 時間步 DT = 0.05 s (模擬時間)

MPC 的狀態是「擴增狀態」: 最近 10 步的狀態 + 最近 9 步的控制
    x_aug = [STATE_{k-9}, ..., STATE_k, U_{k-9}, ..., U_{k-1}]   (NX = 118)
這一步的控制 u = U_k 是決策變數, 跟歷史湊成完整的 10 步 window。
"""
import json
import math
import os

import casadi as ca
import l4casadi as l4c
import numpy as np
import torch
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

from mlp_control_pkg.car_dynamic_model import CarDynamicModel

torch.backends.mkldnn.enabled = False

SEQ = 10                          # 模型看幾步歷史
NS = 10                           # 每步狀態維度
NU = 2                            # 控制維度 (左, 右)
N_ABS = NS - 3                    # 絕對狀態特徵 = 去掉 x, y, theta
NF = NU + N_ABS + NS              # 每步特徵 19
NX = SEQ * NS + (SEQ - 1) * NU    # 擴增狀態 118
CUR = slice((SEQ - 1) * NS, SEQ * NS)          # 目前狀態在擴增狀態裡的位置 (90:100)
PREV_U = slice(NX - NU, NX)                    # 上一步的控制 (116:118)
I_X, I_Y, I_TH, I_VX, I_VY, I_WZ = range(6)
RPM = 2.0 * math.pi / 60.0


def loadConfig(config_path=None):
    """讀 MPC 設定。路徑: 參數 > 環境變數 MPC_CONFIG_PATH > ./config.json。
    model_path / scaler_path 寫相對路徑時, 以設定檔所在的資料夾為準。"""
    path = config_path or os.environ.get('MPC_CONFIG_PATH') or 'config.json'
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f'找不到 MPC 設定檔 {path!r} (是資料夾或不存在)。'
            '用 MPC_CONFIG_PATH 指定, 例如 /workspaces/model/mpc_config.json')
    with open(path, 'r') as f:
        config = json.load(f)
    base = os.path.dirname(os.path.abspath(path))
    for key in ('model_path', 'scaler_path'):
        if key in config and not os.path.isabs(config[key]):
            config[key] = os.path.join(base, config[key])
    return config


def snap_rpm(w, step=1.0):
    """輪速 (rad/s) -> 最接近的整數 rpm (實體車 DDSM115 只收整數 rpm, 訓練資料的指令也是)。"""
    q = step * RPM
    return round(float(w) / q) * q


# ─────────────────────────────────────────────── 車體座標的差 (numpy 版)
def body_delta_np(s_from, s_to):
    d = np.asarray(s_to, dtype=float) - np.asarray(s_from, dtype=float)
    c, s = math.cos(s_from[I_TH]), math.sin(s_from[I_TH])
    d[I_X], d[I_Y] = c * d[I_X] + s * d[I_Y], -s * d[I_X] + c * d[I_Y]
    return d


def apply_body_delta_np(s_from, delta):
    out = np.asarray(s_from, dtype=float) + np.asarray(delta, dtype=float)
    c, s = math.cos(s_from[I_TH]), math.sin(s_from[I_TH])
    out[I_X] = s_from[I_X] + c * delta[I_X] - s * delta[I_Y]
    out[I_Y] = s_from[I_Y] + s * delta[I_X] + c * delta[I_Y]
    return out


def build_window_np(states, controls):
    """states (SEQ, NS), controls (SEQ, NU) -> 模型輸入 (SEQ*NF,), 未正規化。"""
    rows = []
    for i in range(SEQ):
        d = np.zeros(NS) if i == 0 else body_delta_np(states[i - 1], states[i])
        rows.append(np.concatenate([controls[i], states[i][3:], d]))
    return np.concatenate(rows)


# ─────────────────────────────────────────────── 車體座標的差 (CasADi 版)
def compute_body_delta(s_from, s_to):
    d = s_to - s_from
    c, s = ca.cos(s_from[I_TH]), ca.sin(s_from[I_TH])
    return ca.vertcat(c * d[I_X] + s * d[I_Y], -s * d[I_X] + c * d[I_Y], d[2:])


def apply_body_delta(s_from, delta):
    c, s = ca.cos(s_from[I_TH]), ca.sin(s_from[I_TH])
    return ca.vertcat(s_from[I_X] + c * delta[I_X] - s * delta[I_Y],
                      s_from[I_Y] + s * delta[I_X] + c * delta[I_Y],
                      s_from[2:] + delta[2:])


# ─────────────────────────────────────────────── 模型
class FastDynamicsModel(torch.nn.Module):
    """(B, SEQ*NF) 未正規化的 window -> (B, NS) 未正規化的差。正規化包在裡面。"""

    def __init__(self, base_model, in_mean, in_scale, out_mean, out_scale):
        super().__init__()
        self.base = base_model
        self.register_buffer('in_mean', torch.tensor(in_mean, dtype=torch.float32))
        self.register_buffer('in_scale', torch.tensor(in_scale, dtype=torch.float32))
        self.register_buffer('out_mean', torch.tensor(out_mean, dtype=torch.float32))
        self.register_buffer('out_scale', torch.tensor(out_scale, dtype=torch.float32))

    def forward(self, nn_input):
        B = nn_input.shape[0]
        x = (nn_input.view(B, SEQ, NF) - self.in_mean) / self.in_scale
        y = self.base(x.reshape(B, -1))
        return y * self.out_scale + self.out_mean


def loadTorchModel(config):
    """回傳 FastDynamicsModel (CPU, eval)。scaler 用 npz (in_mean, in_scale, out_mean, out_scale)。"""
    base = CarDynamicModel(input_size=SEQ * NF, output_size=NS)
    base.load_state_dict(torch.load(config['model_path'], map_location='cpu'))
    sc = np.load(config['scaler_path'])
    if sc['in_mean'].shape != (NF,) or sc['out_mean'].shape != (NS,):
        raise ValueError(f"scaler 維度 {sc['in_mean'].shape}/{sc['out_mean'].shape} "
                         f"跟 MPC 的格式 ({NF},)/({NS},) 不合 —— 模型是不是用別的 STATE_COLS 訓練的?")
    model = FastDynamicsModel(base, sc['in_mean'], sc['in_scale'], sc['out_mean'], sc['out_scale'])
    model.eval()
    return model


def loadModelFunc(config):
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    model = loadTorchModel(config)
    l4c_model = l4c.L4CasADi(model, device='cpu', name='car_model')
    nn_input_sym = ca.MX.sym('nn_input', SEQ * NF)
    delta_sym = l4c_model(nn_input_sym.T).T
    nn_model_func = ca.Function('nn_model_func', [nn_input_sym], [delta_sym])
    return nn_model_func, l4c_model.shared_lib_dir, l4c_model.name


# ─────────────────────────────────────────────── acados
def createAcadosSolver(nn_model_func, lib_dir, lib_name, config):
    N = int(config['N_steps'])
    dt = float(config['dt'])
    wmax = float(config.get('max_wheel_speed', 10.0))
    w = {'cte': 10.0, 'ate': 3.0, 'head': 8.0, 'speed': 10.0, 'u': 0.2, 'du': 0.5,
         'terminal_scale': 1.5}
    w.update(config.get('weights', {}))
    nrm = {'cte': 0.15, 'ate': 0.5, 'head': 0.2, 'speed': 0.3, 'du': 1.5}
    nrm.update(config.get('norms', {}))

    model = AcadosModel()
    model.name = 'car_model'
    x = ca.MX.sym('x', NX)
    u = ca.MX.sym('u', NU)
    p = ca.MX.sym('p', 5)              # [path x, path y, sin(path heading), cos(path heading), v_ref]

    # 1. 擴增狀態 -> 模型輸入
    X = [x[i * NS:(i + 1) * NS] for i in range(SEQ)]
    U = [x[SEQ * NS + i * NU: SEQ * NS + (i + 1) * NU] for i in range(SEQ - 1)] + [u]
    feats = []
    for i in range(SEQ):
        d = ca.MX.zeros(NS, 1) if i == 0 else compute_body_delta(X[i - 1], X[i])
        feats.append(ca.vertcat(U[i], X[i][3:], d))
    delta = nn_model_func(ca.vertcat(*feats))

    # 2. 下一步: 目前狀態 + 預測的差, window 往前移一格
    x_new = apply_body_delta(X[-1], delta)
    x_next = ca.vertcat(x[NS:SEQ * NS], x_new, x[SEQ * NS + NU:NX], u)

    model.x, model.u, model.p = x, u, p
    model.disc_dyn_expr = x_next

    ocp = AcadosOcp()
    ocp.model = model
    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = N * dt

    # 3. 成本 (NONLINEAR_LS)。路徑航向 psi: (sp, cp) = (sin psi, cos psi)
    cur = x[CUR]
    dx, dy = cur[I_X] - p[0], cur[I_Y] - p[1]
    sp, cp = p[2], p[3]
    along = dx * cp + dy * sp          # 沿路徑方向 (鬆)
    cross = -dx * sp + dy * cp         # 垂直路徑 (緊)
    th = cur[I_TH]
    state_err = ca.vertcat(cross, along, ca.sin(th) - sp, ca.cos(th) - cp)
    speed_err = cur[I_VX] - p[4]       # 車體前進速度 (有正負, 倒車不會被當成前進)
    du = u - x[PREV_U]

    ocp.cost.cost_type_0 = ocp.cost.cost_type = ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ocp.model.cost_y_expr_0 = ocp.model.cost_y_expr = ca.vertcat(state_err, speed_err, u, du)
    ocp.model.cost_y_expr_e = state_err

    norm = np.array([nrm['cte'], nrm['ate'], nrm['head'], nrm['head'], nrm['speed'],
                     wmax, wmax, nrm['du'], nrm['du']]) ** 2
    weights = np.array([w['cte'], w['ate'], w['head'], w['head'], w['speed'],
                        w['u'], w['u'], w['du'], w['du']])
    W = np.diag(weights / norm)
    W_e = np.diag(weights[:4] * w['terminal_scale'] / norm[:4])
    ocp.cost.W_0 = ocp.cost.W = W
    ocp.cost.W_e = W_e
    ocp.cost.yref_0 = ocp.cost.yref = np.zeros(9)
    ocp.cost.yref_e = np.zeros(4)

    # 4. 限制與 solver
    ocp.constraints.x0 = np.zeros(NX)
    ocp.parameter_values = np.zeros(5)
    ocp.constraints.lbu = -wmax * np.ones(NU)
    ocp.constraints.ubu = wmax * np.ones(NU)
    ocp.constraints.idxbu = np.arange(NU)

    ocp.solver_options.integrator_type = 'DISCRETE'
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.model_external_shared_lib_dir = lib_dir
    ocp.solver_options.model_external_shared_lib_name = lib_name
    ocp.solver_options.tol = 1e-3
    ocp.solver_options.qp_tol = 1e-3

    os.environ['CFLAGS'] = '-O3 -ffast-math -march=native'
    return AcadosOcpSolver(ocp, json_file='acados_ocp.json')
