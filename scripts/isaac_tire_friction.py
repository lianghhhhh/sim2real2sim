"""自訂輪胎摩擦力: 關掉 PhysX 的輪胎-地面摩擦, 每個 physics step 自己算 Coulomb 摩擦力加到輪子上。

為什麼 (2026-11-09 無頭 PhysX 實驗):
  PhysX 的接地摩擦做不出滑移轉向 (skid-steer) 邊走邊轉。左右輪速 7 / 10 rad/s 跑 6 s,
  實體車轉約 85 度 (理論 0.26 rad/s), PhysX 只有 0.004 ~ 0.03 rad/s。原因:
    1. 剛體車身 + 4 個剛體輪子是靜不定, 每個輪子分到的正向力 N 會在 0 ~ 60 N 亂跳 (TGS)
       或固定但不平均 (PGS)
    2. 摩擦力方向不跟著滑動方向 (TGS 差 10 ~ 53 度); 弧線行駛時沒有側滑卻有 ~μN 的側向力
    3. PGS 把 x / y 兩軸分開限制 (|F| 可以到 √2·μN), 快的那側輪子黏住地面, 車子直走
  改 μ、static friction、solver、頻率、迭代數、碰撞形狀都沒用。

做法 (每個 physics step 的「前」面執行一次, 跟 Action Graph 無關):
  * 輪子的碰撞體綁一個 μ = 0、combine = min 的物理材質 → PhysX 只負責支撐 (正向力), 不給摩擦
  * 每個輪子在接地點加摩擦力 (車體座標分成前後 x、側向 y 兩個分量):
        F_x = -μ·N·s_x / max(|s|, ε)        F_y = -μ·k_lat·N·s_y / max(|s|, ε)
      s = 接地點相對地面的滑動速度 = 輪心速度 - 輪子角速度·R·(車頭方向), 只取水平分量
      N = 靜態載重: 前後依重心做槓桿分配 (或用 FRONT_SHARE 指定)、左右平分 (不讀 PhysX 的接觸力)
      k_lat = 側向 ÷ 縱向摩擦比 (LAT_RATIO); 1.0 = 等向 Coulomb, 摩擦力正對滑動方向
      |s| < ε 時摩擦力線性變小 (避免在 0 附近來回抖)

要校正的參數 (sim2real2sim) 與它們主要影響什麼:
  MU          縱向摩擦大小 -> 同樣轉速下馬達要出多少力 (effort / 電流); 轉向幾乎不變
  LAT_RATIO   側向 ÷ 縱向 -> 轉多快 (原地自旋 T_eff、弧線轉速)。Coulomb 理論: 1.0 -> T_eff 0.53 m,
              1.17 -> 0.59 m (實體車 2026-11-09 量到 0.59)
  FRONT_SHARE 前輪分到的載重比例 (None = 依 USD 重心, 目前 0.40) -> 也會影響轉向; 可用磅秤直接量
  * 靜摩擦 (STICTION): 上面的 Coulomb 在 |s| < ε 時摩擦力跟滑動成正比 (像阻尼), 沒有東西能把車
    「停死」, 停車後會一直很慢地轉 (2026-11-10: ±7 rad/s 自旋後停 20 s, 還在 0.0002 ~ 0.002 rad/s)。
    所以滑動速度 < STICK_SPEED 時改用錨點彈簧 F = -k·z - c·s (z = 黏住後接地點累積的位移),
    只要 |F| 不超過靜摩擦上限 (μ·STATIC_RATIO·N, 側向照 LAT_RATIO 放大) 就一直黏住; 超過就回到
    Coulomb 滑動。弧線行駛時輪子滑動 >= 1 ~ 2 cm/s, 不會進入黏住, 轉向結果不變。
  * 輪子離地 (輪心高度 > 地面 + R + CONTACT_TOL) 就不加力, 所以懸空測試 (isaac_suspend_car.py)
    不受影響
  無頭實驗結果 (μ 0.5): 原地自旋與 0/6、3/6、5/6、7/10、9/10 弧線都跟 Coulomb 理論一致,
  直線速度正確; TGS+GPU 與 PGS+CPU 結果相同; ε 0.02 ~ 0.1 都穩。
  2026-11-10 用這個檔案本身測 (docker 無頭, GUI-like stepping, 模擬 actuator):
  7 / 10 rad/s 跑 6 s 轉 90.2 度 (實體車 80 ~ 90, 原本 PhysX 10 度), Stop / Play 後自動接回,
  close() 後回到 PhysX 原本的行為。
  注意: Coulomb 模型下「轉多少度」幾乎跟 μ 無關 (只看左右摩擦力的比例), μ 主要影響輪子扭矩
  (/joint_states effort) 和打滑的門檻。

**只寫在 session layer, 按 Save 也不會存進 USD; 每次開 Isaac / 重新開檔都要再跑一次。**

用法 (Isaac GUI):
  1. 開 kros_car.usd, **先不要按 Play** (材質要在 Play 前綁好; 重跑也可以, 會先關掉舊的)
  2. Window > Script Editor, 貼上這整個檔案的內容, Run。會印出每個輪子的 N
  3. 按 Play, 照常開 actuator_model.launch.py 和下指令。Stop / Play 會自己重新接上
  * 改參數 (不用重跑, 下一個 physics step 生效): 在 Script Editor 執行
        import builtins; t = builtins.kros_tire
        t.mu = 0.4; t.lat_ratio = 1.17; t.front_share = 0.43    # front_share = None 回到依重心
        t.stiction = False                                      # 關掉靜摩擦 (比較用)
  * 看目前狀態:  builtins.kros_tire.status()
  * 關掉、恢復 PhysX 原本的摩擦:  builtins.kros_tire.close()

數值穩定 (2026-11-10): |s| < ε 時摩擦力 = μN·s/ε, 每一步的「增益」G = μ·max(1, k_lat)·g·dt / ε。
  G > 2 會發散、> 1 會每步來回翻號, 摩擦力互相抵消, 車子像在冰上 (停車後 μ 1.0 在 60 Hz 下又轉了
  50 度, μ 2.0 轉了 137 度)。60 Hz、ε 0.05 時只有 μ·k_lat ≲ 0.5 是安全的。解法: 物理頻率提高到
  240 Hz (μ 1.0 實測: 轉向與理論一致, 停車 15 s 只動 0.003 度)。這個腳本另外有保護: G 超過 G_MAX
  時自動把 ε 放大到剛好穩定 (ε_eff, 會讓低滑動時的摩擦力稍微變小), 並印一次提醒。

限制:
  * N 是靜態的: 沒有加減速時的前後載重轉移, 也沒有側傾
  * 只適用平地 (地面高度 GROUND_Z, 法向量 +z)
  * μ ≥ 1.0 時輪子扭力會碰到 maxForce 2 N·m 的上限 (起步打滑、弧線轉得比理論少)
"""
import builtins

import numpy as np

CAR = '/kros_car'
ROOT_LINK = 'base_link'
WHEELS = ['new_wheel_v1_1', 'new_wheel_v1__1__1', 'new_wheel_v1__2__1', 'new_wheel_v1__2___1__1']
WHEEL_COLLIDER = 'collisions/mesh_0/cylinder'     # 在每個輪子底下
RADIUS = 0.051356        # m, 輪半徑 (跟 actuator / 定位一致)
MU = 0.5                 # 縱向 (前後) 摩擦係數 μ                     <- 要校正的參數
LAT_RATIO = 1.0          # 側向 μ ÷ 縱向 μ (1.0 = 等向)                  <- 要校正的參數
FRONT_SHARE = None       # 前輪分到的載重比例 0~1, None = 依 USD 重心算  <- 可校正 / 可用磅秤量
EPS = 0.05               # m/s, 滑動速度小於這個時摩擦力線性變小
GROUND_Z = 0.0           # m, 地面高度
CONTACT_TOL = 0.005      # m, 輪子底部離地超過這個就當作離地, 不加力
G_MAX = 1.5              # 數值穩定: 低滑動區每步增益的上限 (見檔頭), 超過就自動放大 ε
STICTION = True          # 停車時用錨點彈簧黏住 (見檔頭)
STATIC_RATIO = 1.0       # 靜摩擦上限 ÷ 動摩擦 (1.0 = 一樣)
STICK_SPEED = 0.003      # m/s, 滑動低於這個才會進入黏住
STICK_K = 3000.0         # N/m, 錨點彈簧 (60 Hz 下穩定: 每輪約 3 kg, ω·dt ≈ 0.5)
STICK_C = 60.0           # N·s/m, 錨點阻尼
ZERO_MAT = '/World/TireZeroFrictionMaterial'
HANDLE = 'kros_tire'     # builtins 裡的名字, 重跑時用來關掉舊的


def _quat_R(q):          # x y z w -> 3x3
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _np(x):
    return np.asarray(x.numpy() if hasattr(x, 'numpy') else x, dtype=np.float64)


class TireFriction:
    def __init__(self, stage, mu=MU, eps=EPS, lat_ratio=LAT_RATIO, front_share=FRONT_SHARE):
        import omni.physx
        import omni.timeline
        self.stage = stage
        self.mu, self.eps = mu, eps
        self.lat_ratio, self.front_share = lat_ratio, front_share
        self.front_cg = None                           # 依重心算的前輪比例 (Play 後才知道)
        self.sv = self.view = None
        self.wheels = []
        self.steps = 0
        self.last_F = np.zeros((len(WHEELS), 3))       # 最近一步加的摩擦力 (世界座標)
        self.last_s = np.zeros((len(WHEELS), 3))       # 算那個力時用的滑動速度 (世界座標)
        self.last_contact = [False] * len(WHEELS)
        self.stiction, self.static_ratio = STICTION, STATIC_RATIO
        self.z = np.zeros((len(WHEELS), 3))            # 黏住時接地點累積的位移 (世界座標)
        self.stuck = [False] * len(WHEELS)
        self.eps_eff = eps
        self._warned_eps = None
        self.center_local = self._wheel_centers()
        self._bind_zero_friction()
        px = omni.physx.get_physx_interface()
        self._step_sub = px.subscribe_physics_on_step_events(self._on_step, True, 0)   # pre-step
        self._tl_sub = omni.timeline.get_timeline_interface().get_timeline_event_stream() \
            .create_subscription_to_pop(self._on_timeline)

    # ---------------------------------------------------------------- USD (session layer)
    def _wheel_centers(self):
        """輪子碰撞圓柱中心在輪子剛體座標裡的位置 (輪子剛體的原點不一定在輪心)。"""
        from pxr import UsdGeom
        cache = UsdGeom.XformCache()
        out = {}
        for w in WHEELS:
            body = self.stage.GetPrimAtPath(f'{CAR}/{w}')
            cyl = self.stage.GetPrimAtPath(f'{CAR}/{w}/{WHEEL_COLLIDER}')
            if not body or not cyl:
                raise RuntimeError(f'找不到 {CAR}/{w}/{WHEEL_COLLIDER}')
            rel = cache.GetLocalToWorldTransform(cyl) * cache.GetLocalToWorldTransform(body).GetInverse()
            out[w] = np.array(rel.ExtractTranslation())
        return out

    def _bind_zero_friction(self):
        from pxr import Sdf, Usd, UsdShade
        st = self.stage
        with Usd.EditContext(st, st.GetSessionLayer()):
            mat = UsdShade.Material.Define(st, ZERO_MAT)
            p = mat.GetPrim()
            p.ApplyAPI('PhysicsMaterialAPI')
            p.ApplyAPI('PhysxMaterialAPI')
            p.CreateAttribute('physics:staticFriction', Sdf.ValueTypeNames.Float).Set(0.0)
            p.CreateAttribute('physics:dynamicFriction', Sdf.ValueTypeNames.Float).Set(0.0)
            p.CreateAttribute('physxMaterial:frictionCombineMode', Sdf.ValueTypeNames.Token).Set('min')
            for w in WHEELS:
                cp = st.GetPrimAtPath(f'{CAR}/{w}/collisions')
                if cp.IsInstanceable():          # 實例化的 prim 底下不能綁材質
                    cp.SetInstanceable(False)
                col = st.GetPrimAtPath(f'{CAR}/{w}/{WHEEL_COLLIDER}')
                api = UsdShade.MaterialBindingAPI.Apply(col)
                api.Bind(mat, UsdShade.Tokens.strongerThanDescendants, 'physics')

    def _unbind_zero_friction(self):
        """刪掉 session layer 裡這個腳本寫的東西 (輪子 collisions 底下的 over 和零摩擦材質)。"""
        from pxr import Sdf
        layer = self.stage.GetSessionLayer()
        edit = Sdf.BatchNamespaceEdit()
        for path in [f'{CAR}/{w}/collisions' for w in WHEELS] + [ZERO_MAT]:
            if layer.GetPrimAtPath(path):
                edit.Add(Sdf.NamespaceEdit.Remove(path))
        layer.Apply(edit)

    # ---------------------------------------------------------------- physics
    def _on_timeline(self, e):
        import omni.timeline
        if e.type == int(omni.timeline.TimelineEventType.STOP):
            self.sv = self.view = None          # Stop 之後 view 失效, 下一次 Play 重建

    def _build(self):
        import omni.physics.tensors as pt
        sv = pt.create_simulation_view('numpy')
        sv.set_subspace_roots('/')
        view = sv.create_articulation_view(f'{CAR}/{ROOT_LINK}')
        if view is None or view.count != 1:
            raise RuntimeError(f'{CAR}/{ROOT_LINK} 不是 articulation root')
        meta = view.shared_metatype
        links, dofs = list(meta.link_names), list(meta.dof_names)
        nl = view.max_links
        lt = _np(view.get_link_transforms()).reshape(nl, 7)
        masses = _np(view.get_masses()).reshape(nl)
        coms = _np(view.get_coms()).reshape(nl, 7)
        M = float(masses.sum())
        cg = sum(masses[i] * (lt[i, :3] + _quat_R(lt[i, 3:7]) @ coms[i, :3]) for i in range(nl)) / M
        ri = links.index(ROOT_LINK)
        Rr = _quat_R(lt[ri, 3:7])
        cg_b = Rr.T @ (cg - lt[ri, :3])
        wheels = []
        for w in WHEELS:
            li = links.index(w)
            pw = lt[li, :3] + _quat_R(lt[li, 3:7]) @ self.center_local[w]
            pb = Rr.T @ (pw - lt[ri, :3])
            jn = ('front' if pb[0] > 0 else 'rear') + '_' + ('left' if pb[1] > 0 else 'right') + '_joint'
            wheels.append(dict(w=w, link=li, dof=dofs.index(jn), joint=jn, x=pb[0]))
        # 靜態載重: 前後輪軸對重心做槓桿, 左右平分
        xf = np.mean([x['x'] for x in wheels if x['x'] > 0])
        xr = np.mean([x['x'] for x in wheels if x['x'] < 0])
        self.front_cg = float((cg_b[0] - xr) / (xf - xr))
        self.sv, self.view, self.links, self.nl, self.ri, self.wheels = sv, view, links, nl, ri, wheels
        self.M, self.cg_b = M, cg_b
        self.z[:] = 0.0
        self.stuck = [False] * len(WHEELS)
        self._update_loads()
        self._F = np.zeros((1, nl, 3), dtype=np.float32)
        self._P = np.zeros((1, nl, 3), dtype=np.float32)
        self._idx = np.array([0], dtype=np.int32)
        print(f'[tire] 接上 {CAR}: 總質量 {M:.2f} kg, 重心 (車體座標) x {cg_b[0]:+.3f} y {cg_b[1]:+.3f} m, '
              f'依重心前輪比例 {self.front_cg:.3f}; {self._params()}')
        for x in wheels:
            print(f'[tire]   {x["joint"]:18s} N = {x["N"]:.1f} N')

    def _params(self):
        fs = 'USD 重心' if self.front_share is None else f'{self.front_share:.3f}'
        st = f'靜摩擦 ×{self.static_ratio}' if self.stiction else '無靜摩擦'
        ep = f'ε {self.eps}' if self.eps_eff <= self.eps else f'ε {self.eps} (自動放大到 {self.eps_eff:.3f})'
        return f'μ {self.mu}, 側/縱 {self.lat_ratio}, 前輪載重比例 {fs}, {st}, {ep}'

    def _update_loads(self):
        """靜態載重: 前後照比例 (front_share 或依重心), 左右平分。參數改了下一步就生效。"""
        front = self.front_cg if self.front_share is None else float(self.front_share)
        for x in self.wheels:
            x['N'] = (front if x['x'] > 0 else 1 - front) / 2 * self.M * 9.81

    def _on_step(self, dt):
        try:
            if self.view is None or not self.sv.is_valid:
                self._build()
            self._apply(dt)
        except Exception as ex:               # 不要讓錯誤每一步都洗版: 印一次就停
            print(f'[tire] 錯誤, 已停用: {ex!r}')
            self._step_sub = None

    def _apply(self, dt=1.0 / 60):
        v, nl = self.view, self.nl
        lt = _np(v.get_link_transforms()).reshape(nl, 7)
        lv = _np(v.get_link_velocities()).reshape(nl, 6)
        dv = _np(v.get_dof_velocities()).reshape(-1)
        root = lt[self.ri]
        xb = _quat_R(root[3:7])[:, 0].copy()
        xb[2] = 0.0
        xb /= max(np.linalg.norm(xb), 1e-9)          # 車頭方向 (水平)
        yb = np.array([-xb[1], xb[0], 0.0])           # 車體左方 (水平)
        v0, w0 = lv[self.ri, :3], lv[self.ri, 3:]
        self._update_loads()
        # 數值穩定: μ 大或物理步長大時放大 ε (見檔頭)
        self.eps_eff = max(self.eps, self.mu * max(1.0, self.lat_ratio) * 9.81 * dt / G_MAX)
        if self.eps_eff > self.eps * 1.2 and self._warned_eps != (self.mu, self.lat_ratio, round(dt, 6)):
            self._warned_eps = (self.mu, self.lat_ratio, round(dt, 6))
            hz_need = int(np.ceil(self.mu * max(1.0, self.lat_ratio) * 9.81 / (G_MAX * self.eps)))
            print(f'[tire] 注意: μ {self.mu} × 側/縱 {self.lat_ratio} 在 {1 / dt:.0f} Hz 物理下不穩定, ε 自動放大到 '
                  f'{self.eps_eff:.3f} m/s (低滑動時摩擦力會偏小)。建議物理頻率 >= {hz_need} Hz')
        F, P = self._F, self._P
        F[:] = 0.0
        for k, x in enumerate(self.wheels):
            li = x['link']
            pc = lt[li, :3] + _quat_R(lt[li, 3:7]) @ self.center_local[x['w']]
            contact = pc[2] - RADIUS - GROUND_Z < CONTACT_TOL
            self.last_contact[k] = contact
            if not contact:
                self.last_F[k] = 0.0
                self.last_s[k] = 0.0
                self.z[k] = 0.0
                self.stuck[k] = False
                continue
            s = v0 + np.cross(w0, pc - root[:3]) - dv[x['dof']] * RADIUS * xb
            s[2] = 0.0
            self.last_s[k] = s
            sn = np.linalg.norm(s)
            f = -self.mu * x['N'] * ((s @ xb) * xb + self.lat_ratio * (s @ yb) * yb) / max(sn, self.eps_eff)
            if self.stiction:
                zt = self.z[k] + s * dt
                fs = -STICK_K * zt - STICK_C * s
                fs[2] = 0.0
                load = np.hypot(fs @ xb, (fs @ yb) / self.lat_ratio)     # 側向上限照 LAT_RATIO 放大
                if (self.stuck[k] or sn < STICK_SPEED) and load <= self.mu * self.static_ratio * x['N']:
                    f, self.z[k], self.stuck[k] = fs, zt, True
                else:
                    self.z[k], self.stuck[k] = 0.0, False
            F[0, li] = f
            P[0, li] = (pc[0], pc[1], GROUND_Z)
            self.last_F[k] = f
        v.apply_forces_and_torques_at_position(F, None, P, self._idx, True)
        self.steps += 1

    # ---------------------------------------------------------------- 使用者介面
    def status(self):
        if self.view is None:
            print(f'[tire] {self._params()}, 尚未 Play')
            return
        print(f'[tire] {self._params()}, 已跑 {self.steps} 步')
        for k, x in enumerate(self.wheels):
            f = self.last_F[k]
            print(f'[tire]   {x["joint"]:18s} 接地 {self.last_contact[k]!s:5s} 黏住 {self.stuck[k]!s:5s} '
                  f'N = {x["N"]:5.1f} N, F = ({f[0]:+6.2f}, {f[1]:+6.2f}) N (世界座標)')

    def close(self):
        self._step_sub = None
        self._tl_sub = None
        self.sv = self.view = None
        self._unbind_zero_friction()
        print('[tire] 已關閉, 輪子回到 PhysX 原本的摩擦 (下一次 Play 生效)')


def start(mu=MU, eps=EPS, lat_ratio=LAT_RATIO, front_share=FRONT_SHARE):
    import omni.usd
    old = getattr(builtins, HANDLE, None)
    if old is not None:
        old.close()
    tf = TireFriction(omni.usd.get_context().get_stage(), mu, eps, lat_ratio, front_share)
    setattr(builtins, HANDLE, tf)
    print(f'[tire] 已啟用 ({tf._params()}). 輪子 PhysX 摩擦 = 0, 按 Play 後每一步自己加摩擦力')
    return tf


try:
    import omni.usd  # noqa: F401
    start()
except ImportError:
    pass                         # 不是在 Isaac 裡 (例如被 import 來測試)
