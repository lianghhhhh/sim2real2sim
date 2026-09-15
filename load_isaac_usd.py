"""Isaac Sim 端的場景控制 (用 isaac-sim.streaming.sh --exec 載入)。

兩種模式:

1) 工作佇列模式 (預設, run_car.sh 用這個)
   bash 把一個工作寫到 /tmp/isaac_job.json (先寫暫存檔再 mv, 保證不會讀到一半):

       {"id": 3, "action": "run",
        "usd": "/home/liang/sim2real2sim/car_sim.usd",
        "friction": {"prim": "/Environment/groundCollider/PhysicsMaterial",
                     "static": 1.93, "dynamic": 1.93}}      # 省略 = 用 USD 原本的值

   action:
     run   載入 USD -> (設定地面摩擦) -> Play -> 寫 ready 旗標 -> 等 stop 旗標 -> Stop
     save  載入 USD -> 設定地面摩擦 -> 另存到 "save_as" -> 寫 ready 旗標 (不 Play)
     quit  不再接工作

   ready 旗標 (/tmp/isaac_ready_flag) 的內容是 JSON, 回報**實際生效**的值:
       {"id": 3, "ok": true, "usd": ..., "static": 1.93, "dynamic": 1.93, "error": ""}
   bash 靠 id 確認這是它剛送的那個工作, 靠 static/dynamic 知道 USD 原本的值
   (friction 省略時)。

   摩擦係數是**載入之後、Play 之前**直接改 stage 上的材質屬性, 不會動到原本的 USD 檔。

2) 清單模式 (舊流程): 設了環境變數 ISAAC_USD_LIST="a.usd,b.usd" 就依序載入,
   每個場景一樣用 ready / stop 旗標跟 bash 交接。
"""
import asyncio
import json
import os

import carb
import omni.kit.app
import omni.timeline
import omni.usd

_FLAG_DIR = os.environ.get("ISAAC_FLAG_DIR", "/tmp")    # run_car.sh 啟動時會傳進來
READY_FLAG_PATH = os.path.join(_FLAG_DIR, "isaac_ready_flag")
STOP_FLAG_PATH = os.path.join(_FLAG_DIR, "isaac_stop_flag")
JOB_PATH = os.path.join(_FLAG_DIR, "isaac_job.json")
DEFAULT_MATERIAL_PRIM = "/Environment/groundCollider/PhysicsMaterial"


def _log(msg):
    carb.log_info(f"[load_and_play] {msg}")
    print(f"[load_and_play] {msg}", flush=True)


def _write_ready(payload):
    tmp = READY_FLAG_PATH + ".tmp"
    with open(tmp, "w") as f:
        json.dump(payload, f)
    os.replace(tmp, READY_FLAG_PATH)


def _remove(path):
    if os.path.exists(path):
        os.remove(path)


async def _open(usd_path):
    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing():
        timeline.stop()
        await omni.kit.app.get_app().next_update_async()
    ok = await omni.usd.get_context().open_stage_async(usd_path)
    # 不同 Kit 版本回傳 bool 或 (bool, err)
    if isinstance(ok, tuple):
        ok = ok[0]
    if ok is False:
        raise RuntimeError(f"開不了 USD: {usd_path}")
    await omni.kit.app.get_app().next_update_async()
    return omni.usd.get_context().get_stage()


def _apply_friction(stage, friction):
    """設定地面材質的摩擦係數; friction=None 就只讀回來。回傳 (static, dynamic)。"""
    from pxr import UsdPhysics

    prim_path = (friction or {}).get("prim", DEFAULT_MATERIAL_PRIM)
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"找不到地面材質 prim: {prim_path}")
    api = UsdPhysics.MaterialAPI.Apply(prim)
    if friction:
        if friction.get("static") is not None:
            api.CreateStaticFrictionAttr().Set(float(friction["static"]))
        if friction.get("dynamic") is not None:
            api.CreateDynamicFrictionAttr().Set(float(friction["dynamic"]))
    s = api.GetStaticFrictionAttr().Get()
    d = api.GetDynamicFrictionAttr().Get()
    return (None if s is None else float(s)), (None if d is None else float(d))


async def _wait_stop():
    while not os.path.exists(STOP_FLAG_PATH):
        await asyncio.sleep(0.5)
    omni.timeline.get_timeline_interface().stop()
    _remove(STOP_FLAG_PATH)
    _log("收到 Stop 訊號")


async def run_job_queue():
    _remove(READY_FLAG_PATH)
    _remove(STOP_FLAG_PATH)
    _log(f"工作佇列模式: 等待 {JOB_PATH}")
    while True:
        if not os.path.exists(JOB_PATH):
            await asyncio.sleep(0.5)
            continue
        try:
            with open(JOB_PATH) as f:
                job = json.load(f)
        except (OSError, ValueError) as e:        # 萬一讀到寫一半的檔
            _log(f"工作檔讀不了 ({e}), 0.5 s 後重試")
            await asyncio.sleep(0.5)
            continue
        _remove(JOB_PATH)

        action = job.get("action", "run")
        if action == "quit":
            _log("收到 quit, 不再接工作")
            return
        reply = {"id": job.get("id"), "ok": False, "usd": job.get("usd"),
                 "static": None, "dynamic": None, "error": ""}
        try:
            _log(f"=== 工作 {job.get('id')}: {action} {job.get('usd')} "
                 f"friction={job.get('friction')} ===")
            stage = await _open(job["usd"])
            s, d = _apply_friction(stage, job.get("friction"))
            reply.update(static=s, dynamic=d)
            _log(f"地面摩擦 static {s}, dynamic {d}")
            if action == "save":
                result = await omni.usd.get_context().save_as_stage_async(job["save_as"])
                if isinstance(result, tuple) and result and result[0] is False:
                    raise RuntimeError(f"另存失敗: {result}")
                _log(f"已另存: {job['save_as']}")
                reply["ok"] = True
                _write_ready(reply)
                continue
            _remove(STOP_FLAG_PATH)
            omni.timeline.get_timeline_interface().play()
            reply["ok"] = True
            _write_ready(reply)
            await _wait_stop()
        except Exception as e:                    # 回報給 bash, 不要讓整個迴圈死掉
            reply["error"] = f"{type(e).__name__}: {e}"
            carb.log_error(f"[load_and_play] 工作失敗: {reply['error']}")
            _write_ready(reply)


async def run_usd_list(usd_paths):
    for usd_path in usd_paths:
        _log(f"=== 準備載入: {usd_path} ===")
        _remove(READY_FLAG_PATH)
        _remove(STOP_FLAG_PATH)
        await _open(usd_path)
        omni.timeline.get_timeline_interface().play()
        _write_ready({"id": None, "ok": True, "usd": usd_path})
        await _wait_stop()
    _log("清單裡的場景都執行完畢, 模擬器待命。")


usd_list_str = os.environ.get("ISAAC_USD_LIST")
if usd_list_str:
    asyncio.ensure_future(run_usd_list([p.strip() for p in usd_list_str.split(",") if p.strip()]))
else:
    asyncio.ensure_future(run_job_queue())
