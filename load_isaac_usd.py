import omni.usd
import omni.timeline
import asyncio
import os
import carb

# 取得 Bash 傳來的 USD 清單字串（以逗號分隔）
usd_list_str = os.environ.get("ISAAC_USD_LIST")

READY_FLAG_PATH = "/tmp/isaac_ready_flag"
STOP_FLAG_PATH = "/tmp/isaac_stop_flag"

if not usd_list_str:
    carb.log_error("[load_and_play] 錯誤：找不到 ISAAC_USD_LIST 環境變數！")
else:
    usd_paths = [p.strip() for p in usd_list_str.split(',') if p.strip()]

    async def run_multiple_usds():
        for i, usd_path in enumerate(usd_paths):
            carb.log_info(f"[load_and_play] === 準備載入: {usd_path} ===")
            
            # 清除殘留旗標
            if os.path.exists(READY_FLAG_PATH): os.remove(READY_FLAG_PATH)
            if os.path.exists(STOP_FLAG_PATH): os.remove(STOP_FLAG_PATH)
            
            # 1. 載入 USD 並按下 Play
            await omni.usd.get_context().open_stage_async(usd_path)
            omni.timeline.get_timeline_interface().play()
            
            # 2. 發送 Ready 訊號給 Bash
            with open(READY_FLAG_PATH, 'w') as f:
                f.write("ready")
            
            # 3. 監控 Stop 訊號
            while True:
                await asyncio.sleep(0.5)
                if os.path.exists(STOP_FLAG_PATH):
                    omni.timeline.get_timeline_interface().stop()
                    os.remove(STOP_FLAG_PATH)
                    carb.log_info("[load_and_play] 收到 Stop 訊號，準備切換。")
                    break # 跳出等待，準備載入下一個 USD
                    
        carb.log_info("[load_and_play] 兩個場景皆執行完畢，模擬器待命。")

    asyncio.ensure_future(run_multiple_usds())