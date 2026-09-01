import omni.usd
import omni.timeline
import asyncio
import os
import carb # Omniverse 內建的日誌系統

# 透過環境變數取得 Bash 傳遞過來的 USD 路徑
usd_path = os.environ.get("ISAAC_USD_PATH")

if not usd_path:
    carb.log_error("[load_and_play] 錯誤：找不到 ISAAC_USD_PATH 環境變數！")
else:
    async def load_and_play():
        carb.log_info(f"[load_and_play] 準備載入 USD: {usd_path}")
        
        # 使用非同步 API 開啟 USD
        await omni.usd.get_context().open_stage_async(usd_path)

        # --- 新增的修改摩擦力邏輯開始 ---
        carb.log_info("[load_and_play] 載入完成，準備修改物理材質參數...")
        
        # 取得目前的 USD Stage
        stage = omni.usd.get_context().get_stage()
        
        if stage:
            # PhysicsMaterial
            prim_path = "/Environment/groundCollider/PhysicsMaterial"
            material_prim = stage.GetPrimAtPath(prim_path)
            
            if material_prim.IsValid():
                # 在這裡設定你想要的摩擦力數值
                target_friction = 0.8
                
                # 更新屬性 (Attributes)
                material_prim.GetAttribute("physics:dynamicFriction").Set(target_friction)
                material_prim.GetAttribute("physics:staticFriction").Set(target_friction)
                
                carb.log_info(f"[load_and_play] 成功更新 {prim_path} 摩擦力！動摩擦: {target_friction}, 靜摩擦: {target_friction}")

                # 另存為新的 USD 檔案，避免覆蓋原檔
                new_usd_path = usd_path.replace(".usd", f"_{target_friction}.usd")
                carb.log_info(f"[load_and_play] 正在另存為新的 USD: {new_usd_path}")
                await omni.usd.get_context().save_as_stage_async(new_usd_path)
                carb.log_info("[load_and_play] 另存完成！")
            else:
                carb.log_warn(f"[load_and_play] 警告：找不到指定的 Prim {prim_path}")
        
        carb.log_info("[load_and_play] 開始模擬 (Play)")
        # 載入完成後，按下 Play
        omni.timeline.get_timeline_interface().play()

    # 將任務放入 Isaac Sim 的事件迴圈中執行
    asyncio.ensure_future(load_and_play())