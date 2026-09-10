import pandas as pd
import matplotlib.pyplot as plt

# ==========================================
# 1. CSV 檔案路徑
# ==========================================
csv_file = "car_run_data/sim_data.csv"   # 改成你的 CSV 檔案名稱

df = pd.read_csv(csv_file)

# ==========================================
# 2. 檢查資料
# ==========================================
print("CSV columns:")
print(df.columns.tolist())

print("\nFirst 5 rows:")
print(df.head())

# ==========================================
# 3. 過濾掉三個定位來源缺值的列
# ==========================================
# collect_data_node 是各 topic 各自非同步發布; car_cam_calib 開機要等
# car_inference 的 YOLO 模型載入完才有第一筆 /camera/pose, 在那之前
# yolo_x/yolo_y 整段是 NaN (不是壞掉, 是還沒收到, 不會再斷斷續續出現)。
# 三個定位來源要同一列都有值才能公平比較, 所以只留三者都有值的列。
position_cols = ["car_position_x", "car_position_y",
                  "yolo_x", "yolo_y",
                  "loc_car_position_x", "loc_car_position_y"]
before = len(df)
df = df.dropna(subset=position_cols).reset_index(drop=True)
print(f"\n過濾 NaN: {before} -> {len(df)} 列 (丟掉 {before - len(df)} 列)")

# ==========================================
# 4. 計算 X / Y 誤差
# ==========================================
df["x_yolo_error"] = df["car_position_x"] - df["yolo_x"]
df["y_yolo_error"] = df["car_position_y"] - df["yolo_y"]
df["x_loc_error"] = df["car_position_x"] - df["loc_car_position_x"]
df["y_loc_error"] = df["car_position_y"] - df["loc_car_position_y"]
df['yolo_loc_x_error'] = df['yolo_x'] - df['loc_car_position_x']
df['yolo_loc_y_error'] = df['yolo_y'] - df['loc_car_position_y']

# 距離誤差 (歐氏距離) 取代分開看 X/Y —— 兩個方向的誤差合成一個數字,
# 比較不同定位方法時更直觀。
df["dist_yolo_error"] = (df["x_yolo_error"] ** 2 + df["y_yolo_error"] ** 2) ** 0.5
df["dist_loc_error"] = (df["x_loc_error"] ** 2 + df["y_loc_error"] ** 2) ** 0.5
df["dist_yolo_loc_error"] = (df["yolo_loc_x_error"] ** 2 + df["yolo_loc_y_error"] ** 2) ** 0.5

# ==========================================
# 5. 畫圖只取「過濾後」資料的中間 100 筆
# ==========================================
# 統計數字 (第 10 節) 仍然用全部過濾後的資料算, 只有圖片畫這個窗口。
N_PLOT = 100
mid = len(df) // 2
start = max(0, mid - N_PLOT // 2)
end = min(len(df), start + N_PLOT)
df_plot = df.iloc[start:end].copy()
# x 軸改成相對這個視窗起點的秒數 —— 原始 timestamp 是 unix epoch (~1.79e9),
# 100 筆資料畫出來的刻度會全部疊在一起, 看不出時間軸的變化。
df_plot["t_rel"] = df_plot["timestamp"] - df_plot["timestamp"].iloc[0]
print(f"畫圖只取中間 100 筆: row {start} ~ {end - 1} (共 {len(df_plot)} 筆)")

# ==========================================
# 6. Plot 1: X 座標比較
# ==========================================
plt.figure(figsize=(12, 5))

plt.plot(
    df_plot["t_rel"],
    df_plot["car_position_x"],
    label="Car Position X"
)

plt.plot(
    df_plot["t_rel"],
    df_plot["yolo_x"],
    label="YOLO X"
)

plt.plot(
    df_plot["t_rel"],
    df_plot["loc_car_position_x"],
    label="loc Car Position X"
)

plt.xlabel("Time since window start (s)")
plt.ylabel("X Position")
plt.title("Car Position X vs YOLO X vs loc Car Position X (middle 100 rows)")
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig("car_run_data/position_x_comparison.png")  # 儲存圖表為 PNG 檔案
plt.show()

# ==========================================
# 7. Plot 2: Y 座標比較
# ==========================================
plt.figure(figsize=(12, 5))

plt.plot(
    df_plot["t_rel"],
    df_plot["car_position_y"],
    label="Car Position Y"
)

plt.plot(
    df_plot["t_rel"],
    df_plot["yolo_y"],
    label="YOLO Y"
)

plt.plot(
    df_plot["t_rel"],
    df_plot["loc_car_position_y"],
    label="loc Car Position Y"
)

plt.xlabel("Time since window start (s)")
plt.ylabel("Y Position")
plt.title("Car Position Y vs YOLO Y vs loc Car Position Y (middle 100 rows)")
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig("car_run_data/position_y_comparison.png")  # 儲存圖表為 PNG 檔案
plt.show()

# ==========================================
# 8. Plot 3: 距離誤差 yolo
# ==========================================
plt.figure(figsize=(12, 5))

plt.plot(
    df_plot["t_rel"],
    df_plot["dist_yolo_error"],
    label="Distance Error"
)

plt.axhline(
    y=0,
    linestyle="--"
)

plt.xlabel("Time since window start (s)")
plt.ylabel("Distance Error")
plt.title("Car Position - YOLO Distance Error (middle 100 rows)")
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig("car_run_data/yolo_position_error.png")  # 儲存圖表為 PNG 檔案
plt.show()


# ==========================================
# 9. Plot 4: 距離誤差 loc
# ==========================================
plt.figure(figsize=(12, 5))

plt.plot(
    df_plot["t_rel"],
    df_plot["dist_loc_error"],
    label="Distance Error"
)

plt.axhline(
    y=0,
    linestyle="--"
)

plt.xlabel("Time since window start (s)")
plt.ylabel("Distance Error")
plt.title("Car Position - loc Car Position Distance Error (middle 100 rows)")
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig("car_run_data/loc_position_error.png")  # 儲存圖表為 PNG 檔案
plt.show()


# ===========================================
# 10. Plot 5: YOLO 與 loc 的距離誤差比較
# ===========================================
plt.figure(figsize=(12, 5))

plt.plot(
    df_plot["t_rel"],
    df_plot["dist_yolo_loc_error"],
    label="YOLO - loc Distance Error"
)

plt.axhline(
    y=0,
    linestyle="--"
)

plt.xlabel("Time since window start (s)")
plt.ylabel("Distance Error")
plt.title("YOLO vs loc Distance Error (middle 100 rows)")
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig("car_run_data/yolo_loc_position_error.png")  # 儲存圖表為 PNG 檔案
plt.show()


# ==========================================
# 11. 額外輸出統計資訊 (用全部過濾後的資料, 不是只有畫圖的 100 筆)
# ==========================================
print("\n========== Distance Error Statistics (all NaN-filtered rows) ==========")
# 距離誤差本身就 >= 0, 所以 Mean 跟 MAE 是同一個數字, 不重複印。
# 額外加 Median / p95, 對長尾 (離群值) 比 Mean/RMSE 更看得出東西。

print("\nDistance Error Yolo (car vs yolo):")
print(f"Mean       : {df['dist_yolo_error'].mean():.6f}")
print(f"Median     : {df['dist_yolo_error'].median():.6f}")
print(f"p95        : {df['dist_yolo_error'].quantile(0.95):.6f}")
print(f"Max        : {df['dist_yolo_error'].max():.6f}")
print(f"RMSE       : {(df['dist_yolo_error'] ** 2).mean() ** 0.5:.6f}")

print("\nDistance Error loc (car vs loc):")
print(f"Mean       : {df['dist_loc_error'].mean():.6f}")
print(f"Median     : {df['dist_loc_error'].median():.6f}")
print(f"p95        : {df['dist_loc_error'].quantile(0.95):.6f}")
print(f"Max        : {df['dist_loc_error'].max():.6f}")
print(f"RMSE       : {(df['dist_loc_error'] ** 2).mean() ** 0.5:.6f}")

print("\nDistance Error YOLO vs loc:")
print(f"Mean       : {df['dist_yolo_loc_error'].mean():.6f}")
print(f"Median     : {df['dist_yolo_loc_error'].median():.6f}")
print(f"p95        : {df['dist_yolo_loc_error'].quantile(0.95):.6f}")
print(f"Max        : {df['dist_yolo_loc_error'].max():.6f}")
print(f"RMSE       : {(df['dist_yolo_loc_error'] ** 2).mean() ** 0.5:.6f}")
