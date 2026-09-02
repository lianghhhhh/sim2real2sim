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
# 3. 計算 X / Y 誤差
# ==========================================
df["x_yolo_error"] = df["car_position_x"] - df["yolo_x"]
df["y_yolo_error"] = df["car_position_y"] - df["yolo_y"]
df["x_loc_error"] = df["car_position_x"] - df["loc_car_position_x"]
df["y_loc_error"] = df["car_position_y"] - df["loc_car_position_y"]

# ==========================================
# 4. Plot 1: X 座標比較
# ==========================================
plt.figure(figsize=(12, 5))

plt.plot(
    df["timestamp"],
    df["car_position_x"],
    label="Car Position X"
)

plt.plot(
    df["timestamp"],
    df["yolo_x"],
    label="YOLO X"
)

plt.plot(
    df["timestamp"],
    df["loc_car_position_x"],
    label="loc Car Position X"
)

plt.xlabel("Timestamp")
plt.ylabel("X Position")
plt.title("Car Position X vs YOLO X vs loc Car Position X")
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig("car_run_data/position_x_comparison.png")  # 儲存圖表為 PNG 檔案
plt.show()

# ==========================================
# 5. Plot 2: Y 座標比較
# ==========================================
plt.figure(figsize=(12, 5))

plt.plot(
    df["timestamp"],
    df["car_position_y"],
    label="Car Position Y"
)

plt.plot(
    df["timestamp"],
    df["yolo_y"],
    label="YOLO Y"
)

plt.plot(
    df["timestamp"],
    df["loc_car_position_y"],
    label="loc Car Position Y"
)

plt.xlabel("Timestamp")
plt.ylabel("Y Position")
plt.title("Car Position Y vs YOLO Y vs loc Car Position Y")
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig("car_run_data/position_y_comparison.png")  # 儲存圖表為 PNG 檔案
plt.show()
# ==========================================
# 6. Plot 3: X / Y 誤差 yolo
# ==========================================
plt.figure(figsize=(12, 5))

plt.plot(
    df["timestamp"],
    df["x_yolo_error"],
    label="X Error"
)

plt.plot(
    df["timestamp"],
    df["y_yolo_error"],
    label="Y Error"
)

plt.axhline(
    y=0,
    linestyle="--"
)

plt.xlabel("Timestamp")
plt.ylabel("Position Error")
plt.title("Car Position - YOLO Position Error")
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig("car_run_data/yolo_position_error.png")  # 儲存圖表為 PNG 檔案
plt.show()



# ==========================================
# 7. Plot 4: X / Y 誤差 loc
# ==========================================
plt.figure(figsize=(12, 5))

plt.plot(
    df["timestamp"],
    df["x_loc_error"],
    label="X Error"
)

plt.plot(
    df["timestamp"],
    df["y_loc_error"],
    label="Y Error"
)

plt.axhline(
    y=0,
    linestyle="--"
)

plt.xlabel("Timestamp")
plt.ylabel("Position Error")
plt.title("Car Position - loc Car Position Error")
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig("car_run_data/loc_position_error.png")  # 儲存圖表為 PNG 檔案
plt.show()


# ==========================================
# 8. 額外輸出統計資訊
# ==========================================
print("\n========== Error Statistics ==========")

print("\nX Error Yolo:")
print(f"Mean Error : {df['x_yolo_error'].mean():.6f}")
print(f"MAE        : {df['x_yolo_error'].abs().mean():.6f}")
print(f"Max Error  : {df['x_yolo_error'].abs().max():.6f}")
print(f"RMSE       : {(df['x_yolo_error'] ** 2).mean() ** 0.5:.6f}")

print("\nY Error Yolo:")
print(f"Mean Error : {df['y_yolo_error'].mean():.6f}")
print(f"MAE        : {df['y_yolo_error'].abs().mean():.6f}")
print(f"Max Error  : {df['y_yolo_error'].abs().max():.6f}")
print(f"RMSE       : {(df['y_yolo_error'] ** 2).mean() ** 0.5:.6f}")

print("\nX Error loc:")
print(f"Mean Error : {df['x_loc_error'].mean():.6f}")
print(f"MAE        : {df['x_loc_error'].abs().mean():.6f}")
print(f"Max Error  : {df['x_loc_error'].abs().max():.6f}")
print(f"RMSE       : {(df['x_loc_error'] ** 2).mean() ** 0.5:.6f}")

print("\nY Error loc:")
print(f"Mean Error : {df['y_loc_error'].mean():.6f}")
print(f"MAE        : {df['y_loc_error'].abs().mean():.6f}")
print(f"Max Error  : {df['y_loc_error'].abs().max():.6f}")
print(f"RMSE       : {(df['y_loc_error'] ** 2).mean() ** 0.5:.6f}")
