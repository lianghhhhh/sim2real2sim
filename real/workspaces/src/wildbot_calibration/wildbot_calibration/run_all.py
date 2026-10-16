"""Wizard: run the calibration steps in order. Gripper intentionally omitted."""

from __future__ import annotations

import sys

from wildbot_calibration.common import prompts
from wildbot_calibration.common import yaml_store

from wildbot_calibration import imu_axis_sanity
from wildbot_calibration import imu_static_calibration
from wildbot_calibration import wheel_linear_calibration
from wildbot_calibration import wheel_angular_calibration
from wildbot_calibration import imu_accel_two_face


# Order rationale:
#  1. axis_sanity — silent killer, must come first
#  2. static_calibration — yaw integration becomes trustworthy after bias removal
#  3. wheel_linear — independent (tape measure)
#  4. wheel_angular — uses IMU yaw integration (needs step 2)
#  5. accel_two_face — order-independent, last
STEPS = [
    ("imu_axis_sanity", imu_axis_sanity.main),
    ("imu_static_calibration", imu_static_calibration.main),
    ("wheel_linear_calibration", wheel_linear_calibration.main),
    ("wheel_angular_calibration", wheel_angular_calibration.main),
    ("imu_accel_two_face", imu_accel_two_face.main),
]


def _prompt_run_or_skip(name: str, idx: int, total: int) -> bool:
    print()
    print(f"[{idx}/{total}] {name}")
    while True:
        try:
            ans = input("  按 ENTER 開始,輸入 s 跳過,輸入 q 結束 wizard: ").strip().lower()
        except EOFError:
            return False
        if ans == "":
            return True
        if ans == "s":
            return False
        if ans == "q":
            raise KeyboardInterrupt
        print("  請輸入 ENTER / s / q。")


def _print_final_summary() -> None:
    prompts.header("最終 Summary")
    try:
        imu = yaml_store.get_section("imu")
        base = yaml_store.get_section("base")
    except Exception as exc:  # noqa: BLE001
        print(f"  讀取 YAML 失敗:{exc}")
        return

    print("  IMU:")
    print(f"    axis_sanity_passed: {imu.get('axis_sanity_passed')}")
    print(f"    gyro_bias:          {imu.get('gyro_bias')}")
    print(f"    gyro_cov_diag:      {imu.get('gyro_covariance_diag')}")
    print(f"    accel_bias_xy:      {imu.get('accel_bias_xy')}")
    print(f"    accel_cov_diag:     {imu.get('accel_covariance_diag')}")
    print(f"    |g|:                {imu.get('accel_gravity_magnitude')}")
    print()
    print("  Base:")
    print(f"    linear_scale:                       {base.get('linear_scale')}")
    print(
        "    wheel_radius:                       "
        f"{base.get('wheel_radius_nominal')} -> {base.get('wheel_radius_corrected')}"
    )
    print(f"    angular_scale:                      {base.get('angular_scale')}")
    print(
        "    wheel_separation_multiplier:        "
        f"{base.get('wheel_separation_multiplier_nominal')}"
        f" -> {base.get('wheel_separation_multiplier_corrected')}"
    )
    print()
    print(f"  完整結果請見:{yaml_store.get_path()}")


def main() -> None:
    prompts.header("wildbot calibration wizard")
    print("  依序執行 5 個校正步驟。每步驟之間會詢問是否執行 / 跳過 / 結束。")
    print("  (夾爪已另行校正,本 wizard 不包含。)")
    print()

    total = len(STEPS)
    for idx, (name, fn) in enumerate(STEPS, start=1):
        try:
            run = _prompt_run_or_skip(name, idx, total)
        except KeyboardInterrupt:
            print("\n  使用者結束 wizard。")
            break
        if not run:
            print(f"  跳過 {name}。")
            if idx == 1:
                # Step 1 skip / fail: warn before continuing.
                print()
                print("  ⚠️  跳過 axis_sanity!後面校正可能因 IMU 軸向錯誤而失效。")
                if not prompts.confirm("  仍要繼續其他步驟?", default=False):
                    print("  結束 wizard。")
                    break
            continue
        try:
            fn()
        except KeyboardInterrupt:
            print(f"\n  {name} 被中斷。")
            if not prompts.confirm("  繼續下一步驟?", default=False):
                break
        except SystemExit:
            raise
        except Exception as exc:  # noqa: BLE001
            print(f"  ✗ {name} 拋出例外:{exc}")
            if not prompts.confirm("  忽略並繼續?", default=False):
                break
        else:
            # If this was axis_sanity and it didn't write passed=true, warn.
            if name == "imu_axis_sanity":
                try:
                    imu = yaml_store.get_section("imu")
                except Exception:
                    imu = {}
                if not imu.get("axis_sanity_passed", False):
                    print()
                    print("  ⚠️  axis_sanity 未通過,後面校正會基於錯誤的軸向。")
                    if not prompts.confirm("  仍要繼續其他步驟?", default=False):
                        print("  結束 wizard。")
                        break

    _print_final_summary()
    print()
    print("  請檢視 ~/.ros/calibration_results.yaml 並依各步驟提示手動更新 controllers.yaml。")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n  使用者中斷。")
        sys.exit(130)
