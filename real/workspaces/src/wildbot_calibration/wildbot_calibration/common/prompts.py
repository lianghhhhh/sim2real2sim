"""Interactive CLI helpers (Traditional Chinese prompts)."""

from __future__ import annotations

import sys
import time


def header(title: str) -> None:
    bar = "=" * 64
    print()
    print(bar)
    print(f"  {title}")
    print(bar)


def countdown(seconds: int, message: str = "") -> None:
    if message:
        print(message)
    for i in range(seconds, 0, -1):
        sys.stdout.write(f"\r  倒數 {i} 秒... ")
        sys.stdout.flush()
        time.sleep(1)
    sys.stdout.write("\r  開始!         \n")
    sys.stdout.flush()


def confirm(message: str, default: bool = False) -> bool:
    suffix = " [Y/n] " if default else " [y/N] "
    while True:
        try:
            ans = input(message + suffix).strip().lower()
        except EOFError:
            return default
        if ans == "":
            return default
        if ans in ("y", "yes"):
            return True
        if ans in ("n", "no"):
            return False
        print("  請輸入 y 或 n。")


def prompt_float(message: str, default: float | None = None) -> float:
    suffix = f" [預設 {default}]: " if default is not None else ": "
    while True:
        try:
            ans = input(message + suffix).strip()
        except EOFError:
            if default is not None:
                return default
            raise
        if ans == "" and default is not None:
            return default
        try:
            return float(ans)
        except ValueError:
            print("  請輸入數字。")


def prompt_int(message: str, default: int | None = None) -> int:
    suffix = f" [預設 {default}]: " if default is not None else ": "
    while True:
        try:
            ans = input(message + suffix).strip()
        except EOFError:
            if default is not None:
                return default
            raise
        if ans == "" and default is not None:
            return default
        try:
            return int(ans)
        except ValueError:
            print("  請輸入整數。")


def wait_enter(message: str = "按 ENTER 開始 (Ctrl+C 取消)...") -> None:
    try:
        input(message)
    except EOFError:
        pass
