#!/usr/bin/env python3
"""把拆分后的飞控模块加入 TC32 生成的 subdir.mk（构建目录被 .gitignore 忽略）。

用法：
  python tools/patch_flight_build.py <Robin_24G_Car_Flash>/vendor/Robin_24G_Car/subdir.mk
可重复运行；已存在的源文件、目标文件和规则不会重复插入。
"""
from pathlib import Path
import sys

MODULES = [
    "FlightLoop", "FlightSafety", "FlightArmDebug", "MotorMixer",
    "RcInput", "AttitudeControl", "RateControl",
]


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    p = Path(sys.argv[1])
    s = p.read_text(encoding="utf-8-sig")

    for name in MODULES:
        src = f"../../../../demo/vendor/Robin_24G_Car/{name}.c"
        obj = f"./vendor/Robin_24G_Car/{name}.o"
        if src not in s:
            marker = "../../../../demo/vendor/Robin_24G_Car/UserRxdFunc.c \\\n"
            s = s.replace(marker, marker + src + " \\\n", 1)
        if obj not in s:
            marker = "./vendor/Robin_24G_Car/UserRxdFunc.o \\\n"
            s = s.replace(marker, marker + obj + " \\\n", 1)

        rule_start = f"vendor/Robin_24G_Car/{name}.o:"
        if rule_start not in s:
            base_start = s.find("vendor/Robin_24G_Car/UserRxdFunc.o:")
            base_end = s.find("\n\n", base_start)
            if base_start < 0 or base_end < 0:
                raise SystemExit("subdir.mk 缺少 UserRxdFunc 规则模板")
            base = s[base_start:base_end]
            s = s[:base_end] + "\n\n" + base.replace("UserRxdFunc", name) + s[base_end:]

    p.write_text(s, encoding="utf-8")
    print("patched", p)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
