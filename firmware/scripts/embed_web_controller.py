#!/usr/bin/env python3
"""
将 workspace/panel 下的 HTML 页面压缩为 gzip，并生成固件侧 PROGMEM 头文件：
- web_controller.html  -> firmware/include/controller_index.h
- calibration.html     -> firmware/include/calibration_index.h

用法：
  python3 firmware/scripts/embed_web_controller.py                 # 默认生成 controller（兼容旧用法）
  python3 firmware/scripts/embed_web_controller.py --page controller
  python3 firmware/scripts/embed_web_controller.py --page calibration
  python3 firmware/scripts/embed_web_controller.py --page all      # 两者都生成

设计目标：
- 不依赖外部工具（gzip/xxd）
- gzip mtime 固定为 0，输出可复现
"""

from __future__ import annotations

from pathlib import Path
import gzip
import argparse
from dataclasses import dataclass


def to_c_array(data: bytes, cols: int = 16) -> str:
    lines: list[str] = []
    for i in range(0, len(data), cols):
        chunk = data[i : i + cols]
        lines.append(",".join(f"0x{b:02X}" for b in chunk) + ("," if i + cols < len(data) else ""))
    return "\n".join(lines)


@dataclass(frozen=True)
class PageSpec:
    key: str
    src_rel: str
    out_rel: str
    gz_name: str
    len_macro: str
    array_name: str


PAGES: dict[str, PageSpec] = {
    "controller": PageSpec(
        key="controller",
        src_rel="workspace/panel/web_controller.html",
        out_rel="firmware/include/controller_index.h",
        gz_name="web_controller.html.gz",
        len_macro="web_controller_html_gz_len",
        array_name="web_controller_html_gz",
    ),
    "calibration": PageSpec(
        key="calibration",
        src_rel="workspace/panel/calibration.html",
        out_rel="firmware/include/calibration_index.h",
        gz_name="calibration.html.gz",
        len_macro="calibration_html_gz_len",
        array_name="calibration_html_gz",
    ),
}


def generate(repo_root: Path, spec: PageSpec) -> tuple[Path, int]:
    src = repo_root / spec.src_rel
    out = repo_root / spec.out_rel

    if not src.exists():
        raise SystemExit(f"source not found: {src}")

    raw = src.read_bytes()
    gz = gzip.compress(raw, compresslevel=9, mtime=0)

    header: list[str] = []
    header.append(f"//File: {spec.gz_name}, Size: {len(gz)}")
    header.append(f"#define {spec.len_macro} {len(gz)}")
    header.append(f"const uint8_t {spec.array_name}[] PROGMEM = {{")
    header.append(to_c_array(gz))
    header.append("};")
    header.append("")

    out.write_text("\n".join(header), encoding="utf-8")
    return out, len(gz)


def main() -> int:
    # __file__ = .../firmware/scripts/embed_web_controller.py
    # parents[0]=scripts, parents[1]=firmware, parents[2]=repo root
    repo_root = Path(__file__).resolve().parents[2]

    parser = argparse.ArgumentParser(description="Embed HTML pages into firmware PROGMEM headers (gzip).")
    parser.add_argument(
        "--page",
        choices=["controller", "calibration", "all"],
        default="controller",
        help="选择要生成的页面（默认 controller，兼容旧用法）",
    )
    args = parser.parse_args()

    keys = ["controller", "calibration"] if args.page == "all" else [args.page]
    for key in keys:
        spec = PAGES[key]
        out, size = generate(repo_root, spec)
        print(f"generated: {out} ({size} bytes gz)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


