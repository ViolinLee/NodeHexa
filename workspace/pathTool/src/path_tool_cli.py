from __future__ import annotations

import argparse
import os
import sys
from typing import Optional

from models import HexapodModel, QuadModel


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="pathTool: generate robot paths")
    parser.add_argument(
        "--robot",
        metavar="ROBOT",
        dest="robot",
        default="hexapod",
        choices=["hexapod", "quad"],
        help="robot model: hexapod or quad (default: hexapod)",
    )
    parser.add_argument(
        "--pathDir",
        metavar="DIR",
        dest="path_dir",
        default="path",
        help="path script directory for hexapod (default: path)",
    )
    parser.add_argument(
        "--outPath",
        metavar="PATH",
        dest="out_path",
        default=None,
        help="output header path (default: firmware/src/generated/movement_table*.h)",
    )
    return parser


def resolve_default_out_path(robot: str, script_dir: str) -> str:
    # 默认输出：直接落到固件目录，避免在仓库里出现多份“生成文件副本”
    repo_root = os.path.abspath(os.path.join(script_dir, "..", "..", ".."))
    generated_dir = os.path.join(repo_root, "firmware", "src", "generated")
    if robot == "quad":
        return os.path.join(generated_dir, "movement_table_quad.h")
    return os.path.join(generated_dir, "movement_table.h")


def ensure_out_dir(out_path: str) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)


def run_hexapod(path_dir: str, out_path: str, script_dir: str) -> None:
    # 兼容从任意工作目录运行：默认 pathDir=path 时，优先尝试以脚本目录为基准解析
    if not os.path.isabs(path_dir) and not os.path.exists(path_dir):
        candidate = os.path.join(script_dir, path_dir)
        if os.path.exists(candidate):
            path_dir = candidate

    # 保持原有行为：从 pathDir 中搜集脚本并生成六足 movement_table.h
    sys.path.insert(0, path_dir)
    model = HexapodModel()
    paths = model.collect_path(path_dir)
    results = {path: generator() for path, generator in paths.items()}

    verified = [1 for path, data in results.items() if not model.verify_path(path, data)]
    if len(verified) > 0:
        print("There were errors, exit...")
        sys.exit(1)

    with open(out_path, "w") as f:
        print("//", file=f)
        print("// This file is generated, dont directly modify content...", file=f)
        print("//", file=f)
        print("namespace {", file=f)
        for path, data in results.items():
            print(model.generate_c_body(path, data), file=f)
        print("}\n", file=f)
        for path in results:
            print(model.generate_c_def(path), file=f)

    print("Hexapod result written to {}".format(os.path.abspath(out_path)))


def run_quad(out_path: str) -> None:
    # 生成四足多步态离线动作表
    model = QuadModel()
    results = model.generate_all_gaits()

    with open(out_path, "w") as f:
        print("//", file=f)
        print("// This file is generated for Quad robot, dont directly modify content...", file=f)
        print("//", file=f)
        print("namespace quadruped {", file=f)
        for path, data in results.items():
            print(model.generate_c_body(path, data), file=f)
        for path in results:
            print(model.generate_c_def(path), file=f)
        print("}\n", file=f)

    print("Quad result written to {}".format(os.path.abspath(out_path)))


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    script_dir = os.path.dirname(os.path.abspath(__file__))
    if args.out_path is None:
        args.out_path = resolve_default_out_path(args.robot, script_dir)
    ensure_out_dir(args.out_path)

    if args.robot == "hexapod":
        run_hexapod(args.path_dir, args.out_path, script_dir)
    elif args.robot == "quad":
        run_quad(args.out_path)
    else:
        raise RuntimeError(f"Unsupported robot: {args.robot}")

    return 0

