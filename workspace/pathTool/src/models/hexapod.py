from __future__ import annotations

import os
from typing import Dict, List, Tuple

import config
import kinematics

from .base import RobotPathModel


class HexapodModel(RobotPathModel):
    """六足 Hexapod 模型实现（保持原有行为）"""

    LEG_COUNT = 6

    def collect_path(self, sub_folder: str) -> Dict[str, callable]:
        scripts = {}
        for script_name in [
            f[:-3]
            for f in sorted(os.listdir(sub_folder))
            if f.endswith(".py") and os.path.isfile(os.path.join(sub_folder, f))
        ]:
            module = __import__(script_name)
            if not hasattr(module, "path_generator"):
                continue
            scripts[script_name] = module.path_generator
        return scripts

    def _verify_points(self, pt: List[float]):
        angles = kinematics.ik(pt)

        ok = True
        failed = []
        for i, angle in enumerate(angles):
            if angle < config.angleLimitation[i][0] or angle > config.angleLimitation[i][1]:
                ok = False
                failed.append((i, angle))

        return ok, failed

    def verify_path(self, path_name: str, params: Tuple) -> bool:
        # 仅六足生成需要 numpy（path.lib 内部 import numpy）
        from path.lib import point_rotate_z, matrix_mul

        data, mode, _, _ = params
        print(f"Verifying {path_name}...")

        all_ok = True
        if mode == "shift":
            # data: float[6][N][3]
            assert len(data) == self.LEG_COUNT

            for i in range(len(data[0])):
                for j in range(self.LEG_COUNT):
                    pt = [
                        config.defaultPosition[j][k]
                        - config.mountPosition[j][k]
                        + data[j][i][k]
                        for k in range(3)
                    ]
                    pt = point_rotate_z(pt, config.defaultAngle[j])
                    ok, failed = self._verify_points(pt)

                    if not ok:
                        print("{}, {} failed: {}".format(i, j, failed))
                        all_ok = False

        elif mode == "matrix":
            # data: np.matrix[N]
            for i in range(len(data)):
                for j in range(self.LEG_COUNT):
                    pt = matrix_mul(data[i], config.defaultPosition[j])
                    for k in range(3):
                        pt[k] -= config.mountPosition[j][k]
                    pt = point_rotate_z(pt, config.defaultAngle[j])

                    ok, failed = self._verify_points(pt)

                    if not ok:
                        print("{}, {} failed: {}".format(i, j, failed))
                        all_ok = False

        return all_ok

    def generate_c_body(self, path_name: str, params: Tuple) -> str:
        data, mode, dur, entries = params
        result = "\nconst Locations {}_paths[] {{\n".format(path_name)

        if mode == "shift":
            # data: float[6][N][3]
            assert len(data) == self.LEG_COUNT

            count = len(data[0])
            for i in range(count):
                result += (
                    "    {"
                    + ", ".join(
                        "{{P{idx}X+({x:.2f}), P{idx}Y+({y:.2f}), P{idx}Z+({z:.2f})}}".format(
                            x=data[j][i][0],
                            y=data[j][i][1],
                            z=data[j][i][2],
                            idx=j + 1,
                        )
                        for j in range(self.LEG_COUNT)
                    )
                    + "},\n"
                )

        elif mode == "matrix":
            # data: np.matrix[N]
            # 注意：这里的 length 必须是“帧数 N”，而不是单个 4x4 矩阵的 len()（那会返回 4）。
            count = len(data)
            for i in range(count):
                result += (
                    "    {"
                    + ", \n     ".join(
                        "{{P{idx}X*{e00:.2f} + P{idx}Y*{e01:.2f} + P{idx}Z*{e02:.2f} + {e03:.2f}, "
                        "P{idx}X*{e10:.2f} + P{idx}Y*{e11:.2f} + P{idx}Z*{e12:.2f} + {e13:.2f}, "
                        "P{idx}X*{e20:.2f} + P{idx}Y*{e21:.2f} + P{idx}Z*{e22:.2f} + {e23:.2f}}}".format(
                            e00=data[i].item((0, 0)),
                            e01=data[i].item((0, 1)),
                            e02=data[i].item((0, 2)),
                            e03=data[i].item((0, 3)),
                            e10=data[i].item((1, 0)),
                            e11=data[i].item((1, 1)),
                            e12=data[i].item((1, 2)),
                            e13=data[i].item((1, 3)),
                            e20=data[i].item((2, 0)),
                            e21=data[i].item((2, 1)),
                            e22=data[i].item((2, 2)),
                            e23=data[i].item((2, 3)),
                            idx=j + 1,
                        )
                        for j in range(self.LEG_COUNT)
                    )
                    + "},\n"
                )

        else:
            raise RuntimeError("Generation mode: {} not supported".format(mode))

        result += "};\n"
        result += "const int {}_entries[] {{ {} }};\n".format(
            path_name, ",".join(str(e) for e in entries)
        )
        result += (
            "const MovementTable {name}_table "
            "{{{name}_paths, {count}, {dur}, {name}_entries, {ecount} }};"
        ).format(name=path_name, count=count, dur=dur, ecount=len(entries))
        return result

    def generate_c_def(self, path_name: str) -> str:
        return """const MovementTable& {name}Table() {{
    return {name}_table;
}}""".format(name=path_name)

