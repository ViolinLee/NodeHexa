from __future__ import annotations

import os
import re
from math import atan2, cos, pi, sin
from typing import Dict, List, Tuple

from .base import RobotPathModel
from .quad_gait import QuadGait


class QuadModel(RobotPathModel):
    """四足多步态离线轨迹模型，直接内置 4 种步态"""

    LEG_COUNT = 4

    def __init__(self):
        # 默认站立足端坐标：从 firmware/include/config.h 推导（与 quad_movements.cpp/quad_robot.cpp 一致）
        # 约定顺序与固件 legs_[] / QuadLocations.p[0..3] 一致：
        #   0:FR, 1:BR, 2:BL, 3:FL
        cfg = self._load_firmware_config()

        # trig constants（与固件保持一致）
        SIN30, COS30 = 0.5, 0.866
        SIN45, COS45 = 0.7071, 0.7071
        SIN15, COS15 = 0.2588, 0.9659

        standby_z_pos = cfg["kLegJoint3ToTip"] * COS15 - cfg["kLegJoint2ToJoint3"] * SIN30
        reach = (
            cfg["kLegRootToJoint1"]
            + cfg["kLegJoint1ToJoint2"]
            + cfg["kLegJoint2ToJoint3"] * COS30
            + cfg["kLegJoint3ToTip"] * SIN15
        )
        reach_xy = reach * COS45

        offset_x = cfg["kQuadLegMountOtherX"] + reach_xy
        offset_y = cfg["kQuadLegMountOtherY"] + reach_xy
        standby_z = -standby_z_pos

        # 坐标系与六足一致：X 向右为正，Y 向前为正
        self.home_x = [offset_x, offset_x, -offset_x, -offset_x]  # FR, BR, BL, FL
        self.home_y = [offset_y, -offset_y, -offset_y, offset_y]
        self.home_z = [standby_z, standby_z, standby_z, standby_z]
        self.gait = QuadGait(self.home_x, self.home_y, self.home_z, frame_time_ms=20)

    @staticmethod
    def _load_firmware_config() -> Dict[str, float]:
        """
        从 firmware/include/config.h 提取需要的浮点常量，保证 pathTool 与固件同源。
        若读取失败则退化为一组保守默认值（不会影响生成流程，但可能与固件不一致）。
        """
        defaults = {
            "kLegRootToJoint1": 20.75,
            "kLegJoint1ToJoint2": 28.0,
            "kLegJoint2ToJoint3": 42.6,
            "kLegJoint3ToTip": 89.07,
            "kQuadLegMountOtherX": 25.0,
            "kQuadLegMountOtherY": 45.0,
        }

        try:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            repo_root = os.path.abspath(os.path.join(script_dir, "..", "..", "..", ".."))
            cfg_path = os.path.join(repo_root, "firmware", "include", "config.h")
            text = open(cfg_path, "r", encoding="utf-8", errors="ignore").read()

            def get_float(name: str) -> float:
                m = re.search(rf"const\s+float\s+{re.escape(name)}\s*=\s*([0-9]+(?:\.[0-9]+)?)\s*;", text)
                return float(m.group(1)) if m else defaults[name]

            out = dict(defaults)
            for k in list(defaults.keys()):
                out[k] = get_float(k)
            return out
        except Exception:
            return defaults

    def verify_path(self, path_name: str, params: Tuple) -> bool:
        # 暂时不做关节空间 IK 校验，后续可根据 Quad IK 补充
        print(f"[Quad] Skip IK verify for {path_name} (to be implemented).")
        return True

    def _generate_shift_data_from_world(self, frames_world: List[List[List[float]]]):
        """
        输入: N x 4 x 3 的世界坐标路径
        输出: data[4][N][3]，相对于 home_x/y/z 的位移
        """
        data = [[[0.0, 0.0, 0.0] for _ in range(len(frames_world))] for _ in range(self.LEG_COUNT)]
        for i, frame in enumerate(frames_world):
            for leg in range(self.LEG_COUNT):
                data[leg][i][0] = frame[leg][0] - self.home_x[leg]
                data[leg][i][1] = frame[leg][1] - self.home_y[leg]
                data[leg][i][2] = frame[leg][2] - self.home_z[leg]
        return data

    def generate_all_gaits(self) -> Dict[str, Tuple]:
        """
        返回 {path_name: (data, mode, dur, entries)}
        path_name 形如: quad_trot_forward / quad_trot_backward / quad_trot_shiftleft / ...
        """
        results: Dict[str, Tuple] = {}

        gait_defs = [
            ("quad_trot", QuadGait.GAIT_TROT),
            ("quad_walk", QuadGait.GAIT_WALK),
            ("quad_gallop", QuadGait.GAIT_GALLOP),
            ("quad_creep", QuadGait.GAIT_CREEP),
        ]

        def rot_z(vec, deg):
            rad = pi * deg / 180.0
            c = cos(rad)
            s = sin(rad)
            x, y, z = vec
            return [x * c - y * s, x * s + y * c, z]

        for base_name, gait_mode in gait_defs:
            # 先生成“前进方向”的世界坐标轨迹
            frames_fwd = self.gait.gen_path(gait_mode, QuadGait.MOVE_FORWARD, gait_speed=0)
            data_fwd = self._generate_shift_data_from_world(frames_fwd)
            # QuadMovementTable.stepDuration 在固件侧表示“每步/每帧时长(ms)”
            # 由于舵机 50Hz，建议固定为 20ms（与 self.gait.frame_time_ms 一致）
            dur = self.gait.frame_time_ms
            entries = [0, len(frames_fwd) // 2] if len(frames_fwd) >= 2 else [0]

            def make_variant(transform_fn):
                data_var = [[[0.0, 0.0, 0.0] for _ in range(len(frames_fwd))] for _ in range(self.LEG_COUNT)]
                for leg in range(self.LEG_COUNT):
                    for i in range(len(frames_fwd)):
                        data_var[leg][i] = transform_fn(leg, data_fwd[leg][i])
                return data_var

            # forward: 直接使用基准轨迹
            results[f"{base_name}_forward"] = (data_fwd, "shift_quad", dur, entries)

            # backward: 关于 X 轴对称 (y -> -y)
            results[f"{base_name}_backward"] = (
                make_variant(lambda leg, v: [v[0], -v[1], v[2]]),
                "shift_quad",
                dur,
                entries,
            )

            # 左右平移：整体绕 Z 轴旋转 ±90°
            # 约定：前进为 +Y，因此 shiftleft 应该为 -X（= rot_z(+90)）
            results[f"{base_name}_shiftleft"] = (
                make_variant(lambda leg, v: rot_z(v, 90.0)),
                "shift_quad",
                dur,
                entries,
            )
            results[f"{base_name}_shiftright"] = (
                make_variant(lambda leg, v: rot_z(v, -90.0)),
                "shift_quad",
                dur,
                entries,
            )

            # 左右转向：对每条腿施加不同旋转角度（参考 gait.py: formated_path_status）
            # 目标：转向轨迹所在平面应垂直于“腿安装点 -> 机身中心”的径向线
            # 因此 XY 上的运动方向应取“切向”（radial ± 90°），而不是径向本身。
            def radial_deg(leg: int) -> float:
                # home_x/home_y 定义了腿相对机身中心的安装位置（FR, BR, BL, FL）
                deg = atan2(self.home_y[leg], self.home_x[leg]) * 180.0 / pi
                return (deg + 360.0) % 360.0

            left_angles = {leg: (radial_deg(leg) + 90.0) % 360.0 for leg in range(self.LEG_COUNT)}
            right_angles = {leg: (radial_deg(leg) - 90.0) % 360.0 for leg in range(self.LEG_COUNT)}

            # 基准 gait 的前进方向为 +Y（90°），因此要得到目标方向角 D，需要旋转 (D - 90°)
            base_forward_deg = 90.0
            results[f"{base_name}_turnleft"] = (
                make_variant(lambda leg, v: rot_z(v, left_angles[leg] - base_forward_deg)),
                "shift_quad",
                dur,
                entries,
            )
            results[f"{base_name}_turnright"] = (
                make_variant(lambda leg, v: rot_z(v, right_angles[leg] - base_forward_deg)),
                "shift_quad",
                dur,
                entries,
            )

        return results

    def generate_c_body(self, path_name: str, params: Tuple) -> str:
        data, mode, dur, entries = params
        if mode != "shift_quad":
            raise RuntimeError(f"[Quad] unsupported mode: {mode}")

        assert len(data) == self.LEG_COUNT
        count = len(data[0])

        result = "\nconst QuadLocations {}_paths[] {{\n".format(path_name)
        for i in range(count):
            # QuadLocations 结构体只有一个成员：Point3D p[4]
            # 这里需要额外一层括号来初始化数组成员，否则会报 “too many initializers”
            result += "    {{" + ", ".join(
                "{{Q{idx}X+({x:.2f}), Q{idx}Y+({y:.2f}), Q{idx}Z+({z:.2f})}}".format(
                    x=data[leg][i][0], y=data[leg][i][1], z=data[leg][i][2], idx=leg + 1
                )
                for leg in range(self.LEG_COUNT)
            ) + "}},\n"
        result += "};\n"
        result += "const int {}_entries[] {{ {} }};\n".format(
            path_name, ",".join(str(e) for e in entries)
        )
        result += (
            "const QuadMovementTable {name}_table "
            "{{{name}_paths, {count}, {dur}, {name}_entries, {ecount} }};"
        ).format(name=path_name, count=count, dur=dur, ecount=len(entries))
        return result

    def generate_c_def(self, path_name: str) -> str:
        # 注意：必须放在 namespace quadruped 内，否则会出现 table 符号不可见的问题
        return """const QuadMovementTable& {name}Table() {{
    return {name}_table;
}}""".format(name=path_name)

