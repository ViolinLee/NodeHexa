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
            print("Load quadruped firmware config failed, using default values!")
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

        # 各步态阶段数
        gait_stages = {
            QuadGait.GAIT_TROT: 2,
            QuadGait.GAIT_WALK: 4,
            QuadGait.GAIT_GALLOP: 4,
            QuadGait.GAIT_CREEP: 6,
        }

        def choose_start_index_from_fr(data_fwd_leg0: List[List[float]]) -> int:
            """
            选择更“顺滑”的起步帧：
            - 优先选 FR(leg0) 抬腿最高点（dz 最大）那一帧；
            - 若最高点有多帧，选 |dy| 最小（更接近 Y=0）的一帧；
            - 若 FR 从未抬腿（dz<=0），返回 -1 由外层兜底。
            data_fwd_leg0: N x 3 (dx, dy, dz) ，相对 home 的位移
            """
            if not data_fwd_leg0:
                return -1
            dzs = [v[2] for v in data_fwd_leg0]
            max_dz = max(dzs)
            if max_dz <= 1e-6:
                return -1
            cand = [i for i, dz in enumerate(dzs) if abs(dz - max_dz) <= 1e-6]
            # 从候选中选 |dy| 最小
            best = min(cand, key=lambda i: abs(data_fwd_leg0[i][1]))
            return int(best)

        def rot_z(vec, deg):
            rad = pi * deg / 180.0
            c = cos(rad)
            s = sin(rad)
            x, y, z = vec
            return [x * c - y * s, x * s + y * c, z]

        def rot_point_x(p, deg):
            rad = pi * deg / 180.0
            c = cos(rad)
            s = sin(rad)
            x, y, z = p
            return [x, y * c - z * s, y * s + z * c]

        def rot_point_y(p, deg):
            rad = pi * deg / 180.0
            c = cos(rad)
            s = sin(rad)
            x, y, z = p
            return [x * c + z * s, y, -x * s + z * c]

        def rot_point_z(p, deg):
            rad = pi * deg / 180.0
            c = cos(rad)
            s = sin(rad)
            x, y, z = p
            return [x * c - y * s, x * s + y * c, z]

        def gen_posture_global_rotation(axis: str, max_deg: float, steps: int = 20) -> Tuple[List[List[List[float]]], int, List[int]]:
            """
            生成“姿态动作”世界坐标序列（不分步态）：
            - 以 home 点为基准，绕机身中心 (0,0,0) 做小幅旋转
            - t=0 时角度为 0，避免从 standby 切换时出现突兀跳变
            返回: (frames_world, stepDurationMs, entries)
            """
            base = [[self.home_x[i], self.home_y[i], self.home_z[i]] for i in range(self.LEG_COUNT)]
            frames: List[List[List[float]]] = []
            for i in range(steps):
                # 0 -> +max -> 0 -> -max -> 0
                ang = max_deg * sin(2.0 * pi * i / steps)
                if axis == "x":
                    rot = rot_point_x
                elif axis == "y":
                    rot = rot_point_y
                elif axis == "z":
                    rot = rot_point_z
                else:
                    raise ValueError(f"unsupported axis: {axis}")
                frames.append([rot(p, ang) for p in base])
            # 起步帧固定为 0（角度=0），更平滑
            return frames, 50, [0]

        def gen_posture_twist(max_deg: float, steps: int = 20) -> Tuple[List[List[List[float]]], int, List[int]]:
            """
            扭腰（twist）：前后腿绕 Z 轴相反方向旋转（前 +deg，后 -deg）
            同样保证 t=0 为 0 角度，避免跳变。
            """
            base = [[self.home_x[i], self.home_y[i], self.home_z[i]] for i in range(self.LEG_COUNT)]
            frames: List[List[List[float]]] = []
            for i in range(steps):
                ang = max_deg * sin(2.0 * pi * i / steps)
                frame: List[List[float]] = []
                for p in base:
                    # y>0 视为“前”，y<0 视为“后”
                    sign = 1.0 if p[1] >= 0 else -1.0
                    frame.append(rot_point_z(p, sign * ang))
                frames.append(frame)
            return frames, 50, [0]

        for base_name, gait_mode in gait_defs:
            # 先生成“前进方向”的世界坐标轨迹
            frames_fwd = self.gait.gen_path(gait_mode, QuadGait.MOVE_FORWARD, gait_speed=0)
            data_fwd = self._generate_shift_data_from_world(frames_fwd)
            # QuadMovementTable.stepDuration 在固件侧表示“每步/每帧时长(ms)”
            # 由于舵机 50Hz，建议固定为 20ms（与 self.gait.frame_time_ms 一致）
            dur = self.gait.frame_time_ms
            stages = gait_stages.get(gait_mode, 2)
            # 1) 优先：用 FR 抬腿峰值来做起步相位（更符合“从 FR 开始”的直觉）
            start_idx = choose_start_index_from_fr(data_fwd[0])
            # 2) 兜底：按 stages 推导一个“抬腿中点”
            if start_idx < 0:
                start_idx = int(len(frames_fwd) / max(1, (2 * stages)))  # 中点：len/(2*stages)
            if start_idx < 0:
                start_idx = 0
            if start_idx >= len(frames_fwd):
                start_idx = 0
            half = (start_idx + len(frames_fwd) // 2) % max(1, len(frames_fwd))
            entries = [start_idx, half] if len(frames_fwd) >= 2 else [start_idx]

            def make_variant(transform_fn):
                data_var = [[[0.0, 0.0, 0.0] for _ in range(len(frames_fwd))] for _ in range(self.LEG_COUNT)]
                for leg in range(self.LEG_COUNT):
                    for i in range(len(frames_fwd)):
                        data_var[leg][i] = transform_fn(leg, data_fwd[leg][i])
                return data_var

            # forward: 直接使用基准轨迹
            results[f"{base_name}_forward"] = (data_fwd, "shift_quad", dur, entries)

            # forwardfast: 跨步更大、抬腿更低（只做前进方向）
            # 通过临时调整 QuadGait 的振幅实现，生成后恢复，避免影响其他路径
            orig_amp_x = getattr(self.gait, "amplitudeX", 25)
            orig_amp_z = getattr(self.gait, "amplitudeZ", 25)
            try:
                self.gait.amplitudeX = float(orig_amp_x) * 1.6
                self.gait.amplitudeZ = float(orig_amp_z) * 0.6
                frames_fast = self.gait.gen_path(gait_mode, QuadGait.MOVE_FORWARD, gait_speed=0)
            finally:
                self.gait.amplitudeX = orig_amp_x
                self.gait.amplitudeZ = orig_amp_z

            data_fast = self._generate_shift_data_from_world(frames_fast)
            # fastforward 的 entries 也用 FR 抬腿峰值策略，避免起步跨到最远处
            start_fast = choose_start_index_from_fr(data_fast[0])
            if start_fast < 0 or start_fast >= len(frames_fast):
                start_fast = 0
            half_fast = (start_fast + len(frames_fast) // 2) % max(1, len(frames_fast))
            entries_fast = [start_fast, half_fast] if len(frames_fast) >= 2 else [start_fast]
            results[f"{base_name}_forwardfast"] = (data_fast, "shift_quad", dur, entries_fast)

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

        # ---- 姿态动作：不分步态（参考六足的 rotate/twist，但实现保持“起点为 0 角度”更平滑） ----
        # 注：climb 四足不实现（重心不稳定），固件侧会做降级处理
        rx_frames, rx_dur, rx_entries = gen_posture_global_rotation("x", max_deg=10.0, steps=20)
        ry_frames, ry_dur, ry_entries = gen_posture_global_rotation("y", max_deg=10.0, steps=20)
        rz_frames, rz_dur, rz_entries = gen_posture_global_rotation("z", max_deg=8.0, steps=20)
        tw_frames, tw_dur, tw_entries = gen_posture_twist(max_deg=10.0, steps=20)

        results["quad_rotatex"] = (self._generate_shift_data_from_world(rx_frames), "shift_quad", rx_dur, rx_entries)
        results["quad_rotatey"] = (self._generate_shift_data_from_world(ry_frames), "shift_quad", ry_dur, ry_entries)
        results["quad_rotatez"] = (self._generate_shift_data_from_world(rz_frames), "shift_quad", rz_dur, rz_entries)
        results["quad_twist"] = (self._generate_shift_data_from_world(tw_frames), "shift_quad", tw_dur, tw_entries)

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

