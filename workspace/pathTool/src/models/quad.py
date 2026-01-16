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
        # 第三关节默认角度：10°
        SIN10, COS10 = 0.1736, 0.9848

        standby_z_pos = cfg["kLegJoint3ToTip"] * COS10 - cfg["kLegJoint2ToJoint3"] * SIN30
        reach = (
            cfg["kLegRootToJoint1"]
            + cfg["kLegJoint1ToJoint2"]
            + cfg["kLegJoint2ToJoint3"] * COS30
            + cfg["kLegJoint3ToTip"] * SIN10
        )
        # 四足站立足端外展角：与固件同源（见 firmware/include/config.h），reach 按 sin/cos 分解到 X/Y
        reach_x = reach * cfg["kQuadStanceCos"]
        reach_y = reach * cfg["kQuadStanceSin"]

        offset_x = cfg["kQuadLegMountOtherX"] + reach_x
        offset_y = cfg["kQuadLegMountOtherY"] + reach_y
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
            "kQuadStanceAngleDeg": 45.0,
            "kQuadStanceCos": 0.7071,
            "kQuadStanceSin": 0.7071,
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
            扭腰（twist）：对齐六足 `path/twist.py` 的效果，使用统一的刚体姿态变换序列（所有腿同一变换）。

            这里不用 numpy，直接用点旋转函数实现与 `m * Rz(a) * Rx(b)` 等价的点变换：
              p' = Rx(raise) ( Rz(a) ( Rx(b) p ) )
            """
            assert (steps % 4) == 0

            base = [[self.home_x[i], self.home_y[i], self.home_z[i]] for i in range(self.LEG_COUNT)]
            frames: List[List[List[float]]] = []

            quarter = int(steps / 4)

            # 对齐 `path/twist.py` 的参数风格：
            # - Z 轴旋转幅度：max_deg（对应 twist_x_angle）
            # - X 轴摆动幅度：按 12/20 比例缩放（对应 twise_y_angle=12, twist_x_angle=20）
            # raise 随旋转幅度渐进：z=0 时 raise=0，entry(0) 即 home。
            raise_deg = 3.0
            max_x_deg = float(max_deg) * (12.0 / 20.0)
            step_z_deg = float(max_deg) / quarter
            step_x_deg = float(max_x_deg) / quarter

            def apply_twist(p: List[float], z_deg: float, x_deg: float) -> List[float]:
                # p' = Rx(raise(z)) ( Rz(z) ( Rx(x) p ) )
                # 其中 raise(z) 在 z=0 时为 0，确保 entry 帧为 home（便于姿态动作互切无需抬腿对齐）
                p1 = rot_point_x(p, x_deg)
                p2 = rot_point_z(p1, z_deg)
                ramp = abs(z_deg) / float(max_deg) if abs(float(max_deg)) > 1e-6 else 0.0
                p3 = rot_point_x(p2, raise_deg * ramp)
                return p3

            # 4 段分段线性：与 `path/twist.py` 结构一致（避免突兀跳变，且能对齐 entries 语义）
            for i in range(quarter):
                z = i * step_z_deg
                x = i * step_x_deg
                frames.append([apply_twist(p, z, x) for p in base])

            for i in range(quarter):
                z = (quarter - i) * step_z_deg
                x = (quarter - i) * step_x_deg
                frames.append([apply_twist(p, z, x) for p in base])

            for i in range(quarter):
                z = -i * step_z_deg
                x = i * step_x_deg
                frames.append([apply_twist(p, z, x) for p in base])

            for i in range(quarter):
                z = (-quarter + i) * step_z_deg
                x = (quarter - i) * step_x_deg
                frames.append([apply_twist(p, z, x) for p in base])

            # entries 对齐六足：0 与 半程（20 steps => 10）
            return frames, 50, [0, quarter * 2]

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

            # ---- 波浪/非对称步态的方向修正（最小改动版）----
            # 现状：backward/shiftleft/shiftright 仅做几何变换（y翻转/旋转），但腿相位(=腿序)保持不变，
            # 对 walk/creep 这类“波浪传播”步态，某些方向会明显变差（真机后退/侧移不顺）。
            #
            # 方案：只对 phase_sensitive_gaits 的非前进方向，额外做一次“腿相位映射”（等价于换腿序）：
            # - backward：相位传播方向翻转（前后互换） => FR<->BR, FL<->BL
            # - shiftleft/shiftright：将传播方向相对前进方向旋转 ±90°（离散近似）
            #
            # 注：腿索引与固件一致：0:FR, 1:BR, 2:BL, 3:FL
            # phase_sensitive_gaits：需要额外“腿相位映射/腿序映射”的步态（波浪传播型）
            # 注：gallop 在这里不按 phase-sensitive 处理（否则会改变其传播特性），但 backward 需要额外做 front/back 映射，
            # 以保证 forward/backward 两表的“同一姿态帧”存在（用于丝滑切换/统一 entry）。
            phase_sensitive_gaits = {QuadGait.GAIT_WALK, QuadGait.GAIT_CREEP}

            # old_leg -> new_leg（把 old_leg 的相位/轨迹分配给 new_leg）
            perm_front_back = {0: 1, 1: 0, 2: 3, 3: 2}
            perm_rotate_cw = {0: 1, 1: 2, 2: 3, 3: 0}   # 传播方向顺时针旋转90°
            perm_rotate_ccw = {0: 3, 3: 2, 2: 1, 1: 0}  # 传播方向逆时针旋转90°

            def remap_legs_by_old_to_new(data_in, old_to_new):
                out_data = [[[0.0, 0.0, 0.0] for _ in range(len(frames_fwd))] for _ in range(self.LEG_COUNT)]
                for old_leg in range(self.LEG_COUNT):
                    new_leg = old_to_new.get(old_leg, old_leg)
                    out_data[new_leg] = data_in[old_leg]
                return out_data

            def compute_entries_for_data(data_leg0):
                s = choose_start_index_from_fr(data_leg0)
                if s < 0:
                    s = int(len(frames_fwd) / max(1, (2 * stages)))
                if s < 0:
                    s = 0
                if s >= len(frames_fwd):
                    s = 0
                h = (s + len(frames_fwd) // 2) % max(1, len(frames_fwd))
                return [s, h] if len(frames_fwd) >= 2 else [s]

            # ---- entry / pose 匹配工具：用于把两表的 entry 做成“等效姿态帧” ----
            def pose_key(data_in, idx: int, nd: int = 4):
                # data_in: float[4][N][3]
                return tuple(
                    (
                        round(float(data_in[leg][idx][0]), nd),
                        round(float(data_in[leg][idx][1]), nd),
                        round(float(data_in[leg][idx][2]), nd),
                    )
                    for leg in range(self.LEG_COUNT)
                )

            def find_matching_index(data_src, idx_src: int, data_dst) -> int:
                """
                在 data_dst 中找一个与 data_src[idx_src] 完全相同的姿态帧索引。
                若找不到则返回 -1。
                """
                if not data_dst or not data_dst[0]:
                    return -1
                key = pose_key(data_src, idx_src)
                for j in range(len(data_dst[0])):
                    if pose_key(data_dst, j) == key:
                        return j
                return -1

            def reverse_cycle(data_in):
                """时间反向（保持循环）：out[i] = in[(N-i)%N]"""
                n = len(data_in[0]) if data_in and data_in[0] else 0
                out = [[[0.0, 0.0, 0.0] for _ in range(n)] for _ in range(self.LEG_COUNT)]
                for leg in range(self.LEG_COUNT):
                    for i in range(n):
                        out[leg][i] = data_in[leg][(n - i) % n]
                return out

            def make_variant_from(data_src, transform_fn):
                data_var = [[[0.0, 0.0, 0.0] for _ in range(len(frames_fwd))] for _ in range(self.LEG_COUNT)]
                for leg in range(self.LEG_COUNT):
                    for i in range(len(frames_fwd)):
                        data_var[leg][i] = transform_fn(leg, data_src[leg][i])
                return data_var

            def make_variant(transform_fn):
                return make_variant_from(data_fwd, transform_fn)

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
            data_bwd = make_variant(lambda leg, v: [v[0], -v[1], v[2]])
            entries_bwd = entries
            if gait_mode in phase_sensitive_gaits or gait_mode == QuadGait.GAIT_GALLOP:
                data_bwd = remap_legs_by_old_to_new(data_bwd, perm_front_back)
                entries_bwd = compute_entries_for_data(data_bwd[0])
            results[f"{base_name}_backward"] = (
                data_bwd,
                "shift_quad",
                dur,
                entries_bwd,
            )

            # 左右平移：整体绕 Z 轴旋转 ±90°
            # 约定：前进为 +Y，因此 shiftleft 应该为 -X（= rot_z(+90)）
            data_sl = make_variant(lambda leg, v: rot_z(v, 90.0))
            entries_sl = entries
            if gait_mode in phase_sensitive_gaits:
                # shiftleft = 方向从 +Y 旋转到 -X（+90°），传播方向做离散同步旋转
                data_sl = remap_legs_by_old_to_new(data_sl, perm_rotate_ccw)
                entries_sl = compute_entries_for_data(data_sl[0])
            results[f"{base_name}_shiftleft"] = (
                data_sl,
                "shift_quad",
                dur,
                entries_sl,
            )
            data_sr = make_variant(lambda leg, v: rot_z(v, -90.0))
            entries_sr = entries
            if gait_mode in phase_sensitive_gaits:
                # shiftright = 方向从 +Y 旋转到 +X（-90°）
                data_sr = remap_legs_by_old_to_new(data_sr, perm_rotate_cw)
                entries_sr = compute_entries_for_data(data_sr[0])
            if gait_mode == QuadGait.GAIT_GALLOP:
                # gallop 的左右侧移表目前不满足“存在等效姿态帧”的要求（用于丝滑切换/单 entry）。
                # 这里改用：shiftright = shiftleft 的时间反向（物理意义上对应侧移方向反转），从而保证两表共享姿态集合。
                data_sr = reverse_cycle(data_sl)
                entries_sr = entries_sl
            results[f"{base_name}_shiftright"] = (
                data_sr,
                "shift_quad",
                dur,
                entries_sr,
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
            # turnright：对 phase-sensitive 步态（walk/creep），复用 backward 的“相位翻转/腿序映射”，
            # 让左右转向的抬腿序不同（否则 turnleft/turnright 只是几何镜像，腿相位完全一致）。
            base_backward_deg = 270.0  # -Y
            data_tr = make_variant_from(data_bwd, lambda leg, v: rot_z(v, right_angles[leg] - base_backward_deg))
            entries_tr = entries
            if gait_mode in phase_sensitive_gaits:
                entries_tr = compute_entries_for_data(data_tr[0])
            if gait_mode == QuadGait.GAIT_GALLOP:
                # gallop 左右转向同理：用 turnleft 的时间反向作为 turnright，保证存在等效姿态帧
                data_tr = reverse_cycle(results[f"{base_name}_turnleft"][0])
                entries_tr = results[f"{base_name}_turnleft"][3]
            results[f"{base_name}_turnright"] = (data_tr, "shift_quad", dur, entries_tr)

            # ---- 单 entry 规范化（组内 entry 等效切换）----
            # 约定：
            # - forward/backward：用 forward 的第2个 entry（entries[1]）做 canonical，
            #   backward 的 entry 通过“姿态帧匹配”自动求得（同一姿态不同索引）。
            # - turnleft/turnright：同理
            # - shiftleft/shiftright：同理（注意 gallop 已做 time-reverse，保证能匹配）
            def normalize_pair_entries(a_key: str, b_key: str) -> None:
                a_data, a_mode, a_dur, a_entries = results[a_key]
                b_data, b_mode, b_dur, b_entries = results[b_key]
                assert a_mode == b_mode == "shift_quad"
                # 选 a 的第二个 entry（若不存在则退化为第一个）
                a_idx = a_entries[1] if len(a_entries) >= 2 else a_entries[0]
                b_idx = find_matching_index(a_data, a_idx, b_data)
                if b_idx < 0:
                    # 兜底：若找不到完全相同姿态，退化为 b 的第二个 entry / 第一个
                    b_idx = b_entries[1] if len(b_entries) >= 2 else b_entries[0]
                results[a_key] = (a_data, a_mode, a_dur, [int(a_idx)])
                results[b_key] = (b_data, b_mode, b_dur, [int(b_idx)])

            def normalize_single_entry(k: str) -> None:
                d, m, du, es = results[k]
                if not es:
                    es = [0]
                # 对非 pair 的路径（如 forwardfast），同样只保留一个 entry：优先用 entries[1]（半程），否则 entries[0]
                idx = es[1] if len(es) >= 2 else es[0]
                results[k] = (d, m, du, [int(idx)])

            normalize_pair_entries(f"{base_name}_forward", f"{base_name}_backward")
            normalize_pair_entries(f"{base_name}_turnleft", f"{base_name}_turnright")
            normalize_pair_entries(f"{base_name}_shiftleft", f"{base_name}_shiftright")
            normalize_single_entry(f"{base_name}_forwardfast")

        # ---- 姿态动作：不分步态（参考六足的 rotate/twist，但实现保持“起点为 0 角度”更平滑） ----
        # 注：climb 四足不实现（重心不稳定），固件侧会做降级处理
        rx_frames, rx_dur, rx_entries = gen_posture_global_rotation("x", max_deg=15.0, steps=20)
        ry_frames, ry_dur, ry_entries = gen_posture_global_rotation("y", max_deg=15.0, steps=20)
        rz_frames, rz_dur, rz_entries = gen_posture_global_rotation("z", max_deg=15.0, steps=20)
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

