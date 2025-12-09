import argparse
import os
import sys

from typing import Dict, Tuple, List

import config
import kinematics
from path.lib import point_rotate_z, matrix_mul


# ------------------------------
# 抽象模型基类
# ------------------------------

class RobotPathModel:
    """路径生成与 C 代码导出抽象接口"""

    def verify_path(self, path_name: str, params: Tuple) -> bool:
        raise NotImplementedError

    def generate_c_body(self, path_name: str, params: Tuple) -> str:
        raise NotImplementedError

    def generate_c_def(self, path_name: str) -> str:
        raise NotImplementedError


# ------------------------------
# 六足 Hexapod 模型实现（保持原有行为）
# ------------------------------

class HexapodModel(RobotPathModel):
    LEG_COUNT = 6

    def collect_path(self, sub_folder: str) -> Dict[str, callable]:
        scripts = {}
        for script_name in [f[:-3] for f in sorted(os.listdir(sub_folder))
                            if f.endswith('.py') and os.path.isfile(os.path.join(sub_folder, f))]:
            module = __import__(script_name)
            if not hasattr(module, 'path_generator'):
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
        data, mode, _, _ = params
        print(f"Verifying {path_name}...")

        all_ok = True
        if mode == "shift":
            # data: float[6][N][3]
            assert len(data) == self.LEG_COUNT

            for i in range(len(data[0])):
                for j in range(self.LEG_COUNT):
                    pt = [config.defaultPosition[j][k] - config.mountPosition[j][k] + data[j][i][k] for k in range(3)]
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
                result += "    {" + ", ".join(
                    "{{P{idx}X+({x:.2f}), P{idx}Y+({y:.2f}), P{idx}Z+({z:.2f})}}".format(
                        x=data[j][i][0], y=data[j][i][1], z=data[j][i][2], idx=j + 1
                    )
                    for j in range(self.LEG_COUNT)
                ) + "},\n"

        elif mode == "matrix":
            # data: np.matrix[N]
            count = len(data)
            for i in range(count):
                result += "    {" + ", \n     ".join(
                    "{{P{idx}X*{e00:.2f} + P{idx}Y*{e01:.2f} + P{idx}Z*{e02:.2f} + {e03:.2f}, "
                    "P{idx}X*{e10:.2f} + P{idx}Y*{e11:.2f} + P{idx}Z*{e12:.2f} + {e13:.2f}, "
                    "P{idx}X*{e20:.2f} + P{idx}Y*{e21:.2f} + P{idx}Z*{e22:.2f} + {e23:.2f}}}".format(
                        e00=data[i].item((0, 0)), e01=data[i].item((0, 1)), e02=data[i].item((0, 2)),
                        e03=data[i].item((0, 3)),
                        e10=data[i].item((1, 0)), e11=data[i].item((1, 1)), e12=data[i].item((1, 2)),
                        e13=data[i].item((1, 3)),
                        e20=data[i].item((2, 0)), e21=data[i].item((2, 1)), e22=data[i].item((2, 2)),
                        e23=data[i].item((2, 3)),
                        idx=j + 1)
                    for j in range(self.LEG_COUNT)
                ) + "},\n"

        else:
            raise RuntimeError("Generation mode: {} not supported".format(mode))

        result += "};\n"
        result += "const int {}_entries[] {{ {} }};\n".format(
            path_name, ",".join(str(e) for e in entries)
        )
        result += (
            "const MovementTable {name}_table "
            "{{{name}_paths, {count}, {dur}, {name}_entries, {ecount} }};"
        ).format(name=path_name, count=len(data[0]), dur=dur, ecount=len(entries))
        return result

    def generate_c_def(self, path_name: str) -> str:
        return """const MovementTable& {name}Table() {{
    return {name}_table;
}}""".format(name=path_name)


# ------------------------------
# 四足 Quad 模型实现：多步态离线轨迹
# ------------------------------

from math import sin, cos, pi


class QuadGait:
    """基于参考 nodequad.gait.py 的四足步态生成（仅前进方向）"""

    GAIT_TROT = 0
    GAIT_WALK = 1
    GAIT_GALLOP = 2
    GAIT_CREEP = 3

    MOVE_STANDBY = 11
    MOVE_FORWARD = 12

    def __init__(self, home_x, home_y, home_z, frame_time_ms=20):
        self.home_x = home_x
        self.home_y = home_y
        self.home_z = home_z
        # gait constants
        self.amplitudeX, self.amplitudeY, self.amplitudeZ = 25, 15, 35
        self.frame_time_ms = frame_time_ms

    def gen_path(self, gait_mode, move_status, gait_speed=0):
        if gait_mode == self.GAIT_TROT:
            path = self.trot_gait(move_status, gait_speed)
        elif gait_mode == self.GAIT_WALK:
            path = self.walk_gait(move_status, gait_speed)
        elif gait_mode == self.GAIT_GALLOP:
            path = self.gallop_gait(move_status, gait_speed)
        elif gait_mode == self.GAIT_CREEP:
            path = self.creep_gait(move_status, gait_speed)
        else:
            raise ValueError()
        return path

    def _formated_path_status(self, fr_path_quad, br_path_quad, bl_path_quad, fl_path_quad, move_status):
        # path_quad: N x 4 x 3 （按 FR, BR, BL, FL 顺序）
        path_quad = [[fr_path_quad[path_id], br_path_quad[path_id],
                      bl_path_quad[path_id], fl_path_quad[path_id]]
                     for path_id in range(len(fr_path_quad))]
        if move_status == self.MOVE_STANDBY:
            corrected_path = [[[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]]
        elif move_status == self.MOVE_FORWARD:
            corrected_path = path_quad
        else:
            # 目前仅支持前进，其他模式留待后续扩展
            corrected_path = path_quad

        # 叠加到世界坐标下默认站立位置
        path_quad_world = [
            [
                [points_set[0][0] + self.home_x[0], points_set[0][1] + self.home_y[0], points_set[0][2] + self.home_z[0]],
                [points_set[1][0] + self.home_x[1], points_set[1][1] + self.home_y[1], points_set[1][2] + self.home_z[1]],
                [points_set[2][0] + self.home_x[2], points_set[2][1] + self.home_y[2], points_set[2][2] + self.home_z[2]],
                [points_set[3][0] + self.home_x[3], points_set[3][1] + self.home_y[3], points_set[3][2] + self.home_z[3]],
            ]
            for points_set in corrected_path
        ]
        return path_quad_world

    def trot_gait(self, move_status, gait_speed=0):
        duration = 200 if gait_speed == 0 else 400
        num_ticks = int(duration / self.frame_time_ms / 2)
        fr_path_quad = [[0.0, 0.0, 0.0] for _ in range(num_ticks * 2)]

        for stage_id in range(2):
            for tick_cnt in range(num_ticks):
                interp_id = stage_id * num_ticks + tick_cnt
                if stage_id == 0:
                    fr_path_quad[interp_id][0] = -self.amplitudeX * cos(pi * tick_cnt / num_ticks)
                    fr_path_quad[interp_id][1] = 0.0
                    fr_path_quad[interp_id][2] = abs(self.amplitudeZ) * sin(pi * tick_cnt / num_ticks) * -1
                elif stage_id == 1:
                    fr_path_quad[interp_id][0] = self.amplitudeX * cos(pi * tick_cnt / num_ticks)
                    fr_path_quad[interp_id][1] = 0.0
                    fr_path_quad[interp_id][2] = 0.0

        # Deep copy & 相位偏移
        # 对齐 nodequad 实现：FR 基准，BR/BL/FL 做旋转移位
        from copy import deepcopy
        def right_rotate_path(path, rotation_num):
            return path[(len(path) - rotation_num):] + path[:(len(path) - rotation_num)]

        br_path_quad = deepcopy(right_rotate_path(fr_path_quad, num_ticks))
        bl_path_quad = deepcopy(fr_path_quad)
        fl_path_quad = deepcopy(br_path_quad)

        return self._formated_path_status(fr_path_quad, br_path_quad, bl_path_quad, fl_path_quad, move_status)

    def walk_gait(self, move_status, gait_speed=0):
        duration = 4 * 4 * self.frame_time_ms if gait_speed == 0 else 1280
        num_ticks = int(duration / self.frame_time_ms / 4)
        fl_path_quad = [[0.0, 0.0, 0.0] for _ in range(num_ticks * 4)]

        for stage_id in range(4):
            for tick_cnt in range(num_ticks):
                interp_id = stage_id * num_ticks + tick_cnt
                if stage_id == 0:
                    fl_path_quad[interp_id][0] = -self.amplitudeX * cos(pi * tick_cnt / num_ticks) * 1.5
                    fl_path_quad[interp_id][1] = 0.0
                    fl_path_quad[interp_id][2] = abs(self.amplitudeZ) * sin(pi * tick_cnt / num_ticks) * -1
                elif stage_id in (1, 2, 3):
                    fl_path_quad[interp_id][0] = (
                        self.amplitudeX - self.amplitudeX * 2 * ((stage_id - 1) * num_ticks + tick_cnt) / (3 * num_ticks)
                    ) * 1.5
                    fl_path_quad[interp_id][1] = 0.0
                    fl_path_quad[interp_id][2] = 0.0

        def right_rotate_path(path, rotation_num):
            return path[(len(path) - rotation_num):] + path[:(len(path) - rotation_num)]

        from copy import deepcopy
        fr_path_quad = deepcopy(right_rotate_path(fl_path_quad, num_ticks * 2))
        br_path_quad = deepcopy(right_rotate_path(fl_path_quad, num_ticks * 1))
        bl_path_quad = deepcopy(right_rotate_path(fl_path_quad, num_ticks * 3))

        return self._formated_path_status(fr_path_quad, br_path_quad, bl_path_quad, fl_path_quad, move_status)

    def gallop_gait(self, move_status, gait_speed=0):
        duration = 4 * 4 * self.frame_time_ms if gait_speed == 0 else 1280
        num_ticks = int(duration / self.frame_time_ms / 4)
        fl_path_quad = [[0.0, 0.0, 0.0] for _ in range(num_ticks * 4)]

        for stage_id in range(4):
            for tick_cnt in range(num_ticks):
                interp_id = stage_id * num_ticks + tick_cnt
                if stage_id == 0:
                    fl_path_quad[interp_id][0] = -self.amplitudeX * cos(pi * tick_cnt / num_ticks) * 1.5
                    fl_path_quad[interp_id][1] = 0.0
                    fl_path_quad[interp_id][2] = abs(self.amplitudeZ) * sin(pi * tick_cnt / num_ticks) * -1
                elif stage_id in (1, 2, 3):
                    fl_path_quad[interp_id][0] = (
                        self.amplitudeX - self.amplitudeX * 2 * ((stage_id - 1) * num_ticks + tick_cnt) / (3 * num_ticks)
                    ) * 1.5
                    fl_path_quad[interp_id][1] = 0.0
                    fl_path_quad[interp_id][2] = 0.0

        def right_rotate_path(path, rotation_num):
            return path[(len(path) - rotation_num):] + path[:(len(path) - rotation_num)]

        from copy import deepcopy
        fr_path_quad = deepcopy(right_rotate_path(fl_path_quad, num_ticks * 1))
        br_path_quad = deepcopy(right_rotate_path(fl_path_quad, num_ticks * 2))
        bl_path_quad = deepcopy(right_rotate_path(fl_path_quad, num_ticks * 3))

        return self._formated_path_status(fr_path_quad, br_path_quad, bl_path_quad, fl_path_quad, move_status)

    def creep_gait(self, move_status, gait_speed=0):
        duration = 720 if gait_speed == 0 else 1440
        num_ticks = int(duration / self.frame_time_ms / 6)
        fr_path_quad = [[0.0, 0.0, 0.0] for _ in range(num_ticks * 6)]
        bl_path_quad = [[0.0, 0.0, 0.0] for _ in range(num_ticks * 6)]

        for stage_id in range(6):
            for tick_cnt in range(num_ticks):
                interp_id = stage_id * num_ticks + tick_cnt
                if stage_id == 0:
                    fr_path_quad[interp_id][0] = -self.amplitudeX * cos(pi * tick_cnt / num_ticks) * 2.0
                    fr_path_quad[interp_id][1] = 0.0
                    fr_path_quad[interp_id][2] = abs(self.amplitudeZ) * sin(pi * tick_cnt / num_ticks) * -1.5
                    bl_path_quad[interp_id][0] = 0.0
                    bl_path_quad[interp_id][1] = 0.0
                    bl_path_quad[interp_id][2] = 0.0
                elif stage_id == 1:
                    fr_path_quad[interp_id][0] = self.amplitudeX * cos(pi / 2 * tick_cnt / num_ticks) * 2.0
                    fr_path_quad[interp_id][1] = 0.0
                    fr_path_quad[interp_id][2] = 0.0
                    bl_path_quad[interp_id][0] = -self.amplitudeX * sin(pi / 2 * tick_cnt / num_ticks) * 2.0
                    bl_path_quad[interp_id][1] = 0.0
                    bl_path_quad[interp_id][2] = 0.0
                elif stage_id == 2:
                    fr_path_quad[interp_id][0] = 0.0
                    fr_path_quad[interp_id][1] = 0.0
                    fr_path_quad[interp_id][2] = 0.0
                    bl_path_quad[interp_id][0] = -self.amplitudeX * cos(pi * tick_cnt / num_ticks) * 2.0
                    bl_path_quad[interp_id][1] = 0.0
                    bl_path_quad[interp_id][2] = abs(self.amplitudeZ) * sin(pi * tick_cnt / num_ticks) * -1.5
                elif stage_id == 3:
                    fr_path_quad[interp_id][0] = 0.0
                    fr_path_quad[interp_id][1] = 0.0
                    fr_path_quad[interp_id][2] = 0.0
                    bl_path_quad[interp_id][0] = self.amplitudeX * 2.0
                    bl_path_quad[interp_id][1] = 0.0
                    bl_path_quad[interp_id][2] = 0.0
                elif stage_id == 4:
                    fr_path_quad[interp_id][0] = -self.amplitudeX * sin(pi / 2 * tick_cnt / num_ticks) * 2.0
                    fr_path_quad[interp_id][1] = 0.0
                    fr_path_quad[interp_id][2] = 0.0
                    bl_path_quad[interp_id][0] = self.amplitudeX * cos(pi / 2 * tick_cnt / num_ticks) * 2.0
                    bl_path_quad[interp_id][1] = 0.0
                    bl_path_quad[interp_id][2] = 0.0
                elif stage_id == 5:
                    fr_path_quad[interp_id][0] = -self.amplitudeX * 2.0
                    fr_path_quad[interp_id][1] = 0.0
                    fr_path_quad[interp_id][2] = 0.0
                    bl_path_quad[interp_id][0] = 0.0
                    bl_path_quad[interp_id][1] = 0.0
                    bl_path_quad[interp_id][2] = 0.0

        def left_rotate_path(path, rotation_num):
            return path[rotation_num:] + path[:rotation_num]

        from copy import deepcopy
        fl_path_quad = deepcopy(left_rotate_path(fr_path_quad, int(len(fr_path_quad) / 2)))
        br_path_quad = deepcopy(left_rotate_path(bl_path_quad, int(len(bl_path_quad) / 2)))

        return self._formated_path_status(fr_path_quad, br_path_quad, bl_path_quad, fl_path_quad, move_status)


class QuadModel(RobotPathModel):
    """四足多步态离线轨迹模型，直接内置 4 种步态"""

    LEG_COUNT = 4

    def __init__(self):
        # 这里的默认站立位置只用于生成与扩展轨迹，在固件侧会有对应的宏定义
        # 先采用一个对称、易理解的矩形布局（单位：mm）
        standby_z = -80.0
        offset_x = 80.0
        offset_y = 80.0
        # FR, BR, BL, FL
        self.home_x = [offset_x, -offset_x, -offset_x, offset_x]
        self.home_y = [-offset_y, -offset_y, offset_y, offset_y]
        self.home_z = [standby_z, standby_z, standby_z, standby_z]
        self.gait = QuadGait(self.home_x, self.home_y, self.home_z, frame_time_ms=20)

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
            dur = len(frames_fwd) * self.gait.frame_time_ms
            entries = [0, len(frames_fwd) // 2] if len(frames_fwd) >= 2 else [0]

            def make_variant(transform_fn):
                data_var = [[[0.0, 0.0, 0.0] for _ in range(len(frames_fwd))] for _ in range(self.LEG_COUNT)]
                for leg in range(self.LEG_COUNT):
                    for i in range(len(frames_fwd)):
                        data_var[leg][i] = transform_fn(leg, data_fwd[leg][i])
                return data_var

            # forward: 直接使用基准轨迹
            results[f"{base_name}_forward"] = (data_fwd, "shift_quad", dur, entries)

            # backward: 关于 Y 轴对称 (x -> -x)
            results[f"{base_name}_backward"] = (
                make_variant(lambda leg, v: [-v[0], v[1], v[2]]),
                "shift_quad",
                dur,
                entries,
            )

            # 左右平移：整体绕 Z 轴旋转 ±90°
            results[f"{base_name}_shiftleft"] = (
                make_variant(lambda leg, v: rot_z(v, -90.0)),
                "shift_quad",
                dur,
                entries,
            )
            results[f"{base_name}_shiftright"] = (
                make_variant(lambda leg, v: rot_z(v, 90.0)),
                "shift_quad",
                dur,
                entries,
            )

            # 左右转向：对每条腿施加不同旋转角度（参考 gait.py: formated_path_status）
            left_angles = {0: 315.0, 1: 45.0, 2: 135.0, 3: 225.0}
            right_angles = {0: 135.0, 1: 225.0, 2: 315.0, 3: 45.0}

            results[f"{base_name}_turnleft"] = (
                make_variant(lambda leg, v: rot_z(v, left_angles[leg])),
                "shift_quad",
                dur,
                entries,
            )
            results[f"{base_name}_turnright"] = (
                make_variant(lambda leg, v: rot_z(v, right_angles[leg])),
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
            result += "    {" + ", ".join(
                "{{Q{idx}X+({x:.2f}), Q{idx}Y+({y:.2f}), Q{idx}Z+({z:.2f})}}".format(
                    x=data[leg][i][0], y=data[leg][i][1], z=data[leg][i][2], idx=leg + 1
                )
                for leg in range(self.LEG_COUNT)
            ) + "},\n"
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
        return """const QuadMovementTable& {name}Table() {{
    return {name}_table;
}}""".format(name=path_name)


# ------------------------------
# CLI 入口
# ------------------------------

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='pathTool: generate robot paths')
    parser.add_argument('--robot', metavar='ROBOT', dest='robot', default='hexapod',
                        choices=['hexapod', 'quad'],
                        help='robot model: hexapod or quad (default: hexapod)')
    parser.add_argument('--pathDir', metavar='DIR', dest='path_dir', default='path',
                        help='path script directory for hexapod (default: path)')
    parser.add_argument('--outPath', metavar='PATH', dest='out_path', default='output/movement_table.h',
                        help='output header path (default: output/movement_table.h)')
    args = parser.parse_args()

    if args.robot == 'hexapod':
        # 保持原有行为：从 pathDir 中搜集脚本并生成六足 movement_table.h
        sys.path.insert(0, args.path_dir)
        model = HexapodModel()
        paths = model.collect_path(args.path_dir)
        results = {path: generator() for path, generator in paths.items()}

        verified = [1 for path, data in results.items() if not model.verify_path(path, data)]
        if len(verified) > 0:
            print("There were errors, exit...")
            sys.exit(1)

        with open(args.out_path, "w") as f:
            print("//", file=f)
            print("// This file is generated, dont directly modify content...", file=f)
            print("//", file=f)
            print("namespace {", file=f)
            for path, data in results.items():
                print(model.generate_c_body(path, data), file=f)
            print("}\n", file=f)
            for path in results:
                print(model.generate_c_def(path), file=f)

        print("Hexapod result written to {}".format(args.out_path))

    elif args.robot == 'quad':
        # 生成四足多步态离线动作表，建议 outPath 设为 output/movement_table_quad.h
        model = QuadModel()
        results = model.generate_all_gaits()

        with open(args.out_path, "w") as f:
            print("//", file=f)
            print("// This file is generated for Quad robot, dont directly modify content...", file=f)
            print("//", file=f)
            print("#pragma once", file=f)
            print("", file=f)
            print("struct QuadLocations {", file=f)
            print("    Point3D p[4];", file=f)
            print("};", file=f)
            print("", file=f)
            print("struct QuadMovementTable {", file=f)
            print("    const QuadLocations* table;", file=f)
            print("    int length;", file=f)
            print("    int stepDuration;", file=f)
            print("    const int* entries;", file=f)
            print("    int entriesCount;", file=f)
            print("};", file=f)
            print("", file=f)
            print("namespace quad {", file=f)
            for path, data in results.items():
                print(model.generate_c_body(path, data), file=f)
            print("}\n", file=f)
            for path in results:
                print(model.generate_c_def(path), file=f)

        print("Quad result written to {}".format(args.out_path))


