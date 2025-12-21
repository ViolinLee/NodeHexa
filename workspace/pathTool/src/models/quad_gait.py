from __future__ import annotations

from math import cos, pi, sin


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

    def _formated_path_status(self, fr_path_quad, fl_path_quad, bl_path_quad, br_path_quad, move_status):
        # path_quad: N x 4 x 3 （按 FR, FL, BL, BR 顺序，与固件 legs_[] 对齐）
        path_quad = [
            [fr_path_quad[path_id], fl_path_quad[path_id], bl_path_quad[path_id], br_path_quad[path_id]]
            for path_id in range(len(fr_path_quad))
        ]
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
                [
                    points_set[0][0] + self.home_x[0],
                    points_set[0][1] + self.home_y[0],
                    points_set[0][2] + self.home_z[0],
                ],
                [
                    points_set[1][0] + self.home_x[1],
                    points_set[1][1] + self.home_y[1],
                    points_set[1][2] + self.home_z[1],
                ],
                [
                    points_set[2][0] + self.home_x[2],
                    points_set[2][1] + self.home_y[2],
                    points_set[2][2] + self.home_z[2],
                ],
                [
                    points_set[3][0] + self.home_x[3],
                    points_set[3][1] + self.home_y[3],
                    points_set[3][2] + self.home_z[3],
                ],
            ]
            for points_set in corrected_path
        ]
        return path_quad_world

    def trot_gait(self, move_status, gait_speed=0):
        stages = 2
        duration = self.frame_time_ms * 20 if gait_speed == 0 else self.frame_time_ms * 40
        num_ticks = int(duration / self.frame_time_ms / 2)
        fr_path_quad = [[0.0, 0.0, 0.0] for _ in range(num_ticks * 2)]

        for stage_id in range(stages):
            for tick_cnt in range(num_ticks):
                interp_id = stage_id * num_ticks + tick_cnt
                if stage_id == 0:
                    fr_path_quad[interp_id][0] = -self.amplitudeX * cos(pi * tick_cnt / num_ticks)
                    fr_path_quad[interp_id][1] = 0.0
                    # Z 抬腿：与 hexapod 一致，抬腿应为“向上”（使 Z 增大）
                    fr_path_quad[interp_id][2] = abs(self.amplitudeZ) * sin(pi * tick_cnt / num_ticks)
                elif stage_id == 1:
                    fr_path_quad[interp_id][0] = self.amplitudeX * cos(pi * tick_cnt / num_ticks)
                    fr_path_quad[interp_id][1] = 0.0
                    fr_path_quad[interp_id][2] = 0.0

        # Deep copy & 相位偏移
        # 对齐 nodequad 实现：FR 基准，BR/BL/FL 做旋转移位
        from copy import deepcopy

        def right_rotate_path(path, rotation_num):
            return path[(len(path) - rotation_num) :] + path[: (len(path) - rotation_num)]

        fl_path_quad = deepcopy(right_rotate_path(fr_path_quad, num_ticks))
        bl_path_quad = deepcopy(fr_path_quad)
        br_path_quad = deepcopy(fl_path_quad)

        return self._formated_path_status(fr_path_quad, fl_path_quad, bl_path_quad, br_path_quad, move_status)

    def walk_gait(self, move_status, gait_speed=0):
        stages = 4
        duration = 4 * 4 * self.frame_time_ms if gait_speed == 0 else 1280
        num_ticks = int(duration / self.frame_time_ms / 4)
        fl_path_quad = [[0.0, 0.0, 0.0] for _ in range(num_ticks * 4)]

        for stage_id in range(stages):
            for tick_cnt in range(num_ticks):
                interp_id = stage_id * num_ticks + tick_cnt
                if stage_id == 0:
                    fl_path_quad[interp_id][0] = -self.amplitudeX * cos(pi * tick_cnt / num_ticks) * 1.5
                    fl_path_quad[interp_id][1] = 0.0
                    fl_path_quad[interp_id][2] = abs(self.amplitudeZ) * sin(pi * tick_cnt / num_ticks)
                elif stage_id in (1, 2, 3):
                    fl_path_quad[interp_id][0] = (
                        self.amplitudeX
                        - self.amplitudeX * 2 * ((stage_id - 1) * num_ticks + tick_cnt) / (3 * num_ticks)
                    ) * 1.5
                    fl_path_quad[interp_id][1] = 0.0
                    fl_path_quad[interp_id][2] = 0.0

        def right_rotate_path(path, rotation_num):
            return path[(len(path) - rotation_num) :] + path[: (len(path) - rotation_num)]

        from copy import deepcopy

        fr_path_quad = deepcopy(right_rotate_path(fl_path_quad, num_ticks * 2))
        br_path_quad = deepcopy(right_rotate_path(fl_path_quad, num_ticks * 1))
        bl_path_quad = deepcopy(right_rotate_path(fl_path_quad, num_ticks * 3))

        return self._formated_path_status(fr_path_quad, fl_path_quad, bl_path_quad, br_path_quad, move_status)

    def gallop_gait(self, move_status, gait_speed=0):
        stages = 4
        duration = 4 * 4 * self.frame_time_ms if gait_speed == 0 else 1280
        num_ticks = int(duration / self.frame_time_ms / 4)
        fl_path_quad = [[0.0, 0.0, 0.0] for _ in range(num_ticks * 4)]

        for stage_id in range(stages):
            for tick_cnt in range(num_ticks):
                interp_id = stage_id * num_ticks + tick_cnt
                if stage_id == 0:
                    fl_path_quad[interp_id][0] = -self.amplitudeX * cos(pi * tick_cnt / num_ticks) * 1.5
                    fl_path_quad[interp_id][1] = 0.0
                    fl_path_quad[interp_id][2] = abs(self.amplitudeZ) * sin(pi * tick_cnt / num_ticks)
                elif stage_id in (1, 2, 3):
                    fl_path_quad[interp_id][0] = (
                        self.amplitudeX
                        - self.amplitudeX * 2 * ((stage_id - 1) * num_ticks + tick_cnt) / (3 * num_ticks)
                    ) * 1.5
                    fl_path_quad[interp_id][1] = 0.0
                    fl_path_quad[interp_id][2] = 0.0

        def right_rotate_path(path, rotation_num):
            return path[(len(path) - rotation_num) :] + path[: (len(path) - rotation_num)]

        from copy import deepcopy

        fr_path_quad = deepcopy(right_rotate_path(fl_path_quad, num_ticks * 1))
        br_path_quad = deepcopy(right_rotate_path(fl_path_quad, num_ticks * 2))
        bl_path_quad = deepcopy(right_rotate_path(fl_path_quad, num_ticks * 3))

        return self._formated_path_status(fr_path_quad, fl_path_quad, bl_path_quad, br_path_quad, move_status)

    def creep_gait(self, move_status, gait_speed=0):
        stages = 6
        duration = 6 * 6 * self.frame_time_ms if gait_speed == 0 else 12 * 6 * self.frame_time_ms
        num_ticks = int(duration / self.frame_time_ms / 6)
        fr_path_quad = [[0.0, 0.0, 0.0] for _ in range(num_ticks * 6)]
        bl_path_quad = [[0.0, 0.0, 0.0] for _ in range(num_ticks * 6)]

        for stage_id in range(stages):
            for tick_cnt in range(num_ticks):
                interp_id = stage_id * num_ticks + tick_cnt
                if stage_id == 0:
                    fr_path_quad[interp_id][0] = -self.amplitudeX * cos(pi * tick_cnt / num_ticks) * 2.0
                    fr_path_quad[interp_id][1] = 0.0
                    fr_path_quad[interp_id][2] = abs(self.amplitudeZ) * sin(pi * tick_cnt / num_ticks) * 1.5
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
                    bl_path_quad[interp_id][2] = abs(self.amplitudeZ) * sin(pi * tick_cnt / num_ticks) * 1.5
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

        return self._formated_path_status(fr_path_quad, fl_path_quad, bl_path_quad, br_path_quad, move_status)

