"""一次完整的威慑攻击；通过现有 shift 模式生成离线动作表。

+Y 为前方，0/5 是前腿。中后四足始终采用同一机身平移，保持触地点不动。
全部关节计算在生成阶段完成；固件使用普通 MovementTable 插值播放。
"""
import math

import config
from path.performance_pose import generate

# Authored timeline; generation compresses its playback duration to about 2/3.
DURATION_MS = 6540
SHOULDER_DEG = 60.0
KNEE_DEG = 30.0  # 第二、第三关节之和为 90°，小腿水平。
HELD_HIP_DEG = 22.5  # 左右镜像后，两条前腿朝向的夹角为 45°。
FIRST_SHIFT_MM = 26.0
SECOND_SHIFT_MM = 34.0


def smooth(value):
    value = min(1.0, max(0.0, value))
    return value * value * (3.0 - 2.0 * value)


def front_tip(leg, hip_deg, knee_deg):
    """与固件 Leg 的 FK、45/135°安装变换保持一致。"""
    hip = math.radians(hip_deg)
    shoulder = math.radians(SHOULDER_DEG)
    tibia = math.radians(SHOULDER_DEG + knee_deg - 90.0)
    reach = (config.kLegJoint1ToJoint2
             + config.kLegJoint2ToJoint3 * math.cos(shoulder)
             + config.kLegJoint3ToTip * math.cos(tibia))
    x = config.kLegRootToJoint1 + reach * math.cos(hip)
    y = reach * math.sin(hip)
    z = (config.kLegJoint2ToJoint3 * math.sin(shoulder)
         + config.kLegJoint3ToTip * math.sin(tibia))
    right_x = config.kLegMountOtherX + config.COS45 * (x - y)
    return (right_x if leg == 0 else -right_x,
            config.kLegMountOtherY + config.SIN45 * (x + y), z)


def sample(time_ms):
    back, lower, lift = 0.0, 0.0, 0.0
    hip, knee = HELD_HIP_DEG, KNEE_DEG
    # 第 0 帧仍为待机姿态；入口平滑交给固件正常的模式切换插值。
    if time_ms < 500:
        u = smooth(time_ms / 500)
        back, lower = 18.0 * u, 4.0 * u
    elif time_ms < 1200:
        lift = smooth((time_ms - 500) / 700)
        back, lower = 18.0 + (FIRST_SHIFT_MM - 18.0) * lift, 4.0
    elif time_ms < 3100:
        back, lower, lift = FIRST_SHIFT_MM, 4.0, 1.0
        if time_ms < 2800:
            wave = math.sin(math.pi * 4 * (time_ms - 1200) / 1600)
            hip -= 40.0 * wave * wave  # 髋关节水平开合四轮。
        # 2800..3100 ms：保持 45°夹角。
    elif time_ms < 3500:
        u = smooth((time_ms - 3100) / 400)
        back = FIRST_SHIFT_MM + (SECOND_SHIFT_MM - FIRST_SHIFT_MM) * u
        lower, lift = 4.0 + 2.0 * u, 1.0
    elif time_ms < 4940:
        back, lower, lift = SECOND_SHIFT_MM, 6.0, 1.0
        wave = math.sin(math.pi * 4 * (time_ms - 3500) / 1440)
        knee -= 30.0 * wave * wave  # 仅第三关节上下戳击四轮。
    elif time_ms < 5640:
        back, lower = SECOND_SHIFT_MM, 6.0
        lift = 1.0 - smooth((time_ms - 4940) / 700)
    else:
        # 前腿先落地，再恢复机身；最后 300 ms 保持待机，吸收播放计时误差。
        u = 1.0 - smooth((time_ms - 5640) / 600)
        back, lower = SECOND_SHIFT_MM * u, 6.0 * u

    offsets = []
    for leg, standby in enumerate(config.defaultPosition):
        tip = [standby[0], standby[1] + back, standby[2] + lower]
        if leg in (0, 5):
            raised = front_tip(leg, hip, knee)
            tip = [a + (b - a) * lift for a, b in zip(tip, raised)]
        offsets.append(tuple(a - b for a, b in zip(tip, standby)))
    return offsets


def path_generator():
    return generate(sample, DURATION_MS, duration_scale=2/3)
