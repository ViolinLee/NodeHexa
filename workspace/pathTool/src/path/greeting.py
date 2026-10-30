"""迎宾致意：欠身、左右招手、双爪舒展、收礼。六足专属单次表演。"""
import math
from path.performance_pose import envelope, generate, pose, track

# Authored timeline; generation compresses its playback duration to about 2/3.
DURATION_MS = 10900
PHASES = [
    (0, 1200, '欠身迎宾'),
    (1200, 2200, '右爪举起'),
    (2200, 3400, '右爪招手 ×3'),
    (3400, 4400, '落爪换侧'),
    (4400, 5000, '左爪举起'),
    (5000, 6200, '左爪招手 ×3'),
    (6200, 7200, '回正蓄势'),
    (7200, 8300, '双爪舒展'),
    (8300, 9300, '轻放双爪'),
    (9300, 10500, '收礼归位'),
    (10500, 10900, '待机结束'),
]
BODY = [
    (0, (0, 0, 0, 0, 0)),
    (600, (0, -8, -6, -6, 0)),
    (1000, (0, -8, -6, -6, 0)),
    (1600, (-8, -22, -4, 0, -3)),
    (3800, (-8, -22, -4, 0, -3)),
    (4400, (8, -22, -4, 0, 3)),
    (6600, (8, -22, -4, 0, 3)),
    (7200, (0, -28, -5, 0, 0)),
    (8800, (0, -28, -5, 0, 0)),
    (9300, (0, -20, -4, 0, 0)),
    (9900, (0, -6, -7, -5, 0)),
    (10500, (0, 0, 0, 0, 0)),
]


def sample(t):
    joints = {}
    for leg, start in [(0,1600),(5,4400)]:
        weight = envelope(t, start, start+600, start+1800, start+2200)
        u = max(0, min(1, (t-start-600)/1200))
        wave = math.sin(math.pi*3*u)**2
        joints[leg] = (weight, 10-28*wave, 55, 25+8*wave)
    both = envelope(t, 7200, 7700, 8300, 8800)
    if both > 0:
        hip = track(t, [(7200,(22,)),(7700,(22,)),(8300,(-14,)),(8800,(-14,))])[0]
        joints = {leg:(both,hip,52,28) for leg in (0,5)}
    return pose(track(t,BODY), joints=joints)


def path_generator():
    return generate(sample, DURATION_MS, duration_scale=2/3)
