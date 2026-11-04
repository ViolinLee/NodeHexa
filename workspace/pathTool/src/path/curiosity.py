"""好奇探察：伏低张望、轮流探爪轻点、惊讶缩爪、谨慎恢复。"""
import math
from path.performance_pose import envelope, generate, pose, track

# Authored timeline; generation compresses its playback duration to about 2/3.
DURATION_MS = 11100
PHASES = [
    (0, 600, '伏低观察'),
    (600, 1800, '向左张望'),
    (1800, 3000, '向右张望'),
    (3000, 4200, '右爪试探'),
    (4200, 4800, '右爪轻点 ×2'),
    (4800, 6000, '收爪换侧'),
    (6000, 6600, '左爪试探'),
    (6600, 7200, '左爪轻点 ×2'),
    (7200, 7800, '收回前爪'),
    (7800, 8300, '惊讶后缩'),
    (8300, 8800, '缩爪停顿'),
    (8800, 9900, '谨慎落地'),
    (9900, 10700, '恢复站姿'),
    (10700, 11100, '待机结束'),
]
BODY = [
    (0, (0, 0, 0, 0, 0)),
    (600, (0, -10, -12, -3, 0)),
    (1300, (0, -14, -12, -3, 8)),
    (1800, (0, -14, -12, -3, 8)),
    (2500, (0, -14, -12, -3, -8)),
    (3000, (0, -14, -12, -3, -8)),
    (3600, (-8, -22, -10, -2, -4)),
    (5400, (-8, -22, -10, -2, -4)),
    (6000, (8, -22, -10, -2, 4)),
    (7800, (8, -22, -10, -2, 4)),
    (8300, (0, -32, -8, 2, 0)),
    (8800, (0, -32, -8, 2, 0)),
    (9900, (0, -20, -8, 0, 0)),
    (10700, (0, 0, 0, 0, 0)),
]


def probe(t, start, side):
    # Lift before reaching; two gentle taps, then lift before returning.
    v = track(t, [(start,(0,0,0)),(start+250,(0,0,24)),
                  (start+600,(side*8,24,12)),(start+1200,(side*8,24,12)),
                  (start+1450,(side*8,24,28)),(start+1600,(0,0,24)),
                  (start+1800,(0,0,0))])
    if start+600 <= t < start+1200:
        wave = math.sin(math.pi*2*(t-start-600)/600)
        v = (v[0],v[1],12*(1-wave*wave))
    return v


def sample(t):
    probes = {0:probe(t,3600,1),5:probe(t,6000,-1)}
    weight = envelope(t,7800,8300,8800,9500)
    # Tuck rather than strike: flexed wrists, held pause, then a slow release.
    joints = {leg:(weight,28,55,-5) for leg in (0,5)}
    return pose(track(t,BODY), joints=joints, probes=probes)


def path_generator():
    return generate(sample, DURATION_MS, duration_scale=2/3)
