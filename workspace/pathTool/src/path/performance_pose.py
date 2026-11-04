"""Offline choreography helpers; never imported by the firmware.

World +Y is forward and Z=0 is the floor. A fixed-foot body transform prevents
support-foot sliding during translations, yaw and bows. Only front legs 0/5
are released; all other feet retain their original world anchors.
"""
import math
import config

STEP_MS = 20


def smooth(u):
    u = max(0.0, min(1.0, u))
    return u*u*(3.0-2.0*u)


def lerp(a, b, u):
    return tuple(x+(y-x)*u for x, y in zip(a, b))


def track(t, keys):
    """C1-continuous interpolation between time/value keyframes."""
    for (start, a), (end, b) in zip(keys, keys[1:]):
        if t < end:
            return lerp(a, b, smooth((t-start)/(end-start)))
    return keys[-1][1]


def envelope(t, start, raised, lower, end):
    return smooth((t-start)/(raised-start))*(1-smooth((t-lower)/(end-lower)))


def body_rotation(pitch, yaw):
    """Rz(yaw) Rx(pitch); negative pitch bows the +Y-facing nose."""
    p, y = math.radians(pitch), math.radians(yaw)
    c, s, cp, sp = math.cos(y), math.sin(y), math.cos(p), math.sin(p)
    return ((c, -s*cp, s*sp), (s, c*cp, -c*sp), (0, sp, cp))


def to_body(world, body):
    x, y, dz, pitch, yaw = body
    delta = (world[0]-x, world[1]-y, world[2]-config.STANDBY_Z-dz)
    r = body_rotation(pitch, yaw)
    return tuple(sum(r[j][i]*delta[j] for j in range(3)) for i in range(3))


def front_fk(leg, hip, shoulder, knee):
    """Mirrored front-leg FK; hip is the right-leg convention for both legs."""
    h, s, k = map(math.radians, (hip, shoulder, shoulder+knee-90))
    reach = config.kLegJoint1ToJoint2 + config.kLegJoint2ToJoint3*math.cos(s) + config.kLegJoint3ToTip*math.cos(k)
    x, y = config.kLegRootToJoint1+reach*math.cos(h), reach*math.sin(h)
    z = config.kLegJoint2ToJoint3*math.sin(s)+config.kLegJoint3ToTip*math.sin(k)
    right = config.kLegMountOtherX+config.COS45*(x-y)
    return (right if leg == 0 else -right, config.kLegMountOtherY+config.SIN45*(x+y), z)


def pose(body, joints=None, probes=None):
    """joints: leg -> (blend, hip, shoulder, knee); probes: world XYZ offsets."""
    result = []
    for leg, rest in enumerate(config.defaultPosition):
        offset = (probes or {}).get(leg, (0, 0, 0))
        world = (rest[0]+offset[0], rest[1]+offset[1], offset[2])
        tip = to_body(world, body)
        if joints and leg in joints:
            weight, hip, shoulder, knee = joints[leg]
            tip = lerp(tip, front_fk(leg, hip, shoulder, knee), weight)
        result.append(tuple(a-b for a, b in zip(tip, rest)))
    return result


def generate(sample, duration, duration_scale=1.0):
    """Resample the authored curve at a new tempo; retain 20 ms playback ticks.

    Round the playback duration up to a whole frame and map that interval onto
    the complete source timeline. Geometry and phase/repetition definitions
    stay in sample(); this changes sampling density, never the runtime player.
    """
    if duration <= 0 or not math.isfinite(duration_scale) or duration_scale <= 0:
        raise ValueError('duration and duration_scale must be positive')
    count = math.ceil(duration * duration_scale / STEP_MS)
    frames = [sample(i * duration / count) for i in range(count)]
    return [[f[leg] for f in frames] for leg in range(6)], 'shift', STEP_MS, [0]
