from pathlib import Path
import re

# 六足生成/IK 校验与固件共用机构尺寸，避免生成的关节姿态随旧常量漂移。
_firmware_config = (Path(__file__).resolve().parents[3]
                    / "firmware/include/config.h").read_text()


def _dimension(name):
    match = re.search(r"const\s+float\s+" + name + r"\s*=\s*([0-9.]+)\s*;", _firmware_config)
    if not match:
        raise ValueError(f"Missing numeric firmware geometry constant: {name}")
    return float(match.group(1))


kLegMountLeftRightX = _dimension("kLegMountLeftRightX")
kLegMountOtherX = _dimension("kLegMountOtherX")
kLegMountOtherY = _dimension("kLegMountOtherY")
kLegRootToJoint1 = _dimension("kLegRootToJoint1")
kLegJoint1ToJoint2 = _dimension("kLegJoint1ToJoint2")
kLegJoint2ToJoint3 = _dimension("kLegJoint2ToJoint3")
kLegJoint3ToTip = _dimension("kLegJoint3ToTip")

SIN30   = 0.5
COS30   = 0.866
SIN45   = 0.7071
COS45   = 0.7071
SIN15   = 0.2588
COS15   = 0.9659

STANDBY_Z = (kLegJoint3ToTip*COS15-kLegJoint2ToJoint3*SIN30)
LEFTRIGHT_X = (kLegMountLeftRightX+kLegRootToJoint1+kLegJoint1ToJoint2+(kLegJoint2ToJoint3*COS30)+kLegJoint3ToTip*SIN15)
OTHER_X = (kLegMountOtherX + (kLegRootToJoint1+kLegJoint1ToJoint2+(kLegJoint2ToJoint3*COS30)+kLegJoint3ToTip*SIN15)*COS45)
OTHER_Y = (kLegMountOtherY + (kLegRootToJoint1+kLegJoint1ToJoint2+(kLegJoint2ToJoint3*COS30)+kLegJoint3ToTip*SIN15)*SIN45)

defaultPosition = (
    (OTHER_X, OTHER_Y, -STANDBY_Z),
    (LEFTRIGHT_X, 0, -STANDBY_Z),
    (OTHER_X, -OTHER_Y, -STANDBY_Z),
    (-OTHER_X, -OTHER_Y, -STANDBY_Z),
    (-LEFTRIGHT_X, 0, -STANDBY_Z),
    (-OTHER_X, OTHER_Y, -STANDBY_Z),
)

mountPosition = (
    (kLegMountOtherX, kLegMountOtherY, 0),
    (kLegMountLeftRightX, 0, 0),
    (kLegMountOtherX, -kLegMountOtherY, 0),
    (-kLegMountOtherX, -kLegMountOtherY, 0),
    (-kLegMountLeftRightX, 0, 0),
    (-kLegMountOtherX, kLegMountOtherY, 0),
)

defaultAngle = (
    -45, 0, 45, 135, 180, 225
)

angleLimitation = (
    (-45, 45),
    (-45, 75),
    (-60, 60),
)
