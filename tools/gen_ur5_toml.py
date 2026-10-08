"""Generate crates/motionamigo/robots/ur5.toml from VAMP's spherized UR5 URDF and SRDF.

Usage:
    python tools/gen_ur5_toml.py path/to/vamp/resources/ur5 > crates/motionamigo/robots/ur5.toml

The sphere decomposition comes from resources/ur5/ur5_spherized.urdf of
https://github.com/KavrakiLab/vamp (Apache-2.0), which originates from
robowflex_resources (MIT). It describes a UR5 with a Robotiq FT sensor and a Robotiq 2F-85
gripper held at a fixed opening. Kinematics use the official Universal Robots standard DH
parameters. The URDF link frames do not coincide with the DH frames, so every link frame is
re-expressed in its DH frame: the script builds both chains, checks that the offset between a
link frame and its DH frame is the same for several joint configurations, and transforms the
spheres with that offset. Frame 0 is the DH base frame, which is the URDF base_link rotated by pi
about z (the "base" frame of the Universal Robots controller). The fixed offset_link above
base_link (the pedestal used by MotionBenchMaker) is not part of the model.
"""

import math
import random
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

PI = math.pi
# Universal Robots UR5, standard DH parameters: a, d, alpha.
DH = [
    (0.0, 0.089159, PI / 2),
    (-0.425, 0.0, 0.0),
    (-0.39225, 0.0, 0.0),
    (0.0, 0.10915, PI / 2),
    (0.0, 0.09465, -PI / 2),
    (0.0, 0.0823, 0.0),
]
NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
# Joint limits of VAMP's UR5 URDF (+-pi), so that the MotionBenchMaker problems are valid.
LIMIT = 3.14159265
BASE = "base_link"
TCP_LINK = "tool0"


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def inverse(m):
    r = [[m[j][i] for j in range(3)] for i in range(3)]
    t = [-sum(r[i][k] * m[k][3] for k in range(3)) for i in range(3)]
    return [r[0] + [t[0]], r[1] + [t[1]], r[2] + [t[2]], [0.0, 0.0, 0.0, 1.0]]


def trans(x, y, z):
    return [[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]]


def rot(axis, a):
    """Rotation by angle a about a unit axis (Rodrigues)."""
    x, y, z = axis
    c, s, v = math.cos(a), math.sin(a), 1 - math.cos(a)
    return [
        [c + x * x * v, x * y * v - z * s, x * z * v + y * s, 0],
        [y * x * v + z * s, c + y * y * v, y * z * v - x * s, 0],
        [z * x * v - y * s, z * y * v + x * s, c + z * z * v, 0],
        [0, 0, 0, 1],
    ]


def origin(xyz, rpy):
    r = matmul(rot((0, 0, 1), rpy[2]), matmul(rot((0, 1, 0), rpy[1]), rot((1, 0, 0), rpy[0])))
    return matmul(trans(*xyz), r)


# The DH base frame (the "base" frame of Universal Robots) is base_link rotated by pi about z.
BASE_LINK_TO_DH = rot((0, 0, 1), PI)


def apply(m, p):
    return tuple(sum(m[i][k] * p[k] for k in range(3)) + m[i][3] for i in range(3))


def dh_frames(q):
    frames = [trans(0, 0, 0)]
    for (a, d, alpha), qi in zip(DH, q):
        t = matmul(rot((0, 0, 1), qi), matmul(trans(a, 0, d), rot((1, 0, 0), alpha)))
        frames.append(matmul(frames[-1], t))
    return frames


def floats(s, default):
    return tuple(float(v) for v in s.split()) if s else default


def parse_joints(urdf):
    """child link -> (parent link, origin matrix, axis or None for fixed, name)."""
    joints = {}
    for j in urdf.findall("joint"):
        o = j.find("origin")
        xyz = floats(o.get("xyz"), (0.0, 0.0, 0.0)) if o is not None else (0.0, 0.0, 0.0)
        rpy = floats(o.get("rpy"), (0.0, 0.0, 0.0)) if o is not None else (0.0, 0.0, 0.0)
        axis = None
        if j.get("type") == "revolute":
            axis = floats(j.find("axis").get("xyz"), (1.0, 0.0, 0.0))
        joints[j.find("child").get("link")] = (
            j.find("parent").get("link"),
            origin(xyz, rpy),
            axis,
            j.get("name"),
        )
    return joints


def urdf_pose(joints, link, q):
    """Pose of `link` relative to base_link and the number of revolute joints above it."""
    chain = []
    while link != BASE:
        chain.append(joints[link])
        link = joints[link][0]
    m, frame = trans(0, 0, 0), 0
    for _, o, axis, name in reversed(chain):
        m = matmul(m, o)
        if axis is not None:
            assert name == NAMES[frame], name
            m = matmul(m, rot(axis, q[frame]))
            frame += 1
    return m, frame


def offset(joints, link):
    """Constant transform from the DH frame of `link` to its URDF frame."""
    rng = random.Random(0)
    offsets = []
    for _ in range(5):
        q = [rng.uniform(-PI, PI) for _ in range(6)]
        m, frame = urdf_pose(joints, link, q)
        offsets.append(matmul(inverse(matmul(BASE_LINK_TO_DH, dh_frames(q)[frame])), m))
    for o in offsets[1:]:
        err = max(abs(o[i][j] - offsets[0][i][j]) for i in range(3) for j in range(4))
        assert err < 1e-6, f"{link}: URDF and DH chains disagree ({err})"
    return offsets[0], frame


def always_colliding(a, b):
    """True for link pairs that cannot move relative to each other and touch, like the force
    torque sensor and the wrist_2 link (both centered on the wrist_3 axis). Like MoveIt's
    "always in collision" rule and VAMP's generated UR5 model, such pairs are not checked."""
    rng = random.Random(1)
    for _ in range(200):
        frames = dh_frames([rng.uniform(-PI, PI) for _ in range(6)])
        if not any(
            math.dist(apply(frames[a[1]], s[:3]), apply(frames[b[1]], t[:3])) < s[3] + t[3]
            for s in a[2]
            for t in b[2]
        ):
            return False
    return True


def rpy_of(m):
    """Inverse of `origin` for the rotation part (R = Rz(yaw) Ry(pitch) Rx(roll))."""
    pitch = math.asin(-max(-1.0, min(1.0, m[2][0])))
    roll = math.atan2(m[2][1], m[2][2])
    yaw = math.atan2(m[1][0], m[0][0])
    return roll, pitch, yaw


def fmt(v):
    s = f"{v:.6f}".rstrip("0").rstrip(".")
    return "0.0" if s in ("-0", "0") else (s if "." in s else s + ".0")


def main(resources):
    urdf = ET.parse(Path(resources) / "ur5_spherized.urdf").getroot()
    srdf = ET.parse(Path(resources) / "ur5.srdf").getroot()
    joints = parse_joints(urdf)

    links = []
    for link in urdf.findall("link"):
        name = link.get("name")
        spheres = []
        for col in link.findall("collision"):
            sph = col.find("geometry/sphere")
            if sph is None:
                continue
            xyz = floats(col.find("origin").get("xyz"), (0.0, 0.0, 0.0))
            spheres.append((xyz, float(sph.get("radius"))))
        if not spheres:
            continue
        off, frame = offset(joints, name)
        links.append((name, frame, [(*apply(off, p), r) for p, r in spheres]))
    links.sort(key=lambda l: l[1])

    disabled = {
        frozenset((d.get("link1"), d.get("link2"))) for d in srdf.findall("disable_collisions")
    }
    pairs = [
        (a[0], b[0])
        for i, a in enumerate(links)
        for b in links[i + 1:]
        # Links on the same frame are rigidly attached, their distance never changes.
        if a[1] != b[1]
        and frozenset((a[0], b[0])) not in disabled
        and not always_colliding(a, b)
    ]

    tcp, tcp_frame = offset(joints, TCP_LINK)
    assert tcp_frame == 6

    out = []
    out.append("# Universal Robots UR5 with a Robotiq 2F-85 gripper for motionAmigo.")
    out.append("# Generated by tools/gen_ur5_toml.py.")
    out.append("# Kinematics: official Universal Robots standard DH parameters, joint limits +-pi")
    out.append("# as in VAMP's UR5 URDF.")
    out.append("# Collision spheres: derived from VAMP resources/ur5/ur5_spherized.urdf")
    out.append("# (Apache-2.0, https://github.com/KavrakiLab/vamp), originally from")
    out.append("# robowflex_resources (MIT, https://github.com/KavrakiLab/robowflex_resources).")
    out.append("# Self-collision pairs: sphere-carrying link pairs on different frames that are not disabled")
    out.append("# in ur5.srdf and not in collision in every configuration (the 55 pairs VAMP checks).")
    out.append("")
    out.append('name = "ur5"')
    out.append('convention = "dh"')
    out.append("")
    out.append("self_collision = [")
    for a, b in pairs:
        out.append(f'  ["{a}", "{b}"],')
    out.append("]")
    out.append("")
    for name, (a, d, alpha) in zip(NAMES, DH):
        out.append("[[joints]]")
        out.append(f'name = "{name}"')
        out.append(f"a = {fmt(a)}")
        out.append(f"d = {fmt(d)}")
        out.append(f"alpha = {repr(alpha) if alpha else '0.0'}")
        out.append(f"lower = {-LIMIT!r}")
        out.append(f"upper = {LIMIT!r}")
        out.append("")
    out.append("# Tool center point: the tool0 flange frame of the URDF.")
    out.append("[tcp]")
    out.append("xyz = [" + ", ".join(fmt(tcp[i][3]) for i in range(3)) + "]")
    out.append("rpy = [" + ", ".join(fmt(v) for v in rpy_of(tcp)) + "]")
    out.append("")
    for name, frame, spheres in links:
        out.append("[[links]]")
        out.append(f'name = "{name}"')
        out.append(f"frame = {frame}")
        out.append("spheres = [")
        for s in spheres:
            out.append("  [" + ", ".join(fmt(v) for v in s) + "],")
        out.append("]")
        out.append("")
    print("\n".join(out))


if __name__ == "__main__":
    main(sys.argv[1])
