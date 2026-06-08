#
# Gate geometry helpers.
#
# Turns the sim's ground-truth track data + drone pose into the relative
# position of a gate in the DRONE'S BODY FRAME: how far ahead, how far left/
# right, how far up/down, plus bearing/elevation angles.
#
# This is ground-truth ("privileged") info from the sim - the basis for an
# oracle pilot that flies the course to collect data, and for auto-labeling.
# The eventual vision pilot must estimate these same quantities from the
# camera instead (detect gate -> PnP), since the real race has no absolute
# coordinates.
#
# Coordinate frames (MAVLink convention):
#   NED world frame: x=North, y=East, z=Down
#   Body frame:      x=forward, y=right, z=down
#   Attitude quaternion q=(w,x,y,z) rotates BODY -> WORLD.
#

import math


def quat_to_rotmat(q):
    # rotation matrix R such that v_world = R @ v_body  (body -> world)
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ]


def world_to_body(q, v_world):
    # v_body = R^T @ v_world  (transpose of body->world is world->body)
    r = quat_to_rotmat(q)
    return [
        r[0][0] * v_world[0] + r[1][0] * v_world[1] + r[2][0] * v_world[2],
        r[0][1] * v_world[0] + r[1][1] * v_world[1] + r[2][1] * v_world[2],
        r[0][2] * v_world[0] + r[1][2] * v_world[1] + r[2][2] * v_world[2],
    ]


def get_drone_pose(data):
    # prefer ODOMETRY (position + quaternion); returns (pos_ned, quat) or None
    odo = data.get("odometry")
    if odo is not None:
        return (odo["x"], odo["y"], odo["z"]), (odo["qw"], odo["qx"], odo["qy"], odo["qz"])
    return None


def relative_gate(drone_pos, drone_quat, gate):
    """Relative geometry of one gate, in the drone's body frame.

    Returns a dict:
      forward / right / down : metres in body frame
      distance               : straight-line distance (m)
      azimuth_deg            : horizontal bearing, 0 = dead ahead, + = gate to the right
      elevation_deg          : + = gate is above the drone
    """
    rel_world = [
        gate["position_ned"][0] - drone_pos[0],
        gate["position_ned"][1] - drone_pos[1],
        gate["position_ned"][2] - drone_pos[2],
    ]
    forward, right, down = world_to_body(drone_quat, rel_world)
    horizontal = math.hypot(forward, right)
    return {
        "gate_id": gate.get("gate_id"),
        "forward": forward,
        "right": right,
        "down": down,
        "distance": math.sqrt(forward * forward + right * right + down * down),
        "azimuth_deg": math.degrees(math.atan2(right, forward)),
        "elevation_deg": math.degrees(math.atan2(-down, horizontal)),
    }


def active_gate_relative(data):
    """Relative geometry of the CURRENT target gate, read from shared_data.

    Needs odometry, the gate list, and (optionally) race_status for the active
    index. Returns None until all of those have arrived.
    """
    pose = get_drone_pose(data)
    gates = data.get("gates")
    if pose is None or not gates:
        return None

    race_status = data.get("race_status")
    idx = race_status["active_gate_index"] if race_status else 0

    gate = next((g for g in gates if g.get("gate_id") == idx), None)
    if gate is None:
        if 0 <= idx < len(gates):
            gate = gates[idx]
        else:
            return None

    return relative_gate(pose[0], pose[1], gate)
