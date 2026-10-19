"""Track a path that has cusps: one segment at a time, stopping at every cusp.

Each segment gets its own controller. A segment starts from rest: the steering first
settles toward the new controller's command, then the vehicle accelerates to the cruise
speed in the segment's gear. Near the end of the segment the speed follows a braking
profile (the speed that can still stop at the end with the set deceleration, never below
a creep speed), and the vehicle brakes to a standstill at the cusp.
"""
import math
from dataclasses import dataclass, field

import numpy as np

from trackloop.models import kinematic as kin
from trackloop.paths import cross_track_error
from trackloop.projection import project
from trackloop.segments import split_at_cusps


@dataclass(frozen=True)
class RunnerParams:
    speed: float = 1.5            # m/s, cruise speed in both gears
    accel: float = 1.0            # m/s^2, acceleration and braking
    v_floor: float = 0.2          # m/s, creep speed on the way into a cusp
    arrive: float = 0.03          # m, distance to the segment end at which braking starts
    settle: float = 0.02          # rad, steering error that counts as settled
    settle_timeout: float = 5.0   # s
    dt: float = 0.05
    max_time: float = 300.0


@dataclass
class RunResult:
    states: np.ndarray            # (n + 1, 5): x, y, heading, speed, steering
    time: np.ndarray
    segment: np.ndarray           # index of the active segment at each sample
    cte: np.ndarray               # signed rear-axle error against the active segment
    cusps: list = field(default_factory=list)   # one dict per cusp
    finished: bool = False
    end_position_error: float = math.nan
    end_heading_error: float = math.nan


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def track_with_cusps(path, make_controller, state0, params=None, kin_params=None):
    """make_controller(segment_path) returns a controller with command(state) and reset()."""
    p = params if params is not None else RunnerParams()
    kp = kin_params if kin_params is not None else kin.KinematicParams()
    state = np.array(state0, dtype=float)
    state[3] = 0.0
    rows, seg_id, cte = [state.copy()], [0], []
    segments = split_at_cusps(path)
    cte.append(cross_track_error(segments[0].path, state[0], state[1]))
    result = RunResult(np.empty(0), np.empty(0), np.empty(0, int), np.empty(0))

    def advance(k, seg, cmd, accel):
        nonlocal state
        state = kin.step(state, cmd, accel, p.dt, kp)
        rows.append(state.copy())
        seg_id.append(k)
        cte.append(cross_track_error(seg.path, state[0], state[1]))

    for k, seg in enumerate(segments):
        ctrl = make_controller(seg.path)
        ctrl.reset()
        settle_time = 0.0
        while settle_time < p.settle_timeout and len(rows) * p.dt < p.max_time:
            cmd = ctrl.command(state)
            if abs(cmd - state[4]) < p.settle:
                break
            advance(k, seg, cmd, 0.0)
            settle_time += p.dt
        if k > 0:
            result.cusps[-1]["settle_time"] = settle_time
        total, index, speed = float(seg.path.s[-1]), 0, 0.0
        arrived = False
        while len(rows) * p.dt < p.max_time:
            proj = project(seg.path, state[0], state[1], index, 300)
            index = proj.index
            remaining = total - proj.s
            if remaining <= p.arrive:
                arrived = True
                break
            target = max(min(p.speed, math.sqrt(2.0 * p.accel * remaining)), p.v_floor)
            speed = float(np.clip(target, speed - p.accel * p.dt, speed + p.accel * p.dt))
            accel = (seg.gear * speed - state[3]) / p.dt
            advance(k, seg, ctrl.command(state), accel)
        if not arrived:
            break
        while speed > 0.0:
            speed = max(0.0, speed - p.accel * p.dt)
            advance(k, seg, ctrl.command(state), (seg.gear * speed - state[3]) / p.dt)
        end = (seg.path.x[-1], seg.path.y[-1], seg.path.heading[-1])
        pos_err = math.hypot(state[0] - end[0], state[1] - end[1])
        head_err = abs(_wrap(state[2] - end[2]))
        if k < len(segments) - 1:
            result.cusps.append({"after_segment": k, "position_error": pos_err,
                                 "heading_error": head_err, "settle_time": math.nan})
        else:
            result.finished = True
            result.end_position_error, result.end_heading_error = pos_err, head_err
    result.states = np.array(rows)
    result.time = np.arange(len(rows)) * p.dt
    result.segment = np.array(seg_id, dtype=int)
    result.cte = np.array(cte)
    return result
