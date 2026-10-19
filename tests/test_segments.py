import numpy as np
import pytest

from trackloop import paths
from trackloop.rs_export import rs_to_path
from trackloop.segments import split_at_cusps

START = (0.0, 0.0, 0.0)
SEGS = [("L", 1.0), ("S", -1.0), ("R", 0.5)]


def full_path():
    return rs_to_path(START, SEGS, radius=4.0)[0]


def test_path_without_cusps_is_one_segment():
    p = paths.straight(20.0)
    (seg,) = split_at_cusps(p)
    assert seg.gear == 1 and seg.first == 0 and seg.s_start == 0.0
    assert np.array_equal(seg.path.x, p.x) and np.array_equal(seg.path.s, p.s)


def test_reverse_only_path_is_one_reverse_segment():
    p, _ = rs_to_path(START, [("S", -5.0)], radius=2.0)
    (seg,) = split_at_cusps(p)
    assert seg.gear == -1 and np.all(seg.path.direction == -1.0)


def test_two_cusps_give_three_segments_in_alternating_gears():
    segs = split_at_cusps(full_path())
    assert [s.gear for s in segs] == [1, -1, 1]
    for s in segs:
        assert np.all(s.path.direction == s.gear)
        assert s.path.s[0] == 0.0 and np.all(np.diff(s.path.s) > 0.0)


def test_neighbours_share_the_cusp_sample_and_lengths_add_up():
    p = full_path()
    segs = split_at_cusps(p)
    for a, b in zip(segs[:-1], segs[1:]):
        assert (a.path.x[-1], a.path.y[-1], a.path.heading[-1]) == (
            b.path.x[0], b.path.y[0], b.path.heading[0])
    assert sum(s.path.s[-1] for s in segs) == pytest.approx(p.s[-1])


def test_offsets_point_back_into_the_full_path():
    p = full_path()
    for s in split_at_cusps(p):
        n = len(s.path)
        assert np.allclose(s.path.s + s.s_start, p.s[s.first:s.first + n])
        assert np.array_equal(s.path.x, p.x[s.first:s.first + n])


def test_ending_segment_keeps_its_own_curvature_at_the_cusp():
    p = full_path()
    first, second, _ = split_at_cusps(p)
    cusp = second.first
    assert p.curvature[cusp] == 0.0 and p.curvature[cusp - 1] == pytest.approx(0.25)
    assert first.path.curvature[-1] == pytest.approx(0.25)
    assert second.path.curvature[0] == 0.0


def test_direction_other_than_plus_minus_one_is_rejected():
    p = paths.straight(5.0)
    bad = paths.Path(p.x, p.y, p.heading, p.curvature, p.s, np.zeros(len(p)))
    with pytest.raises(ValueError):
        split_at_cusps(bad)
