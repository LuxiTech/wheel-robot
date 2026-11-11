"""Motor reference/sign handling for the official v1 approximation."""

import pytest
from wheel_motion_core.gripper import finger_displacement


def test_official_endpoints_and_mirror():
    for sign in [-1, 1]:
        r = finger_displacement(sign * 1.0472, offset_rad=0, opening_sign=sign, bounds=(0, 0.044))
        assert r["estimated_finger_displacement_m"] == pytest.approx(0.044)
        assert r["within_urdf_limits"]


def test_software_offset_is_restored_once():
    r = finger_displacement(
        0, offset_rad=-0.3732738231479363, opening_sign=-1, bounds=(0.01286, 0.044)
    )
    assert r["estimated_finger_displacement_m"] == pytest.approx(0.015683774081846066)
    assert r["within_urdf_limits"]


def test_incompatible_zero_is_reported_not_clipped():
    r = finger_displacement(
        0, offset_rad=-0.36717021438925773, opening_sign=1, bounds=(0.01286, 0.044)
    )
    assert r["estimated_finger_displacement_m"] < 0
    assert not r["within_urdf_limits"]


def test_confirmed_sdk_closed_zero_is_independent_of_raw_motor_offset():
    for sign, offset in [(1, -0.36717), (-1, -0.37327)]:
        r = finger_displacement(
            0,
            offset_rad=offset,
            opening_sign=sign,
            bounds=(0.01286, 0.044),
            closed_feedback_rad=0,
            closed_displacement_m=0.01286,
        )
        assert r["estimated_finger_displacement_m"] == pytest.approx(0.01286)
        assert r["within_urdf_limits"]
