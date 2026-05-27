import math

import pytest

from simbiosys_perception.bin_wall_fit import (
    ScanPoint,
    fit_bin_wall,
    scan_to_ordered_points,
)


def _wall_points(x_m=0.35, y_min=-0.35, y_max=0.55, count=60):
    step = (y_max - y_min) / float(count - 1)
    return [
        ScanPoint(x_m, y_min + index * step, x_m)
        for index in range(count)
    ]


def _fit(points, direction="left"):
    return fit_bin_wall(
        points,
        desired_surface_angle_rad=math.pi / 2.0,
        strafe_direction=direction,
        max_fit_error_m=0.025,
        min_inliers=10,
        min_wall_length_m=0.30,
        cluster_jump_m=0.08,
        corner_endpoint_threshold_m=0.10,
        max_yaw_error_rad=math.radians(20.0),
    )


def test_fits_straight_bin_wall():
    result = _fit(_wall_points())

    assert result.valid
    assert not result.corner_detected
    assert result.distance_m == pytest.approx(0.35, abs=0.001)
    assert abs(result.yaw_error_rad) < math.radians(1.0)
    assert result.wall_length_m > 0.85


def test_ignores_small_leg_outliers():
    points = _wall_points()
    points.extend(
        [
            ScanPoint(0.33, -0.10, 0.33),
            ScanPoint(0.33, -0.09, 0.33),
            ScanPoint(0.33, 0.22, 0.33),
            ScanPoint(0.33, 0.23, 0.33),
        ]
    )

    result = _fit(points)

    assert result.valid
    assert result.distance_m == pytest.approx(0.35, abs=0.01)


def test_detects_corner_in_left_strafe_direction():
    result = _fit(_wall_points(y_min=-0.50, y_max=0.08), direction="left")

    assert result.valid
    assert result.corner_detected
    assert result.endpoint_in_direction_m <= 0.10


def test_scan_angle_offset_maps_raw_right_to_robot_front():
    points = scan_to_ordered_points(
        ranges=[1.0],
        angle_min=-math.pi / 2.0,
        angle_increment=0.0,
        min_range_m=0.1,
        max_range_m=2.0,
        roi_min_angle_rad=-0.1,
        roi_max_angle_rad=0.1,
        scan_angle_offset_rad=-math.pi / 2.0,
    )

    assert len(points) == 1
    assert points[0].x == pytest.approx(1.0)
    assert points[0].y == pytest.approx(0.0, abs=1e-6)
