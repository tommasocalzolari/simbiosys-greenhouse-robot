import math

from simbiosys_perception.surface_fit import Point2D, fit_surface_alignment, scan_ranges_to_points


def test_fit_front_surface_parallel_to_y_axis() -> None:
    points = [Point2D(0.5, y / 100.0) for y in range(-30, 31, 5)]

    result = fit_surface_alignment(
        points,
        desired_surface_angle_rad=math.pi / 2.0,
        max_fit_error_m=0.02,
        min_inliers=5,
    )

    assert result.valid
    assert result.distance_m == pytest_approx(0.5)
    assert result.yaw_error_rad == pytest_approx(0.0)
    assert result.confidence > 0.9


def test_fit_rejects_scattered_points() -> None:
    points = [
        Point2D(0.2, -0.4),
        Point2D(0.8, -0.1),
        Point2D(0.4, 0.3),
        Point2D(1.0, 0.4),
    ]

    result = fit_surface_alignment(
        points,
        desired_surface_angle_rad=math.pi / 2.0,
        max_fit_error_m=0.01,
        min_inliers=4,
    )

    assert not result.valid


def test_scan_ranges_to_points_filters_roi_and_range() -> None:
    ranges = [0.4, 0.5, math.inf, 2.0, 0.6]
    points = scan_ranges_to_points(
        ranges,
        angle_min=-0.2,
        angle_increment=0.1,
        min_range_m=0.2,
        max_range_m=1.0,
        roi_min_angle_rad=-0.15,
        roi_max_angle_rad=0.15,
    )

    assert len(points) == 1
    assert points[0].x == pytest_approx(0.5 * math.cos(-0.1))
    assert points[0].y == pytest_approx(0.5 * math.sin(-0.1))


def pytest_approx(value: float):
    import pytest

    return pytest.approx(value, abs=1e-6)
