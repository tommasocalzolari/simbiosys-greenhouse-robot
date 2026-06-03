import math
from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float


@dataclass(frozen=True)
class SurfaceFitResult:
    valid: bool
    distance_m: float = math.nan
    yaw_error_rad: float = math.nan
    confidence: float = 0.0
    inlier_count: int = 0
    rms_error_m: float = math.inf
    message: str = ""


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def normalize_line_angle(angle: float) -> float:
    angle = normalize_angle(angle)
    if angle > math.pi / 2.0:
        angle -= math.pi
    elif angle < -math.pi / 2.0:
        angle += math.pi
    return angle


def scan_ranges_to_points(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    min_range_m: float,
    max_range_m: float,
    roi_min_angle_rad: float,
    roi_max_angle_rad: float,
    angle_offset_rad: float = 0.0,
) -> list[Point2D]:
    points: list[Point2D] = []
    for index, range_m in enumerate(ranges):
        if not math.isfinite(range_m) or range_m < min_range_m or range_m > max_range_m:
            continue
        angle = angle_min + index * angle_increment + angle_offset_rad
        if angle < roi_min_angle_rad or angle > roi_max_angle_rad:
            continue
        points.append(Point2D(range_m * math.cos(angle), range_m * math.sin(angle)))
    return points


def fit_surface_alignment(
    points: list[Point2D],
    desired_surface_angle_rad: float,
    max_fit_error_m: float,
    min_inliers: int,
) -> SurfaceFitResult:
    if len(points) < min_inliers:
        return SurfaceFitResult(False, message=f"not enough points: {len(points)}")

    dominant = _fit_dominant_line(
        points,
        desired_surface_angle_rad,
        max_fit_error_m,
        min_inliers,
    )
    if dominant is None:
        return SurfaceFitResult(False, message="dominant line fit failed")

    normal_x, normal_y, offset, inliers = dominant
    residuals = [_point_line_distance(point, normal_x, normal_y, offset) for point in inliers]
    rms_error = math.sqrt(sum(error * error for error in residuals) / len(residuals))
    if rms_error > max_fit_error_m:
        return SurfaceFitResult(
            False,
            inlier_count=len(inliers),
            rms_error_m=rms_error,
            message=f"fit error too high: {rms_error:.3f}m",
        )

    line_angle = math.atan2(normal_x, -normal_y)
    yaw_error = normalize_line_angle(line_angle - desired_surface_angle_rad)
    confidence = _confidence(len(inliers), len(points), rms_error, max_fit_error_m)
    return SurfaceFitResult(
        valid=True,
        distance_m=abs(offset),
        yaw_error_rad=yaw_error,
        confidence=confidence,
        inlier_count=len(inliers),
        rms_error_m=rms_error,
        message=(
            f"dominant line fit {len(inliers)}/{len(points)} points, "
            f"rms={rms_error:.3f}m"
        ),
    )


def _fit_dominant_line(
    points: list[Point2D],
    desired_surface_angle_rad: float,
    max_fit_error_m: float,
    min_inliers: int,
) -> tuple[float, float, float, list[Point2D]] | None:
    best: tuple[
        tuple[float, float, float, float],
        float,
        float,
        float,
        list[Point2D],
    ] | None = None

    for candidate in _candidate_lines(points):
        inliers = _line_inliers(points, candidate, max_fit_error_m)
        if len(inliers) < min_inliers:
            continue

        refined = _fit_line(inliers)
        if refined is None:
            continue

        inliers = _line_inliers(points, refined, max_fit_error_m)
        if len(inliers) < min_inliers:
            continue

        refined = _fit_line(inliers)
        if refined is None:
            continue

        normal_x, normal_y, offset = refined
        residuals = [_point_line_distance(point, normal_x, normal_y, offset) for point in inliers]
        rms_error = math.sqrt(sum(error * error for error in residuals) / len(residuals))
        if rms_error > max_fit_error_m:
            continue

        line_angle = math.atan2(normal_x, -normal_y)
        angle_error = abs(normalize_line_angle(line_angle - desired_surface_angle_rad))
        span = _projected_span(inliers, line_angle)
        score = (float(len(inliers)), span, -angle_error, -rms_error)
        if best is None or score > best[0]:
            best = (score, normal_x, normal_y, offset, inliers)

    if best is None:
        return None
    _, normal_x, normal_y, offset, inliers = best
    return normal_x, normal_y, offset, inliers


def _candidate_lines(points: list[Point2D]) -> Iterable[tuple[float, float, float]]:
    sample_count = min(80, len(points))
    if sample_count <= 1:
        return

    if len(points) <= sample_count:
        sample_indices = list(range(len(points)))
    else:
        step = (len(points) - 1) / float(sample_count - 1)
        sample_indices = sorted({round(index * step) for index in range(sample_count)})

    for outer_index, point_index in enumerate(sample_indices):
        for other_index in sample_indices[outer_index + 1 :]:
            line = _line_from_points(points[point_index], points[other_index])
            if line is not None:
                yield line


def _line_from_points(first: Point2D, second: Point2D) -> tuple[float, float, float] | None:
    dx = second.x - first.x
    dy = second.y - first.y
    norm = math.hypot(dx, dy)
    if norm < 1e-6:
        return None

    normal_x = dy / norm
    normal_y = -dx / norm
    offset = -(normal_x * first.x + normal_y * first.y)
    return normal_x, normal_y, offset


def _line_inliers(
    points: list[Point2D],
    line: tuple[float, float, float],
    max_fit_error_m: float,
) -> list[Point2D]:
    normal_x, normal_y, offset = line
    return [
        point
        for point in points
        if _point_line_distance(point, normal_x, normal_y, offset) <= max_fit_error_m
    ]


def _fit_line(points: list[Point2D]) -> tuple[float, float, float] | None:
    if len(points) < 2:
        return None

    mean_x = sum(point.x for point in points) / len(points)
    mean_y = sum(point.y for point in points) / len(points)
    cov_xx = sum((point.x - mean_x) ** 2 for point in points) / len(points)
    cov_xy = sum((point.x - mean_x) * (point.y - mean_y) for point in points) / len(points)
    cov_yy = sum((point.y - mean_y) ** 2 for point in points) / len(points)

    trace = cov_xx + cov_yy
    determinant_term = math.sqrt(max(0.0, (cov_xx - cov_yy) ** 2 + 4.0 * cov_xy * cov_xy))
    smallest_eigenvalue = 0.5 * (trace - determinant_term)

    if abs(cov_xy) > 1e-9:
        normal_x = cov_xy
        normal_y = smallest_eigenvalue - cov_xx
    elif cov_xx < cov_yy:
        normal_x = 1.0
        normal_y = 0.0
    else:
        normal_x = 0.0
        normal_y = 1.0

    norm = math.hypot(normal_x, normal_y)
    if norm < 1e-9:
        return None

    normal_x /= norm
    normal_y /= norm
    offset = -(normal_x * mean_x + normal_y * mean_y)
    return normal_x, normal_y, offset


def _point_line_distance(point: Point2D, normal_x: float, normal_y: float, offset: float) -> float:
    return abs(normal_x * point.x + normal_y * point.y + offset)


def _projected_span(points: list[Point2D], line_angle: float) -> float:
    direction_x = math.cos(line_angle)
    direction_y = math.sin(line_angle)
    projections = [point.x * direction_x + point.y * direction_y for point in points]
    return max(projections) - min(projections)


def _confidence(
    inlier_count: int,
    point_count: int,
    rms_error_m: float,
    max_fit_error_m: float,
) -> float:
    if point_count <= 0 or max_fit_error_m <= 0.0:
        return 0.0
    inlier_score = min(1.0, inlier_count / max(1.0, float(point_count)))
    fit_score = max(0.0, 1.0 - rms_error_m / max_fit_error_m)
    return max(0.0, min(1.0, 0.6 * inlier_score + 0.4 * fit_score))
