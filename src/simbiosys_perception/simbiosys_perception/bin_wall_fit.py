import math
from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class ScanPoint:
    x: float
    y: float
    range_m: float


@dataclass(frozen=True)
class BinWallResult:
    valid: bool
    corner_detected: bool = False
    distance_m: float = math.nan
    yaw_error_rad: float = math.nan
    confidence: float = 0.0
    wall_length_m: float = 0.0
    wall_start_m: float = math.nan
    wall_end_m: float = math.nan
    endpoint_in_direction_m: float = math.nan
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


def scan_to_ordered_points(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    min_range_m: float,
    max_range_m: float,
    roi_min_angle_rad: float,
    roi_max_angle_rad: float,
    scan_angle_offset_rad: float = 0.0,
) -> list[ScanPoint]:
    points: list[ScanPoint] = []
    for index, range_m in enumerate(ranges):
        if (
            not math.isfinite(range_m)
            or range_m < min_range_m
            or range_m > max_range_m
        ):
            continue
        angle = angle_min + index * angle_increment - scan_angle_offset_rad
        if angle < roi_min_angle_rad or angle > roi_max_angle_rad:
            continue
        points.append(
            ScanPoint(
                range_m * math.cos(angle),
                range_m * math.sin(angle),
                range_m,
            )
        )
    return points


def fit_bin_wall(
    points: list[ScanPoint],
    desired_surface_angle_rad: float,
    strafe_direction: str,
    max_fit_error_m: float,
    min_inliers: int,
    min_wall_length_m: float,
    cluster_jump_m: float,
    corner_endpoint_threshold_m: float,
    max_yaw_error_rad: float,
) -> BinWallResult:
    clusters = _split_clusters(points, cluster_jump_m)
    best: BinWallResult | None = None
    best_score = -math.inf

    for cluster in clusters:
        if len(cluster) < min_inliers:
            continue
        result = _fit_cluster(
            cluster,
            desired_surface_angle_rad,
            strafe_direction,
            max_fit_error_m,
            min_inliers,
            min_wall_length_m,
            corner_endpoint_threshold_m,
            max_yaw_error_rad,
        )
        if not result.valid:
            continue
        score = (
            result.confidence
            + min(1.0, result.wall_length_m)
            + 0.02 * result.inlier_count
        )
        if score > best_score:
            best = result
            best_score = score

    if best is None:
        return BinWallResult(
            False,
            message=f"no usable wall cluster from {len(points)} points",
        )
    return best


def _split_clusters(
    points: list[ScanPoint],
    cluster_jump_m: float,
) -> list[list[ScanPoint]]:
    if not points:
        return []
    clusters: list[list[ScanPoint]] = [[points[0]]]
    for point in points[1:]:
        previous = clusters[-1][-1]
        cartesian_jump = math.hypot(
            point.x - previous.x,
            point.y - previous.y,
        )
        range_jump = abs(point.range_m - previous.range_m)
        if (
            cartesian_jump > cluster_jump_m
            and range_jump > 0.5 * cluster_jump_m
        ):
            clusters.append([point])
        else:
            clusters[-1].append(point)
    return clusters


def _fit_cluster(
    points: list[ScanPoint],
    desired_surface_angle_rad: float,
    strafe_direction: str,
    max_fit_error_m: float,
    min_inliers: int,
    min_wall_length_m: float,
    corner_endpoint_threshold_m: float,
    max_yaw_error_rad: float,
) -> BinWallResult:
    initial = _fit_line(points)
    if initial is None:
        return BinWallResult(False, message="line fit failed")

    inliers = [
        point
        for point in points
        if _point_line_distance(
            point,
            initial[0],
            initial[1],
            initial[2],
        ) <= max_fit_error_m
    ]
    if len(inliers) < min_inliers:
        return BinWallResult(
            False,
            message=f"not enough inliers: {len(inliers)}",
        )

    refined = _fit_line(inliers)
    if refined is None:
        return BinWallResult(False, message="refined line fit failed")

    normal_x, normal_y, offset = refined
    line_angle = math.atan2(normal_x, -normal_y)
    yaw_error = normalize_line_angle(line_angle - desired_surface_angle_rad)
    if abs(yaw_error) > max_yaw_error_rad:
        return BinWallResult(
            False,
            message=f"wall yaw error too high: {yaw_error:.3f}rad",
        )

    residuals = [
        _point_line_distance(point, normal_x, normal_y, offset)
        for point in inliers
    ]
    rms_error = math.sqrt(
        sum(error * error for error in residuals) / len(residuals)
    )
    if rms_error > max_fit_error_m:
        return BinWallResult(
            False,
            message=f"fit error too high: {rms_error:.3f}m",
        )

    axis_x = math.cos(desired_surface_angle_rad)
    axis_y = math.sin(desired_surface_angle_rad)
    projections = [
        point.x * axis_x + point.y * axis_y
        for point in inliers
    ]
    wall_start = min(projections)
    wall_end = max(projections)
    wall_length = wall_end - wall_start
    if wall_length < min_wall_length_m:
        return BinWallResult(
            False,
            message=f"wall segment too short: {wall_length:.3f}m",
        )

    direction_sign = (
        -1.0
        if strafe_direction.strip().lower() in {"right", "-1", "negative"}
        else 1.0
    )
    endpoint = wall_end if direction_sign > 0.0 else -wall_start
    corner_detected = endpoint <= corner_endpoint_threshold_m
    confidence = _confidence(
        len(inliers),
        len(points),
        rms_error,
        max_fit_error_m,
        wall_length,
    )

    return BinWallResult(
        valid=True,
        corner_detected=corner_detected,
        distance_m=abs(offset),
        yaw_error_rad=yaw_error,
        confidence=confidence,
        wall_length_m=wall_length,
        wall_start_m=wall_start,
        wall_end_m=wall_end,
        endpoint_in_direction_m=endpoint,
        inlier_count=len(inliers),
        rms_error_m=rms_error,
        message=(
            f"fit {len(inliers)}/{len(points)} points, "
            f"length={wall_length:.3f}m, "
            f"endpoint={endpoint:.3f}m, rms={rms_error:.3f}m"
        ),
    )


def _fit_line(points: list[ScanPoint]) -> tuple[float, float, float] | None:
    if len(points) < 2:
        return None

    mean_x = sum(point.x for point in points) / len(points)
    mean_y = sum(point.y for point in points) / len(points)
    cov_xx = sum((point.x - mean_x) ** 2 for point in points) / len(points)
    cov_xy = (
        sum((point.x - mean_x) * (point.y - mean_y) for point in points)
        / len(points)
    )
    cov_yy = sum((point.y - mean_y) ** 2 for point in points) / len(points)

    trace = cov_xx + cov_yy
    determinant_term = math.sqrt(
        max(0.0, (cov_xx - cov_yy) ** 2 + 4.0 * cov_xy * cov_xy)
    )
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


def _point_line_distance(
    point: ScanPoint,
    normal_x: float,
    normal_y: float,
    offset: float,
) -> float:
    return abs(normal_x * point.x + normal_y * point.y + offset)


def _confidence(
    inlier_count: int,
    point_count: int,
    rms_error_m: float,
    max_fit_error_m: float,
    wall_length_m: float,
) -> float:
    if point_count <= 0 or max_fit_error_m <= 0.0:
        return 0.0
    inlier_score = min(1.0, inlier_count / max(1.0, float(point_count)))
    fit_score = max(0.0, 1.0 - rms_error_m / max_fit_error_m)
    length_score = min(1.0, wall_length_m / 0.5)
    return max(
        0.0,
        min(
            1.0,
            0.45 * inlier_score + 0.35 * fit_score + 0.20 * length_score,
        ),
    )
