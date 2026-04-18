"""
Localization Engine: Triangulation + Trilateration + Overlap

Geometric indoor localization with weighted least squares,
RANSAC outlier rejection, and EKF position tracking.
"""

import logging
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)

# --------------------------------------------------------------------------- #
# Trilateration
# --------------------------------------------------------------------------- #


def trilaterate_lsq(
    distances: np.ndarray,
    anchors: np.ndarray,
    weights: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Weighted least-squares trilateration.

    Solves the linearised trilateration equations using the last anchor
    as a pivot point.  The linear system is
        A @ [x, y]^T = b
    where for each anchor i = 0 .. N-2 (relative to pivot N-1):
        A[i]  = 2 * [x_{N-1} - x_i,  y_{N-1} - y_i]
        b[i]  = d_i^2 - d_{N-1}^2 - x_i^2 + x_{N-1}^2 - y_i^2 + y_{N-1}^2

    Parameters
    ----------
    distances:
        1-D array of shape (N,) with measured distances to each anchor.
    anchors:
        2-D array of shape (N, 2) with anchor (x, y) coordinates.
    weights:
        Optional 1-D array of shape (N,).  When provided, WLS is used
        by scaling each row of *A* and element of *b* by ``sqrt(w_i)``.

    Returns
    -------
    np.ndarray
        Estimated position (x, y).

    Raises
    ------
    ValueError
        If fewer than two anchors are supplied, or if the linear system
        cannot be solved.
    """
    distances = np.asarray(distances, dtype=float).ravel()
    anchors = np.asarray(anchors, dtype=float)

    if anchors.ndim != 2 or anchors.shape[1] != 2:
        raise ValueError("anchors must have shape (N, 2)")
    if distances.shape[0] != anchors.shape[0]:
        raise ValueError("distances and anchors must have the same length")
    if distances.shape[0] < 2:
        raise ValueError("need at least 2 anchors for trilateration")

    n = distances.shape[0]
    pivot = anchors[-1]
    d_pivot = distances[-1]

    # Build linear system (N-1 equations)
    A = 2.0 * (pivot - anchors[:-1])          # shape (N-1, 2)
    b = (
        distances[:-1] ** 2
        - d_pivot ** 2
        - np.sum(anchors[:-1] ** 2, axis=1)
        + np.sum(pivot ** 2)
    )

    # Weighted least squares (weights for all N anchors; apply to rows)
    if weights is not None:
        weights = np.asarray(weights, dtype=float).ravel()
        if weights.shape[0] != n:
            raise ValueError("weights length must match number of anchors")
        row_w = np.sqrt(weights[:-1])
        A = A * row_w[:, np.newaxis]
        b = b * row_w

    pos, residuals, rank, _ = np.linalg.lstsq(A, b, rcond=None)
    if rank < 2:
        raise ValueError("trilateration linear system is rank-deficient")

    return pos


# --------------------------------------------------------------------------- #
# Triangulation
# --------------------------------------------------------------------------- #


def triangulate_angles(
    angles: np.ndarray,
    anchor_positions: np.ndarray,
    anchor_orientations: np.ndarray,
) -> np.ndarray:
    """Triangulate a position from AoA measurements at multiple anchors.

    Each anchor reports an AoA relative to its local orientation.  The
    measurement is converted to a ray in world coordinates, and the
    position is the least-squares intersection of all rays.

    The line from anchor *i* is  :math:`p_i + t_i * u_i`  where
    :math:`u_i = [\\cos(\\theta_i), \\sin(\\theta_i)]` and
    :math:`\\theta_i = \\text{orientation}_i + \\text{angle}_i`.

    The closest point to all lines is found by solving a linear system
    derived from the perpendicular constraints (see Bjorck, *Numerical
    Methods for Least Squares Problems*).

    Parameters
    ----------
    angles:
        1-D array of shape (N,) with AoA values in **radians** relative
        to each anchor's local orientation.
    anchor_positions:
        2-D array of shape (N, 2) with anchor (x, y) coordinates.
    anchor_orientations:
        1-D array of shape (N,) with absolute anchor orientations in
        radians.

    Returns
    -------
    np.ndarray
        Estimated position (x, y).

    Raises
    ------
    ValueError
        If fewer than two angle measurements are provided, or if the
        linear system is rank-deficient.
    """
    angles = np.asarray(angles, dtype=float).ravel()
    anchor_positions = np.asarray(anchor_positions, dtype=float)
    anchor_orientations = np.asarray(anchor_orientations, dtype=float).ravel()

    if angles.shape[0] < 2:
        raise ValueError("need at least 2 angles for triangulation")
    if anchor_positions.shape[0] != angles.shape[0]:
        raise ValueError("anchor_positions and angles must have the same length")
    if anchor_orientations.shape[0] != angles.shape[0]:
        raise ValueError("anchor_orientations and angles must have the same length")

    # Global ray directions
    global_angles = anchor_orientations + angles
    cos_t = np.cos(global_angles)
    sin_t = np.sin(global_angles)

    # Solve the linear system for the closest point to all lines.
    # For each line: (I - u u^T) p = (I - u u^T) a
    # Stack all equations and solve via lstsq.
    n = angles.shape[0]
    A = np.zeros((2 * n, 2), dtype=float)
    b = np.zeros(2 * n, dtype=float)

    for i in range(n):
        ux, uy = cos_t[i], sin_t[i]
        # Projection matrix P_perp = I - u u^T
        pxx = 1.0 - ux * ux
        pxy = -ux * uy
        pyy = 1.0 - uy * uy
        ax, ay = anchor_positions[i]

        A[2 * i] = [pxx, pxy]
        A[2 * i + 1] = [pxy, pyy]
        b[2 * i] = pxx * ax + pxy * ay
        b[2 * i + 1] = pxy * ax + pyy * ay

    pos, _, rank, _ = np.linalg.lstsq(A, b, rcond=None)
    if rank < 2:
        raise ValueError("triangulation linear system is rank-deficient")

    return pos


# --------------------------------------------------------------------------- #
# Overlap / Consistency Check
# --------------------------------------------------------------------------- #


def overlap_consistency_check(
    pos_tri: np.ndarray,
    pos_tri_lat: np.ndarray,
    cov_tri: np.ndarray,
    cov_tri_lat: np.ndarray,
    threshold: float = 3.841,  # chi2(2, 0.95)
) -> Tuple[np.ndarray, float]:
    """Fuse triangulation and trilateration estimates via Covariance Intersection.

    First computes the Mahalanobis distance between the two position
    estimates.  If the distance is below *threshold*, the estimates are
    deemed consistent and fused using Covariance Intersection (CI).
    The confidence score decays exponentially as the Mahalanobis distance
    increases.

    Parameters
    ----------
    pos_tri:
        Position from triangulation, shape (2,).
    pos_tri_lat:
        Position from trilateration, shape (2,).
    cov_tri:
        Covariance of triangulation estimate, shape (2, 2).
    cov_tri_lat:
        Covariance of trilateration estimate, shape (2, 2).
    threshold:
        Mahalanobis threshold for consistency (default corresponds to
        95% confidence on a 2-D chi-squared distribution).

    Returns
    -------
    Tuple[np.ndarray, float]
        ``(fused_position, confidence_score)``.  *confidence_score* is
        1.0 when fully consistent and decays toward 0 as the distance
        grows.
    """
    pos_tri = np.asarray(pos_tri, dtype=float)
    pos_tri_lat = np.asarray(pos_tri_lat, dtype=float)
    cov_tri = np.asarray(cov_tri, dtype=float)
    cov_tri_lat = np.asarray(cov_tri_lat, dtype=float)

    diff = pos_tri - pos_tri_lat

    # Mahalanobis distance using the combined covariance
    cov_sum = cov_tri + cov_tri_lat
    try:
        cov_sum_inv = np.linalg.inv(cov_sum)
        mahal = float(diff @ cov_sum_inv @ diff)
    except np.linalg.LinAlgError:
        mahal = float("inf")

    if mahal < threshold:
        # Consistent — fuse via Covariance Intersection
        omega = _ci_optimal_weight(cov_tri, cov_tri_lat)
        cov_fused_inv = omega * np.linalg.inv(cov_tri + 1e-9 * np.eye(2)) + (
            1.0 - omega
        ) * np.linalg.inv(cov_tri_lat + 1e-9 * np.eye(2))
        cov_fused = np.linalg.inv(cov_fused_inv)
        w1 = omega * cov_fused @ np.linalg.inv(cov_tri + 1e-9 * np.eye(2))
        w2 = (1.0 - omega) * cov_fused @ np.linalg.inv(
            cov_tri_lat + 1e-9 * np.eye(2)
        )
        pos_fused = w1 @ pos_tri + w2 @ pos_tri_lat
        confidence = 1.0
    else:
        # Inconsistent — select the estimate with smaller trace(cov)
        trace_tri = np.trace(cov_tri)
        trace_tri_lat = np.trace(cov_tri_lat)
        if trace_tri < trace_tri_lat:
            pos_fused = pos_tri
            cov_fused = cov_tri
        else:
            pos_fused = pos_tri_lat
            cov_fused = cov_tri_lat
        # Confidence decays exponentially with Mahalanobis distance
        confidence = float(np.exp(-(mahal - threshold) / threshold))

    return pos_fused, confidence


def _ci_optimal_weight(cov_a: np.ndarray, cov_b: np.ndarray) -> float:
    """Optimise the CI weight that minimises trace of the fused covariance.

    Uses a simple golden-section search on the interval [0, 1].
    """
    gr = (np.sqrt(5.0) + 1.0) / 2.0  # golden ratio
    a, b = 0.0, 1.0
    c = b - (b - a) / gr
    d = a + (b - a) / gr
    tol = 1e-4

    inv_a = np.linalg.inv(cov_a + 1e-9 * np.eye(2))
    inv_b = np.linalg.inv(cov_b + 1e-9 * np.eye(2))

    def _fused_cov(w: float) -> np.ndarray:
        s = w * inv_a + (1.0 - w) * inv_b
        return np.linalg.inv(s)

    while abs(c - d) > tol:
        if np.trace(_fused_cov(c)) < np.trace(_fused_cov(d)):
            b = d
        else:
            a = c
        c = b - (b - a) / gr
        d = a + (b - a) / gr

    omega = (b + a) / 2.0
    return float(np.clip(omega, 0.0, 1.0))


# --------------------------------------------------------------------------- #
# GDOP
# --------------------------------------------------------------------------- #


def compute_gdop(anchor_positions: np.ndarray, target: np.ndarray) -> float:
    """Geometric Dilution of Precision.

    Constructs the geometry matrix *G* whose rows are the unit vectors
    from the *target* to each anchor, and returns
    ``sqrt(trace(inv(G^T @ G)))``.

    Parameters
    ----------
    anchor_positions:
        Array of shape (N, 2).
    target:
        Target position (x, y), shape (2,).

    Returns
    -------
    float
        The GDOP value.  Larger values indicate poorer geometric
        conditioning.

    Raises
    ------
    ValueError
        If ``G^T @ G`` is singular (all anchors collinear with target).
    """
    anchor_positions = np.asarray(anchor_positions, dtype=float)
    target = np.asarray(target, dtype=float)

    diff = anchor_positions - target
    dist = np.linalg.norm(diff, axis=1, keepdims=True)
    dist = np.where(dist == 0, 1.0, dist)  # avoid div-by-zero
    G = diff / dist  # shape (N, 2)

    gtg = G.T @ G
    try:
        gdop = float(np.sqrt(np.trace(np.linalg.inv(gtg))))
    except np.linalg.LinAlgError as exc:
        raise ValueError("GDOP matrix is singular — anchors/target are degenerate") from exc

    return gdop


# --------------------------------------------------------------------------- #
# RANSAC
# --------------------------------------------------------------------------- #


def ransac_localization(
    measurements: List[Dict[str, Any]],
    threshold: float = 0.5,
    max_iterations: int = 100,
    min_sample_size: int = 3,
) -> np.ndarray:
    """Random Sample Consensus for robust trilateration.

    Repeatedly samples a minimal set of anchors, estimates a position
    via ``trilaterate_lsq``, and counts inliers within *threshold*.
    The position supported by the most inliers is returned.

    Parameters
    ----------
    measurements:
        List of dicts, each with keys ``'anchor'`` (x, y) and
        ``'distance'`` (float).
    threshold:
        Distance threshold (metres) for inlier classification.
    max_iterations:
        Maximum RANSAC iterations.
    min_sample_size:
        Minimum number of measurements required for a model hypothesis
        (default 3).

    Returns
    -------
    np.ndarray
        Estimated position (x, y).

    Raises
    ------
    ValueError
        If fewer than *min_sample_size* measurements are available.
    """
    if len(measurements) < min_sample_size:
        raise ValueError(
            f"need at least {min_sample_size} measurements, got {len(measurements)}"
        )

    anchors = np.array([m["anchor"] for m in measurements], dtype=float)
    distances = np.array([m["distance"] for m in measurements], dtype=float)
    n = len(measurements)

    best_pos = None
    best_inliers = -1
    best_inlier_indices = None

    rng = np.random.default_rng(seed=42)

    for _ in range(max_iterations):
        idx = rng.choice(n, size=min_sample_size, replace=False)
        sample_anchors = anchors[idx]
        sample_distances = distances[idx]

        try:
            pos = trilaterate_lsq(sample_distances, sample_anchors)
        except (ValueError, np.linalg.LinAlgError):
            continue

        # Count inliers
        residuals = np.linalg.norm(anchors - pos, axis=1) - distances
        inlier_mask = np.abs(residuals) < threshold
        n_inliers = int(np.sum(inlier_mask))

        if n_inliers > best_inliers:
            best_inliers = n_inliers
            best_pos = pos
            best_inlier_indices = inlier_mask

    if best_pos is None:
        raise ValueError("RANSAC failed to find a valid model")

    # Refine with all inliers
    if best_inlier_indices is not None and best_inliers >= min_sample_size:
        try:
            refined_pos = trilaterate_lsq(
                distances[best_inlier_indices],
                anchors[best_inlier_indices],
            )
            return refined_pos
        except (ValueError, np.linalg.LinAlgError):
            pass

    return best_pos


# --------------------------------------------------------------------------- #
# Localization Engine
# --------------------------------------------------------------------------- #


class LocalizationEngine:
    """Fuses trilateration and triangulation with overlap consistency checking.

    Maintains a set of *anchors* (ESP32s) with positions and
    orientations.  Distance and angle measurements are fed incrementally
    via :meth:`update_distances` and :meth:`update_angles`, and the
    current position estimate is obtained from
    :meth:`estimate_position`.

    Parameters
    ----------
    anchors : List[Dict]
        Each dict must contain keys ``'id'`` (str), ``'x'`` (float),
        ``'y'`` (float), and ``'orientation'`` (float, radians).
    """

    def __init__(self, anchors: List[Dict[str, Any]]):
        if not anchors:
            raise ValueError("at least one anchor is required")

        self.anchors: Dict[str, Dict[str, Any]] = {}
        for a in anchors:
            self.anchors[a["id"]] = {
                "x": float(a["x"]),
                "y": float(a["y"]),
                "orientation": float(a["orientation"]),
            }

        self._distances: Dict[str, Tuple[float, float]] = {}  # id -> (dist, var)
        self._angles: Dict[str, Tuple[float, float]] = {}     # id -> (angle, var)

        # EKF-style position tracking (constant-velocity)
        self._x = np.zeros(4)  # [px, py, vx, vy]
        self._P = np.eye(4) * 10.0
        self._Q = np.eye(4) * 0.1
        self._last_estimate_time: Optional[float] = None

    # ------------------------------------------------------------------ #

    def update_distances(self, esp_id: str, distance: float, variance: float) -> None:
        """Store a distance measurement from an anchor.

        Parameters
        ----------
        esp_id:
            Anchor identifier.
        distance:
            Measured distance (metres).
        variance:
            Measurement variance.
        """
        if esp_id not in self.anchors:
            logger.warning("distance update from unknown anchor '%s'", esp_id)
            return
        self._distances[esp_id] = (float(distance), float(variance))

    def update_angles(self, esp_id: str, angle: float, variance: float) -> None:
        """Store an angle measurement from an anchor.

        Parameters
        ----------
        esp_id:
            Anchor identifier.
        angle:
            Measured AoA in **radians**.
        variance:
            Measurement variance (rad^2).
        """
        if esp_id not in self.anchors:
            logger.warning("angle update from unknown anchor '%s'", esp_id)
            return
        self._angles[esp_id] = (float(angle), float(variance))

    # ------------------------------------------------------------------ #

    def estimate_position(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Estimate the current position and covariance.

        The estimator follows this priority:

        1. If both >=3 distances and >=2 angles are available, perform
           trilateration and triangulation independently, then fuse
           them via :func:`overlap_consistency_check`.
        2. If only >=3 distances, return the trilateration result.
        3. If only >=2 angles, return the triangulation result.
        4. Otherwise return ``None``.

        Returns
        -------
        Optional[Tuple[np.ndarray, np.ndarray]]
            ``(position, covariance)`` where each array has shape (2, 2)
            for *covariance*, or ``None``.
        """
        n_dist = len(self._distances)
        n_ang = len(self._angles)

        if n_dist >= 3 and n_ang >= 2:
            pos_lat, cov_lat = self._run_trilateration()
            pos_tri, cov_tri = self._run_triangulation()
            pos, conf = overlap_consistency_check(pos_tri, pos_lat, cov_tri, cov_lat)
            logger.debug("overlap fusion: conf=%.3f", conf)
            cov = cov_lat if conf < 0.5 else (cov_lat + cov_tri) * 0.5
            self._ekf_update(pos, cov)
            return pos, cov

        if n_dist >= 3:
            pos, cov = self._run_trilateration()
            self._ekf_update(pos, cov)
            return pos, cov

        if n_ang >= 2:
            pos, cov = self._run_triangulation()
            self._ekf_update(pos, cov)
            return pos, cov

        return None

    # ------------------------------------------------------------------ #
    # Internal helpers
    # ------------------------------------------------------------------ #

    def _run_trilateration(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return (position, covariance) from stored distances."""
        ids = list(self._distances.keys())
        anchors = np.array(
            [[self.anchors[i]["x"], self.anchors[i]["y"]] for i in ids],
            dtype=float,
        )
        distances = np.array([self._distances[i][0] for i in ids], dtype=float)
        variances = np.array([self._distances[i][1] for i in ids], dtype=float)

        weights = 1.0 / (variances + 1e-6)
        pos = trilaterate_lsq(distances, anchors, weights=weights)

        # Approximate covariance from residual Jacobian
        J = anchors - pos  # shape (N, 2)
        dists = np.linalg.norm(J, axis=1, keepdims=True)
        dists = np.where(dists == 0, 1.0, dists)
        J = J / dists  # unit vectors
        cov = np.linalg.inv(J.T @ np.diag(weights) @ J + 1e-6 * np.eye(2))

        return pos, cov

    def _run_triangulation(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return (position, covariance) from stored angles."""
        ids = list(self._angles.keys())
        positions = np.array(
            [[self.anchors[i]["x"], self.anchors[i]["y"]] for i in ids],
            dtype=float,
        )
        orientations = np.array(
            [self.anchors[i]["orientation"] for i in ids], dtype=float
        )
        angles = np.array([self._angles[i][0] for i in ids], dtype=float)
        variances = np.array([self._angles[i][1] for i in ids], dtype=float)

        pos = triangulate_angles(angles, positions, orientations)

        # Approximate covariance from ray directions
        global_angles = orientations + angles
        cos_t = np.cos(global_angles)
        sin_t = np.sin(global_angles)
        # Perpendicular vectors
        perp = np.column_stack((-sin_t, cos_t))
        weights = 1.0 / (variances + 1e-6)
        cov = np.linalg.inv(
            perp.T @ np.diag(weights) @ perp + 1e-6 * np.eye(2)
        )

        return pos, cov

    def _ekf_update(self, pos: np.ndarray, cov: np.ndarray) -> None:
        """Simple Kalman-style smoothing of the position estimate."""
        now = time.time()
        if self._last_estimate_time is not None:
            dt = now - self._last_estimate_time
            dt = np.clip(dt, 0.001, 1.0)
            F = np.array(
                [[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]]
            )
            self._x = F @ self._x
            self._P = F @ self._P @ F.T + self._Q

        # Update with position measurement
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
        R = np.zeros((2, 2), dtype=float)
        R[0, 0] = cov[0, 0] if cov.shape[0] > 0 else 1.0
        R[1, 1] = cov[1, 1] if cov.shape[0] > 1 else 1.0

        y = pos - H @ self._x
        S = H @ self._P @ H.T + R + 1e-6 * np.eye(2)
        try:
            K = self._P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return
        self._x = self._x + K @ y
        self._P = (np.eye(4) - K @ H) @ self._P
        self._last_estimate_time = now

    def reset_measurements(self) -> None:
        """Clear all stored distance and angle measurements."""
        self._distances.clear()
        self._angles.clear()


# --------------------------------------------------------------------------- #
# Self-test
# --------------------------------------------------------------------------- #


def _test() -> None:
    """Run basic correctness checks on all public functions."""
    logging.basicConfig(level=logging.DEBUG)
    print("=" * 60)
    print("LOCALIZATION MODULE SELF-TEST")
    print("=" * 60)

    # --- trilaterate_lsq ---
    print("\n--- trilaterate_lsq ---")
    anchors = np.array([[0, 0], [10, 0], [5, 8]], dtype=float)
    true_pos = np.array([4.0, 3.0], dtype=float)
    distances = np.linalg.norm(anchors - true_pos, axis=1)
    est = trilaterate_lsq(distances, anchors)
    err = np.linalg.norm(est - true_pos)
    print(f"  true_pos = {true_pos},  est = {est},  error = {err:.4f}")
    assert err < 1e-6, "trilateration error too large"

    # Weighted
    weights = np.array([1.0, 1.0, 0.1])
    est_w = trilaterate_lsq(distances, anchors, weights=weights)
    print(f"  weighted est = {est_w}")

    # --- triangulate_angles ---
    print("\n--- triangulate_angles ---")
    anchor_positions = np.array([[0, 0], [10, 0]], dtype=float)
    anchor_orientations = np.array([0.0, np.pi], dtype=float)
    # Target at (4, 3)
    true_pos = np.array([4.0, 3.0])
    angles = np.array(
        [np.arctan2(3, 4), np.arctan2(3, 4 - 10) - np.pi], dtype=float
    )
    est_ang = triangulate_angles(angles, anchor_positions, anchor_orientations)
    err_ang = np.linalg.norm(est_ang - true_pos)
    print(f"  true_pos = {true_pos},  est = {est_ang},  error = {err_ang:.4f}")
    assert err_ang < 1e-6, "triangulation error too large"

    # --- overlap_consistency_check ---
    print("\n--- overlap_consistency_check ---")
    pos_tri = np.array([4.1, 3.05])
    pos_lat = np.array([3.95, 2.98])
    cov_tri = np.eye(2) * 0.5
    cov_lat = np.eye(2) * 0.3
    fused, conf = overlap_consistency_check(pos_tri, pos_lat, cov_tri, cov_lat)
    print(f"  fused = {fused},  confidence = {conf:.4f}")
    assert conf == 1.0, "should be consistent"

    # Inconsistent case
    pos_far = np.array([20.0, 20.0])
    _, conf_bad = overlap_consistency_check(
        pos_tri, pos_far, cov_tri, cov_lat
    )
    print(f"  inconsistent confidence = {conf_bad:.4f}")
    assert conf_bad < 1.0, "should be inconsistent"

    # --- compute_gdop ---
    print("\n--- compute_gdop ---")
    gdop = compute_gdop(anchors, true_pos)
    print(f"  GDOP = {gdop:.4f}")
    assert gdop > 0, "GDOP must be positive"

    # --- ransac_localization ---
    print("\n--- ransac_localization ---")
    measurements = [
        {"anchor": np.array([0.0, 0.0]), "distance": 5.0},
        {"anchor": np.array([10.0, 0.0]), "distance": 5.0},
        {"anchor": np.array([5.0, 8.0]), "distance": 5.0},
        {"anchor": np.array([5.0, 0.0]), "distance": 100.0},  # outlier
    ]
    est_ransac = ransac_localization(measurements, threshold=2.0)
    print(f"  RANSAC est = {est_ransac}")

    # --- LocalizationEngine ---
    print("\n--- LocalizationEngine ---")
    engine = LocalizationEngine(
        [
            {"id": "esp0", "x": 0, "y": 0, "orientation": 0},
            {"id": "esp1", "x": 10, "y": 0, "orientation": np.pi / 2},
            {"id": "esp2", "x": 5, "y": 8, "orientation": np.pi},
            {"id": "esp3", "x": 2, "y": 5, "orientation": -np.pi / 2},
        ]
    )
    engine.update_distances("esp0", 5.0, 0.5)
    engine.update_distances("esp1", 5.0, 0.5)
    result = engine.estimate_position()
    assert result is None, "need >= 3 distances"
    engine.update_distances("esp2", 5.0, 0.5)
    result = engine.estimate_position()
    assert result is not None
    pos, cov = result
    print(f"  engine pos = {pos}, cov diag = {np.diag(cov)}")

    # With angles too
    engine.update_angles("esp0", np.arctan2(3, 4), 0.1)
    engine.update_angles("esp1", np.arctan2(3, 4 - 10) - np.pi / 2, 0.1)
    result2 = engine.estimate_position()
    assert result2 is not None
    pos2, cov2 = result2
    print(f"  fusion pos = {pos2}, cov diag = {np.diag(cov2)}")

    print("\n" + "=" * 60)
    print("ALL LOCALIZATION TESTS PASSED")
    print("=" * 60)


if __name__ == "__main__":
    _test()
