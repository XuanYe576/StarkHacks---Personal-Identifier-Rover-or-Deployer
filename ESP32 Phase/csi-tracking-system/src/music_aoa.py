"""
MUSIC Algorithm for WiFi AoA Estimation

Implements subspace-based direction finding with spatial smoothing.
Supports virtual arrays from ESP32 subcarriers and physical antenna arrays.

References:
    - Schmidt, R.O. "Multiple Emitter Location and Signal Parameter Estimation."
      IEEE Trans. Antennas Propag., 1986.
    - SpotFi: Decimeter Level Localization Using WiFi (ACM SIGCOMM 2015)
"""

import logging
from typing import List, Optional

import numpy as np
from scipy.signal import find_peaks

logger = logging.getLogger(__name__)


def estimate_covariance(X: np.ndarray) -> np.ndarray:
    """Estimate the sample covariance matrix from spatial snapshots.

    Computes the maximum-likelihood sample covariance:
        R = (1/N) * X @ X^H

    Args:
        X: Array of shape (M, N) where M is the number of sensors
           (or virtual sensors) and N is the number of snapshots.

    Returns:
        R: Covariance matrix of shape (M, M).

    Raises:
        ValueError: If X is not a 2-D array.
    """
    if X.ndim != 2:
        raise ValueError(f"Expected 2-D array, got {X.ndim}D")
    M, N = X.shape
    if N == 0:
        logger.warning("Empty snapshot matrix; returning zeros covariance")
        return np.zeros((M, M), dtype=X.dtype)
    R = (1.0 / N) * (X @ X.conj().T)
    return R


def music_spectrum(
    R: np.ndarray,
    steering_vectors: np.ndarray,
    n_signals: int,
) -> np.ndarray:
    """Compute the MUSIC pseudospectrum from a covariance matrix.

    Performs eigendecomposition on R, separates signal and noise
    subspaces, and evaluates the inverse of the projection onto the
    noise subspace across all steering vectors.

    Args:
        R: Covariance matrix of shape (M, M).
        steering_vectors: Steering vector matrix of shape (M, P)
            where P is the number of angle grid points.
        n_signals: Assumed number of incident signals (must be < M).

    Returns:
        pseudospectrum: 1-D array of shape (P,) containing MUSIC
        spectrum values (larger = more likely angle).

    Raises:
        ValueError: If n_signals >= M or if dimensions are incompatible.
    """
    if R.ndim != 2 or R.shape[0] != R.shape[1]:
        raise ValueError("Covariance matrix R must be square")
    M = R.shape[0]
    if n_signals >= M:
        raise ValueError(
            f"n_signals ({n_signals}) must be less than number of sensors ({M})"
        )
    if steering_vectors.shape[0] != M:
        raise ValueError(
            f"Steering vectors first dim ({steering_vectors.shape[0]}) must match M ({M})"
        )

    # Eigendecomposition of Hermitian covariance matrix
    eigenvalues, eigenvectors = np.linalg.eigh(R)

    # Sort in descending order of eigenvalue magnitude
    idx = np.argsort(eigenvalues)[::-1]
    eigenvectors = eigenvectors[:, idx]

    # Noise subspace: eigenvectors corresponding to smallest eigenvalues
    En = eigenvectors[:, n_signals:]  # shape (M, M - n_signals)

    # Compute pseudospectrum: P(theta) = 1 / (a^H @ En @ En^H @ a)
    # Vectorized over all steering vectors
    # For each steering vector a (column of steering_vectors):
    #   denom = a^H @ En @ En^H @ a = ||En^H @ a||^2
    EnH_a = En.conj().T @ steering_vectors  # shape (M - n_signals, P)
    denom = np.sum(np.abs(EnH_a) ** 2, axis=0)  # shape (P,)

    # Avoid division by zero
    eps = np.finfo(float).eps * np.max(np.abs(eigenvalues))
    denom = np.maximum(denom, eps)

    pseudospectrum = 1.0 / denom
    return pseudospectrum


def spatial_smoothing(X: np.ndarray, subarray_size: int) -> np.ndarray:
    """Forward-backward spatial smoothing for coherent signal handling.

    Divides the sensor array into overlapping subarrays, computes the
    covariance matrix for each subarray, and averages them together with
    their conjugate-reversed (backward) counterparts. This decorrelates
    coherent signals that would otherwise cause rank deficiency.

    Args:
        X: Array of shape (M, N) where M is the number of sensors
           and N is the number of snapshots.
        subarray_size: Number of sensors per subarray (L). Must be <= M.

    Returns:
        R_ss: Spatially smoothed covariance matrix of shape (L, L).

    Raises:
        ValueError: If subarray_size > M or subarray_size < 1.
    """
    if X.ndim != 2:
        raise ValueError(f"Expected 2-D array, got {X.ndim}D")
    M, N = X.shape
    if subarray_size < 1:
        raise ValueError("subarray_size must be at least 1")
    if subarray_size > M:
        raise ValueError(
            f"subarray_size ({subarray_size}) cannot exceed number of sensors ({M})"
        )

    n_subarrays = M - subarray_size + 1
    R_fwd = np.zeros((subarray_size, subarray_size), dtype=np.complex128)

    # Forward smoothing: average covariance from all subarrays
    for i in range(n_subarrays):
        X_sub = X[i : i + subarray_size, :]  # shape (L, N)
        R_sub = estimate_covariance(X_sub)
        R_fwd += R_sub
    R_fwd /= n_subarrays

    # Backward smoothing: conjugate-reversed subarrays
    J = np.fliplr(np.eye(subarray_size))
    R_bwd = J @ R_fwd.conj() @ J

    # Forward-backward averaged covariance
    R_ss = 0.5 * (R_fwd + R_bwd)
    return R_ss


def build_steering_ula(
    angles: np.ndarray,
    d: float,
    wavelength: float,
    M: int,
) -> np.ndarray:
    """Build Uniform Linear Array (ULA) steering vectors for an angle grid.

    For a ULA with element spacing d, the steering vector for angle theta is:
        a(theta) = [1, e^(-j*2*pi*d*sin(theta)/lambda), ...,
                    e^(-j*2*pi*(M-1)*d*sin(theta)/lambda)]^T

    Args:
        angles: 1-D array of angles in RADIANS.
        d: Inter-element spacing (meters).
        wavelength: Signal wavelength (meters).
        M: Number of array elements (sensors).

    Returns:
        steering_vectors: Matrix of shape (M, len(angles)) where each
            column is the steering vector for the corresponding angle.

    Raises:
        ValueError: If d or wavelength is not positive, or M < 1.
    """
    if d <= 0:
        raise ValueError(f"Element spacing d must be positive, got {d}")
    if wavelength <= 0:
        raise ValueError(f"Wavelength must be positive, got {wavelength}")
    if M < 1:
        raise ValueError(f"Number of elements M must be >= 1, got {M}")

    angles = np.asarray(angles).flatten()
    k = 2.0 * np.pi * d / wavelength  # wave number scaled by spacing

    # Element indices: 0, 1, ..., M-1
    m_idx = np.arange(M)[:, np.newaxis]  # shape (M, 1)

    # Steering vector matrix: each column is a(theta_i)
    # shape (M, len(angles))
    steering_vectors = np.exp(-1j * k * m_idx * np.sin(angles))
    return steering_vectors


def estimate_aoa(
    csi_data: np.ndarray,
    n_signals: int = 3,
    antenna_positions: Optional[np.ndarray] = None,
) -> List[float]:
    """Estimate Angle of Arrival using the MUSIC algorithm.

    Main entry point for AoA estimation. If physical antenna positions
    are provided, uses them to build the steering vectors. Otherwise,
    uses a virtual array constructed from subcarrier grouping (SpotFi-style).

    Args:
        csi_data: Complex CSI data. If antenna_positions is None, expected
            shape is (n_subcarriers, n_snapshots). If antenna_positions is
            provided, expected shape is (n_antennas, n_snapshots).
        n_signals: Assumed number of incident signals. Default is 3.
        antenna_positions: Optional 1-D or 2-D array of physical antenna
            positions. If 1-D, treated as positions along a ULA (meters).
            If None, uses virtual subcarrier array.

    Returns:
        List of estimated angles in DEGREES, sorted by peak prominence.

    Raises:
        ValueError: If input dimensions are invalid.
    """
    csi_data = np.asarray(csi_data)
    if csi_data.ndim != 2:
        raise ValueError(f"csi_data must be 2-D, got shape {csi_data.shape}")
    M, N = csi_data.shape
    if N == 0:
        logger.warning("No snapshots available for AoA estimation")
        return []

    # Determine array configuration
    if antenna_positions is not None:
        # Physical antenna array
        positions = np.asarray(antenna_positions).flatten()
        if len(positions) != M:
            raise ValueError(
                f"Number of antenna positions ({len(positions)}) must match "
                f"number of sensors ({M})"
            )
        d = np.median(np.diff(positions)) if len(positions) > 1 else 0.025
        logger.debug("Using physical antenna array with d=%.4f m", d)
    else:
        # Virtual array from subcarriers: treat each subcarrier as a
        # virtual sensor. Spacing is conceptual (subcarrier spacing).
        d = 0.5  # half-wavelength spacing in normalized units
        logger.debug("Using virtual subcarrier array (%d subcarriers)", M)

    # Default wavelength (2.4 GHz WiFi)
    wavelength = 0.125  # meters (c / 2.4e9)

    # Apply spatial smoothing
    subarray_size = min(M - 1, max(2, M // 2))
    if subarray_size < 2:
        subarray_size = M
    R_ss = spatial_smoothing(csi_data, subarray_size)

    # Build angle grid (fine resolution)
    angles_rad = np.linspace(-np.pi / 2, np.pi / 2, 361)  # -90 to 90 degrees

    # Build steering vectors for the smoothed subarray size
    steering_vectors = build_steering_ula(angles_rad, d, wavelength, subarray_size)

    # Compute MUSIC pseudospectrum
    n_sig = min(n_signals, subarray_size - 1)
    if n_sig < 1:
        n_sig = 1
    pseudospectrum = music_spectrum(R_ss, steering_vectors, n_sig)

    # Peak detection using scipy.signal.find_peaks
    # Normalize for robust peak finding
    ps_norm = pseudospectrum / (np.max(pseudospectrum) + np.finfo(float).eps)
    peaks, properties = find_peaks(ps_norm, height=0.1, distance=5)

    if len(peaks) == 0:
        logger.debug("No peaks found in MUSIC spectrum")
        return []

    # Sort peaks by prominence (descending)
    prominences = properties.get("prominences", ps_norm[peaks])
    sorted_idx = np.argsort(prominences)[::-1]
    peaks = peaks[sorted_idx]

    # Limit to at most n_signals peaks
    peaks = peaks[:n_sig]

    # Convert peak angles from radians to degrees
    estimated_angles_deg = np.degrees(angles_rad[peaks]).tolist()
    logger.debug("MUSIC estimated angles: %s", estimated_angles_deg)

    return estimated_angles_deg


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import matplotlib.pyplot as plt

    logging.basicConfig(level=logging.INFO)

    print("=" * 60)
    print("MUSIC AoA Estimation — Self-Test")
    print("=" * 60)

    # --- Test 1: Covariance estimation ---
    print("\n[Test 1] Covariance estimation")
    rng = np.random.default_rng(42)
    M, N = 8, 100
    X_test = rng.normal(size=(M, N)) + 1j * rng.normal(size=(M, N))
    R_test = estimate_covariance(X_test)
    assert R_test.shape == (M, M), f"Expected ({M},{M}), got {R_test.shape}"
    # Hermitian check
    assert np.allclose(R_test, R_test.conj().T), "Covariance not Hermitian"
    print(f"  Covariance shape: {R_test.shape} OK")
    print(f"  Hermitian: {'PASS' if np.allclose(R_test, R_test.conj().T) else 'FAIL'}")

    # --- Test 2: Build steering vectors ---
    print("\n[Test 2] ULA steering vectors")
    angles_deg = np.arange(-90, 91, 1)
    angles_rad = np.deg2rad(angles_deg)
    d = 0.5 * 0.125  # half-wavelength at 2.4 GHz
    wavelength = 0.125
    M_sv = 8
    sv = build_steering_ula(angles_rad, d, wavelength, M_sv)
    assert sv.shape == (M_sv, len(angles_rad)), f"Expected ({M_sv},{len(angles_rad)}), got {sv.shape}"
    # First element should be 1 for all angles
    assert np.allclose(sv[0, :], 1.0), "First steering element should be 1"
    print(f"  Steering vectors shape: {sv.shape} OK")

    # --- Test 3: MUSIC spectrum with synthetic data ---
    print("\n[Test 3] MUSIC spectrum on synthetic two-source data")
    M_syn = 8
    N_snap = 200
    true_angles_deg = [30.0, -20.0]
    true_angles_rad = np.deg2rad(true_angles_deg)
    wavelength_syn = 0.125
    d_syn = 0.5 * wavelength_syn

    # Build true steering vectors
    A = build_steering_ula(true_angles_rad, d_syn, wavelength_syn, M_syn)

    # Generate source signals
    src = rng.normal(size=(len(true_angles_deg), N_snap))

    # Snapshot matrix
    noise_power = 0.1
    X_syn = A @ src + np.sqrt(noise_power) * (
        rng.normal(size=(M_syn, N_snap)) + 1j * rng.normal(size=(M_syn, N_snap))
    )

    # Spatial smoothing
    R_ss_syn = spatial_smoothing(X_syn, subarray_size=6)

    # MUSIC
    angles_grid_deg = np.linspace(-90, 90, 721)
    angles_grid_rad = np.deg2rad(angles_grid_deg)
    sv_grid = build_steering_ula(angles_grid_rad, d_syn, wavelength_syn, 6)
    ps = music_spectrum(R_ss_syn, sv_grid, n_signals=2)

    # Find peaks
    ps_norm = ps / np.max(ps)
    peaks, _ = find_peaks(ps_norm, height=0.3, distance=20)
    found_angles = angles_grid_deg[peaks]
    print(f"  True angles: {true_angles_deg}")
    print(f"  Found peaks: {found_angles}")
    for ta in true_angles_deg:
        closest = min(found_angles, key=lambda x: abs(x - ta))
        print(f"    Angle {ta}: closest peak {closest:.1f} (error {abs(closest - ta):.1f})")

    # --- Test 4: estimate_aoa entry point ---
    print("\n[Test 4] estimate_aoa() entry point")
    aoa_est = estimate_aoa(X_syn, n_signals=2)
    print(f"  Estimated AoA: {aoa_est}")

    # --- Test 5: Virtual array mode with subcarriers ---
    print("\n[Test 5] Virtual array (subcarrier) mode")
    n_subcarriers = 52
    n_snapshots = 100
    X_virtual = rng.normal(
        size=(n_subcarriers, n_snapshots)
    ) + 1j * rng.normal(size=(n_subcarriers, n_snapshots))
    aoa_virtual = estimate_aoa(X_virtual, n_signals=1)
    print(f"  Virtual array AoA (noise only): {aoa_virtual}")

    # --- Test 6: Spatial smoothing ---
    print("\n[Test 6] Forward-backward spatial smoothing")
    X_coherent = np.tile(src[0:1, :], (M_syn, 1))  # coherent (rank-1)
    R_raw = estimate_covariance(X_coherent)
    rank_raw = np.linalg.matrix_rank(R_raw)
    R_smooth = spatial_smoothing(X_coherent, subarray_size=6)
    rank_smooth = np.linalg.matrix_rank(R_smooth, tol=1e-6)
    print(f"  Raw covariance rank: {rank_raw}")
    print(f"  Smoothed covariance rank: {rank_smooth}")

    # --- Visualization (optional) ---
    try:
        plt.figure(figsize=(10, 5))
        plt.plot(angles_grid_deg, 10 * np.log10(ps_norm + 1e-10))
        for ta in true_angles_deg:
            plt.axvline(ta, color="r", linestyle="--", alpha=0.7, label=f"True: {ta}")
        plt.xlabel("Angle (degrees)")
        plt.ylabel("MUSIC Spectrum (dB)")
        plt.title("MUSIC Pseudospectrum — Synthetic Two-Source Test")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.savefig("/mnt/agents/output/csi-tracking-system/src/music_test_plot.png")
        print("\n  Plot saved to music_test_plot.png")
    except Exception as e:
        print(f"\n  Plot skipped ({e})")

    print("\n" + "=" * 60)
    print("All MUSIC self-tests completed.")
    print("=" * 60)
