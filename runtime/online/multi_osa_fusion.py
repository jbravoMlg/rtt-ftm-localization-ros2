#!/usr/bin/env python3
"""
Multi-OSA Fusion Node — Online centralised multilateration from multiple drones.

Subscribes to ROS 2 topics published by all RTT-SearchAgent phones and runs
a centralised Gauss-Newton + IEKF3D estimator per detected AP, fusing RTT
measurements from every drone that sees it.

Usage:
    source /opt/ros/humble/setup.bash   # or iron/rolling
    python3 runtime/online/multi_osa_fusion.py [--hz 2] [--min-samples 6] [--log-dir ./data/fusion_logs]

Topics subscribed (auto-discovered per namespace):
    /{ns}/ftm_rtt          std_msgs/String   — raw RTT JSON per measurement
    /{ns}/phone/location   Float64MultiArray  — [lat, lon, alt, acc_m, t_ms]

Topics published:
    /fusion/anchor/{ap_id}/estimate   Float64MultiArray — fused [lat,lon,alt,cov_xx,cov_yy,cov_xy,n,hash]
    /fusion/status                    std_msgs/String   — periodic JSON summary

The estimator mirrors the phone-side pipeline:
    1. Accumulate (drone_position, range) pairs per AP from ALL drones.
    2. Bootstrap with Gauss-Newton 3D (≥ min_samples observations).
    3. Refine with IEKF3D (Joseph-form, NIS gating).
    4. Publish stable estimates when σ_H < threshold.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
import threading
import time
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from std_msgs.msg import String, Float64MultiArray
except ImportError:
    print("ERROR: rclpy not found. Source your ROS 2 workspace first:")
    print("  source /opt/ros/humble/setup.bash")
    sys.exit(1)


# ═══════════════════════════════════════════════════════════════════════════
#  Maths — ENU projection
# ═══════════════════════════════════════════════════════════════════════════

R_EARTH = 6_378_137.0


def deg2rad(d: float) -> float:
    return d * math.pi / 180.0


def rad2deg(r: float) -> float:
    return r * 180.0 / math.pi


def latlon_to_enu(
    lat: float, lon: float, alt: float,
    lat0: float, lon0: float, alt0: float,
) -> tuple[float, float, float]:
    """WGS-84 → local ENU (metres)."""
    d_lat = deg2rad(lat - lat0)
    d_lon = deg2rad(lon - lon0)
    mean_lat = deg2rad((lat + lat0) / 2.0)
    x = d_lon * math.cos(mean_lat) * R_EARTH   # East
    y = d_lat * R_EARTH                         # North
    z = alt - alt0                               # Up
    return x, y, z


def enu_to_latlon(
    x: float, y: float, z: float,
    lat0: float, lon0: float, alt0: float,
) -> tuple[float, float, float]:
    """Local ENU (metres) → WGS-84."""
    d_lat = y / R_EARTH
    d_lon = x / (R_EARTH * math.cos(deg2rad(lat0)))
    return lat0 + rad2deg(d_lat), lon0 + rad2deg(d_lon), alt0 + z


# ═══════════════════════════════════════════════════════════════════════════
#  Data structures
# ═══════════════════════════════════════════════════════════════════════════

@dataclass
class ENUSample:
    """One range observation in ENU coords."""
    x: float          # drone East (m)
    y: float          # drone North (m)
    z: float          # drone Up (m)
    r: float          # RTT distance (m)
    sigma: float      # measurement std (m)
    drone_ns: str     # which drone contributed this sample
    t_wall: float     # epoch seconds


_POSE_BUF_MAX = 50


@dataclass
class DroneState:
    """Latest known position of a drone."""
    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0
    acc_m: float = -1.0
    t_ms: float = 0.0
    updated: bool = False
    _pose_buf: list = field(default_factory=list)  # [(t_ms, lat, lon, alt)]

    def push_pose(self, t_ms: float, lat: float, lon: float, alt: float):
        """Append a timestamped pose to the circular buffer."""
        self._pose_buf.append((t_ms, lat, lon, alt))
        if len(self._pose_buf) > _POSE_BUF_MAX:
            self._pose_buf = self._pose_buf[-_POSE_BUF_MAX:]

    def interp_pose(self, t_ms: float) -> tuple[float, float, float] | None:
        """Linearly interpolate (lat, lon, alt) at t_ms."""
        buf = self._pose_buf
        if not buf:
            return None
        if len(buf) == 1:
            return buf[0][1], buf[0][2], buf[0][3]
        if t_ms <= buf[0][0]:
            return buf[0][1], buf[0][2], buf[0][3]
        if t_ms >= buf[-1][0]:
            return buf[-1][1], buf[-1][2], buf[-1][3]
        lo, hi = 0, len(buf) - 1
        while lo < hi - 1:
            mid = (lo + hi) // 2
            if buf[mid][0] <= t_ms:
                lo = mid
            else:
                hi = mid
        t0, lat0, lon0, alt0 = buf[lo]
        t1, lat1, lon1, alt1 = buf[hi]
        dt = t1 - t0
        if dt <= 0:
            return lat0, lon0, alt0
        alpha = (t_ms - t0) / dt
        return (
            lat0 + alpha * (lat1 - lat0),
            lon0 + alpha * (lon1 - lon0),
            alt0 + alpha * (alt1 - alt0),
        )


# ═══════════════════════════════════════════════════════════════════════════
#  Gauss-Newton 3D (ported from AnchorEstimator.kt)
# ═══════════════════════════════════════════════════════════════════════════

def _geometric_spread(samples: list[ENUSample]) -> float:
    """Return the 2D bounding-box diagonal of drone positions (m)."""
    xs = [s.x for s in samples]
    ys = [s.y for s in samples]
    dx = max(xs) - min(xs)
    dy = max(ys) - min(ys)
    return math.sqrt(dx * dx + dy * dy)


def gauss_newton_3d(
    samples: list[ENUSample],
    agl_hint: float = 10.0,
    max_iters: int = 15,
    tol: float = 1e-4,
    min_spread: float = 15.0,
) -> tuple[float, float, float, float] | None:
    """
    Robust 3D Gauss-Newton multilateration with WLS.

    Returns (x, y, z, sigma2) in ENU or None.
    Requires minimum geometric spread to avoid degenerate solutions.
    """
    n = len(samples)
    if n < 4:
        return None

    # Reject if drone positions are too clustered for reliable trilateration
    spread = _geometric_spread(samples)
    if spread < min_spread:
        return None

    # --- Linear 2D init (differences w.r.t. sample 0) ---
    s0 = samples[0]
    r0sq = s0.r * s0.r

    ata00 = ata01 = ata11 = atb0 = atb1 = 0.0
    for s in samples[1:]:
        risq = s.r * s.r
        a0 = 2.0 * (s.x - s0.x)
        a1 = 2.0 * (s.y - s0.y)
        c = r0sq - risq + s.x**2 - s0.x**2 + s.y**2 - s0.y**2
        w = 1.0 / (s.sigma * s.sigma)
        ata00 += w * a0 * a0
        ata01 += w * a0 * a1
        ata11 += w * a1 * a1
        atb0 += w * a0 * c
        atb1 += w * a1 * c

    det = ata00 * ata11 - ata01 * ata01
    if abs(det) < 1e-9:
        return None

    ax = (ata11 * atb0 - ata01 * atb1) / det
    ay = (-ata01 * atb0 + ata00 * atb1) / det

    # z init from AGL hint (median)
    z_candidates = sorted(s.z - agl_hint for s in samples)
    az = z_candidates[len(z_candidates) // 2]

    # --- Gauss-Newton iterations ---
    for _ in range(max_iters):
        j = [[0.0] * 3 for _ in range(3)]  # J^T W J
        g = [0.0, 0.0, 0.0]                # J^T W r

        for s in samples:
            dx = ax - s.x
            dy = ay - s.y
            dz = az - s.z
            di = math.sqrt(dx * dx + dy * dy + dz * dz + 1e-12)
            resid = di - s.r
            jac = [dx / di, dy / di, dz / di]
            w = 1.0 / (s.sigma * s.sigma)
            for a in range(3):
                for b in range(a, 3):
                    j[a][b] += w * jac[a] * jac[b]
                g[a] += w * jac[a] * resid

        # fill symmetric
        j[1][0] = j[0][1]
        j[2][0] = j[0][2]
        j[2][1] = j[1][2]

        # invert 3x3
        inv = _invert_3x3(j)
        if inv is None:
            return None

        step = [0.0, 0.0, 0.0]
        for a in range(3):
            for b in range(3):
                step[a] -= inv[a][b] * g[b]

        ax += step[0]
        ay += step[1]
        az += step[2]

        if math.sqrt(sum(s * s for s in step)) < tol:
            break

    # residual variance
    dof = max(1, n - 3)
    chi2 = 0.0
    for s in samples:
        dx = ax - s.x
        dy = ay - s.y
        dz = az - s.z
        di = math.sqrt(dx * dx + dy * dy + dz * dz + 1e-12)
        resid = di - s.r
        chi2 += (resid / s.sigma) ** 2

    # Sanity check: result should be within max_range of the drone cluster centroid
    cx = sum(s.x for s in samples) / n
    cy = sum(s.y for s in samples) / n
    max_range = max(s.r for s in samples) * 2.0
    if math.sqrt((ax - cx) ** 2 + (ay - cy) ** 2) > max_range:
        return None

    return ax, ay, az, chi2 / dof


def _invert_3x3(m: list[list[float]]) -> list[list[float]] | None:
    """Invert a 3×3 matrix. Returns None if singular."""
    a, b, c = m[0]
    d, e, f = m[1]
    g, h, i = m[2]
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < 1e-30:
        return None
    inv_det = 1.0 / det
    return [
        [(e * i - f * h) * inv_det, (c * h - b * i) * inv_det, (b * f - c * e) * inv_det],
        [(f * g - d * i) * inv_det, (a * i - c * g) * inv_det, (c * d - a * f) * inv_det],
        [(d * h - e * g) * inv_det, (b * g - a * h) * inv_det, (a * e - b * d) * inv_det],
    ]


# ═══════════════════════════════════════════════════════════════════════════
#  IEKF 3D (ported from AnchorIEKF3D.kt)
# ═══════════════════════════════════════════════════════════════════════════

class IEKF3D:
    """4-state Iterated Extended Kalman Filter: [x, y, z, bias] in ENU."""
    N = 4

    def __init__(self, x0: list[float], p0: list[list[float]]):
        self.x = list(x0)
        self.P = [row[:] for row in p0]
        self.n_accepted = 0
        self.n_rejected = 0
        self.last_nis = 0.0         # last computed NIS (normalised innov² / S)
        self.last_innov = 0.0       # last raw innovation (m)
        self._qx = 0.01
        self._qy = 0.01
        self._qz = 0.04
        self._qb = 0.0001

    @property
    def bias(self) -> float:
        return self.x[3]

    def predict(self):
        self.P[0][0] += self._qx
        self.P[1][1] += self._qy
        self.P[2][2] += self._qz
        self.P[3][3] += self._qb

    def update(
        self,
        z_meas: float,
        sigma: float,
        ue: float,
        un: float,
        uu: float,
        iters: int = 3,
        gate_gamma: float = 9.0,
    ) -> bool:
        """IEKF update with range measurement. Returns True if accepted."""
        if not math.isfinite(z_meas) or not math.isfinite(sigma) or sigma <= 0:
            return False

        R = sigma * sigma

        # Pre-gating
        dx = self.x[0] - ue
        dy = self.x[1] - un
        dz = self.x[2] - uu
        h = max(math.sqrt(dx * dx + dy * dy + dz * dz), 1e-6)
        innov = z_meas - (h + self.x[3])
        H = [dx / h, dy / h, dz / h, 1.0]
        S = _quad_form(H, self.P) + R
        if not math.isfinite(S) or S <= 0:
            return False
        nis = (innov * innov) / S
        self.last_innov = innov
        self.last_nis = nis
        if nis > gate_gamma:
            self.n_rejected += 1
            return False

        # IEKF iterations
        for _ in range(max(1, iters)):
            dx = self.x[0] - ue
            dy = self.x[1] - un
            dz = self.x[2] - uu
            h = max(math.sqrt(dx * dx + dy * dy + dz * dz), 1e-6)
            r = z_meas - (h + self.x[3])
            H = [dx / h, dy / h, dz / h, 1.0]
            S = _quad_form(H, self.P) + R
            if not math.isfinite(S) or S <= 0:
                return False

            # K = P H^T / S
            PHt = _mat_vec(self.P, H)
            K = [ph / S for ph in PHt]

            # x += K * r
            for i in range(self.N):
                self.x[i] += K[i] * r

            # Joseph form: P = (I-KH) P (I-KH)^T + K R K^T
            IKH = [[0.0] * self.N for _ in range(self.N)]
            for i in range(self.N):
                for j in range(self.N):
                    IKH[i][j] = (1.0 if i == j else 0.0) - K[i] * H[j]

            P_new = _mat_mul(IKH, _mat_mul(self.P, _transpose(IKH)))
            for i in range(self.N):
                for j in range(self.N):
                    P_new[i][j] += K[i] * R * K[j]
            self.P = P_new

        self.n_accepted += 1
        return True

    def update_z_prior(self, z_prior: float, sigma_z: float = 10.0):
        """Soft constraint on z (ground-height prior) — full covariance update."""
        R = sigma_z * sigma_z
        H = [0.0, 0.0, 1.0, 0.0]
        S = _quad_form(H, self.P) + R
        if not math.isfinite(S) or S <= 0:
            return
        innov = z_prior - self.x[2]
        PHt = _mat_vec(self.P, H)
        K = [ph / S for ph in PHt]
        for i in range(self.N):
            self.x[i] += K[i] * innov
        IKH = [[0.0] * self.N for _ in range(self.N)]
        for i in range(self.N):
            for j in range(self.N):
                IKH[i][j] = (1.0 if i == j else 0.0) - K[i] * H[j]
        P_new = _mat_mul(IKH, _mat_mul(self.P, _transpose(IKH)))
        for i in range(self.N):
            for j in range(self.N):
                P_new[i][j] += K[i] * R * K[j]
        self.P = P_new

    @property
    def sigma_h(self) -> float:
        return math.sqrt(self.P[0][0] + self.P[1][1])

    @property
    def sigma_z(self) -> float:
        return math.sqrt(self.P[2][2])


def _quad_form(h: list[float], P: list[list[float]]) -> float:
    n = len(h)
    val = 0.0
    for i in range(n):
        for j in range(n):
            val += h[i] * P[i][j] * h[j]
    return val


def _mat_vec(M: list[list[float]], v: list[float]) -> list[float]:
    n = len(v)
    return [sum(M[i][j] * v[j] for j in range(n)) for i in range(n)]


def _mat_mul(A: list[list[float]], B: list[list[float]]) -> list[list[float]]:
    n = len(A)
    return [
        [sum(A[i][k] * B[k][j] for k in range(n)) for j in range(n)]
        for i in range(n)
    ]


def _transpose(M: list[list[float]]) -> list[list[float]]:
    n = len(M)
    return [[M[j][i] for j in range(n)] for i in range(n)]


# ═══════════════════════════════════════════════════════════════════════════
#  MAD-based RTT gate (ported from RobustRttGate.kt)
# ═══════════════════════════════════════════════════════════════════════════

class RttGate:
    """Median Absolute Deviation gating for RTT outlier rejection."""

    def __init__(self, window_ms: int = 1500, max_n: int = 30, min_n: int = 8, k: float = 3.5):
        self._window_ms = window_ms
        self._max_n = max_n
        self._min_n = min_n
        self._k = k
        self._sigma_floor = 0.12
        self._buf: list[tuple[float, float]] = []  # (t_ms, distance)

    def push_and_gate(self, t_ms: float, d: float, sigma: float) -> tuple[bool, float]:
        """Returns (accepted, robust_sigma)."""
        cutoff = t_ms - self._window_ms
        self._buf = [(t, v) for t, v in self._buf if t >= cutoff]
        self._buf.append((t_ms, d))
        if len(self._buf) > self._max_n:
            self._buf = self._buf[-self._max_n:]

        if len(self._buf) < self._min_n:
            return True, max(sigma, self._sigma_floor)  # warm-up: accept all

        vals = sorted(v for _, v in self._buf)
        median = vals[len(vals) // 2]
        abs_devs = sorted(abs(v - median) for v in vals)
        mad = abs_devs[len(abs_devs) // 2] * 1.4826  # scale to σ

        robust_sigma = max(mad, self._sigma_floor)
        if abs(d - median) > self._k * robust_sigma:
            return False, robust_sigma

        return True, robust_sigma


# ═══════════════════════════════════════════════════════════════════════════
#  Per-AP Anchor Track
# ═══════════════════════════════════════════════════════════════════════════

class AnchorTrack:
    """State for one AP being localised by the fusion node."""

    def __init__(self, bssid: str):
        self.bssid = bssid
        self.samples: list[ENUSample] = []
        self.ekf: IEKF3D | None = None
        self.gate = RttGate()
        self.lock = threading.Lock()
        self.contributing_drones: set[str] = set()

        # Stability
        self.stable_streak = 0
        self.converging_streak = 0
        self.prev_sigma_h = float("inf")
        self.last_pub_xyz: tuple[float, float, float] | None = None
        self.n_published = 0

        # Divergence detection: rolling window of normalised innovations.
        # If the recent innovations are consistently outside their
        # expected distribution we reset the EKF back to GN bootstrap.
        self._nis_window: list[float] = []      # last N NIS values
        self._nis_window_max = 20
        self._consecutive_bad = 0
        self.n_resets = 0

        # z-ground median buffer
        self._z_ground: list[float] = []

    def reset_ekf(self, reason: str = ""):
        """Drop the EKF state and go back to the GN bootstrap phase.

        Called by the fusion node when sustained large innovations suggest
        the filter has diverged from the true anchor position.
        """
        self.ekf = None
        self.samples.clear()
        self.stable_streak = 0
        self.converging_streak = 0
        self.prev_sigma_h = float("inf")
        self.last_pub_xyz = None
        self._nis_window.clear()
        self._consecutive_bad = 0
        self.n_resets += 1

    def z_ground_median(self) -> float | None:
        if not self._z_ground:
            return None
        s = sorted(self._z_ground)
        return s[len(s) // 2]

    def push_z_ground(self, z: float):
        self._z_ground.append(z)
        if len(self._z_ground) > 25:
            self._z_ground = self._z_ground[-25:]


# ═══════════════════════════════════════════════════════════════════════════
#  ROS 2 Fusion Node
# ═══════════════════════════════════════════════════════════════════════════

class MultiOsaFusionNode(Node):
    """
    ROS 2 node that discovers RTT-SearchAgent phones, subscribes to their
    topics, and runs centralised multilateration per AP.
    """

    def __init__(
        self,
        min_samples: int = 6,
        agl_hint: float = 10.0,
        range_bias: float = 0.0,
        sigma_h_threshold: float = 2.5,
        gnss_acc_max: float = 5.0,
        z_prior_sigma: float = 5.0,
        log_dir: str | None = None,
        discover_hz: float = 0.2,
        reset_on_divergence: bool = True,
        divergence_reject_streak: int = 8,
        divergence_nis_mean: float = 6.0,
    ):
        super().__init__("multi_osa_fusion")
        self.get_logger().info("Multi-OSA Fusion Node starting …")

        self._min_samples = min_samples
        self._agl_hint = agl_hint
        self._range_bias = range_bias
        self._sigma_h_threshold = sigma_h_threshold
        self._gnss_acc_max = gnss_acc_max
        self._z_prior_sigma = z_prior_sigma
        self._reset_on_divergence = reset_on_divergence
        self._divergence_reject_streak = max(3, divergence_reject_streak)
        self._divergence_nis_mean = max(1.0, divergence_nis_mean)

        # ENU origin (set from first GNSS fix)
        self._origin: tuple[float, float, float] | None = None
        self._origin_lock = threading.Lock()

        # Per-drone latest GNSS
        self._drones: dict[str, DroneState] = {}
        self._drones_lock = threading.Lock()

        # Per-AP tracks
        self._tracks: dict[str, AnchorTrack] = {}
        self._tracks_lock = threading.Lock()

        # Known subscriptions (to avoid duplicates)
        self._subscribed_ns: set[str] = set()
        self._subscribed_estimate_topics: set[str] = set()
        self._subs: list = []  # keep references alive

        # Publisher cache
        self._anchor_pubs: dict[str, object] = {}
        self._state_pubs: dict[str, object] = {}
        self._status_pub = self.create_publisher(String, "/fusion/status", 10)

        # Logging
        self._log_dir: Path | None = None
        self._log_files: dict[str, object] = {}
        if log_dir:
            self._log_dir = Path(log_dir)
            self._log_dir.mkdir(parents=True, exist_ok=True)

        # QoS for best-effort (matching phone publishers)
        self._qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        # Discovery timer
        self._discover_timer = self.create_timer(
            1.0 / discover_hz, self._discover_namespaces
        )
        # Status timer (every 2s)
        self._status_timer = self.create_timer(2.0, self._publish_status)

        self.get_logger().info(
            f"Config: min_samples={min_samples}, agl_hint={agl_hint}, "
            f"range_bias={range_bias}, "
            f"sigma_h_threshold={sigma_h_threshold}, gnss_acc_max={gnss_acc_max}, "
            f"z_prior_sigma={z_prior_sigma}, "
            f"reset_on_divergence={reset_on_divergence} "
            f"(streak={self._divergence_reject_streak}, "
            f"nis_mean={self._divergence_nis_mean:.1f})"
        )

    # ─── Discovery ───────────────────────────────────────────────────────

    def _discover_namespaces(self):
        """Discover new RTT-SearchAgent namespaces from topic list."""
        topic_list = self.get_topic_names_and_types()
        pattern = re.compile(r"^/([^/]+)/ftm_rtt$")
        est_pattern = re.compile(r"^/([^/]+)/ftm/anchor/(ap_[0-9a-f]+)/estimate$")

        for topic_name, _ in topic_list:
            m = pattern.match(topic_name)
            if m:
                ns = m.group(1)
                if ns not in self._subscribed_ns and ns != "fusion":
                    self._subscribe_to_drone(ns)

            # Cross-initialisation: subscribe to phone-side anchor estimates
            em = est_pattern.match(topic_name)
            if em:
                ns_est = em.group(1)
                if topic_name not in self._subscribed_estimate_topics and ns_est != "fusion":
                    self._subscribed_estimate_topics.add(topic_name)
                    ap_id = em.group(2)
                    sub = self.create_subscription(
                        Float64MultiArray,
                        topic_name,
                        lambda msg, _ns=ns_est, _ap=ap_id: self._on_phone_estimate(msg, _ns, _ap),
                        self._qos,
                    )
                    self._subs.append(sub)
                    self.get_logger().info(
                        f"Subscribed to phone estimate: {topic_name}"
                    )

    def _subscribe_to_drone(self, ns: str):
        """Subscribe to a drone's RTT and location topics."""
        self._subscribed_ns.add(ns)
        self.get_logger().info(f"Subscribing to drone: {ns}")

        # RTT topic (std_msgs/String with JSON)
        rtt_sub = self.create_subscription(
            String,
            f"/{ns}/ftm_rtt",
            lambda msg, _ns=ns: self._on_rtt(msg, _ns),
            self._qos,
        )
        self._subs.append(rtt_sub)

        # Location topic (Float64MultiArray)
        loc_sub = self.create_subscription(
            Float64MultiArray,
            f"/{ns}/phone/location",
            lambda msg, _ns=ns: self._on_location(msg, _ns),
            self._qos,
        )
        self._subs.append(loc_sub)

        with self._drones_lock:
            self._drones[ns] = DroneState()

    # ─── Callbacks ───────────────────────────────────────────────────────

    def _on_location(self, msg: Float64MultiArray, ns: str):
        """Handle phone/location: [lat, lon, alt, acc_m, t_ms]."""
        data = list(msg.data)
        if len(data) < 5:
            return

        lat, lon, alt, acc, t_ms = data[0], data[1], data[2], data[3], data[4]

        # Reject poor GNSS
        if acc > 0 and acc > self._gnss_acc_max:
            return

        # Set ENU origin from first fix with good accuracy
        with self._origin_lock:
            if self._origin is None:
                if acc <= 0 or acc > 2.0:
                    self.get_logger().warning(
                        f"Waiting for RTK-quality fix for ENU origin "
                        f"(acc={acc:.1f}m, need 0 < acc <= 2.0m, from {ns})",
                        throttle_duration_sec=5.0,
                    )
                else:
                    self._origin = (lat, lon, alt)
                    self.get_logger().info(
                        f"ENU origin set: lat={lat:.7f}, lon={lon:.7f}, alt={alt:.1f} "
                        f"acc={acc:.2f}m (from {ns})"
                    )

        with self._drones_lock:
            ds = self._drones.get(ns)
            if ds is None:
                ds = DroneState()
                self._drones[ns] = ds
            ds.lat = lat
            ds.lon = lon
            ds.alt = alt
            ds.acc_m = acc
            ds.t_ms = t_ms
            ds.updated = True
            ds.push_pose(t_ms, lat, lon, alt)

    def _on_rtt(self, msg: String, ns: str):
        """Handle ftm_rtt: JSON with distance, bssid, etc."""
        try:
            j = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        bssid = j.get("bssid", "")
        if not bssid:
            return

        accepted = j.get("accepted", True)
        if accepted is False or accepted == "false":
            return  # already rejected by phone's MAD gate

        dist_m = j.get("distance_m", -1.0)
        sigma_m = j.get("sigma_used_m", j.get("std_m", 1.0))
        if dist_m <= 0 or sigma_m <= 0:
            return

        t_wall = j.get("t_ms", time.time() * 1000)
        t_mono = j.get("t_mono_ms", t_wall)

        # Get drone position interpolated at time of RTT measurement
        with self._drones_lock:
            ds = self._drones.get(ns)
            if ds is None or not ds.updated:
                return
            interp = ds.interp_pose(t_wall)
            if interp is not None:
                drone_lat, drone_lon, drone_alt = interp
            else:
                drone_lat, drone_lon, drone_alt = ds.lat, ds.lon, ds.alt

        # Convert to ENU
        with self._origin_lock:
            origin = self._origin
        if origin is None:
            return

        ue, un, uu = latlon_to_enu(drone_lat, drone_lon, drone_alt, *origin)

        # Retrieve drone GPS accuracy for R inflation
        with self._drones_lock:
            gps_acc_m = ds.acc_m if ds is not None else -1.0

        # Process measurement
        self._process_rtt(
            bssid=bssid,
            dist_m=dist_m,
            sigma_m=sigma_m,
            ue=ue, un=un, uu=uu,
            drone_ns=ns,
            t_wall=t_wall / 1000.0,
            t_mono=t_mono,
            gps_acc_m=gps_acc_m,
        )

    def _on_phone_estimate(self, msg: Float64MultiArray, ns: str, ap_id: str):
        """Cross-initialisation: use a phone's converged anchor estimate to
        seed the fusion EKF, skipping the GN bootstrap phase."""
        data = list(msg.data)
        if len(data) < 7:
            return

        lat, lon, alt = data[0], data[1], data[2]
        cov_xx, cov_yy = data[3], data[4]
        n_samples = data[6]

        # Only use estimates with enough samples
        if n_samples < 10:
            return

        # Convert ap_XXXXXXXXXXXX back to colon-separated BSSID
        hex_part = ap_id.replace("ap_", "")
        if len(hex_part) == 12:
            bssid = ":".join(hex_part[i:i+2] for i in range(0, 12, 2))
        else:
            bssid = hex_part

        with self._tracks_lock:
            track = self._tracks.get(bssid)
            if track is None:
                track = AnchorTrack(bssid)
                self._tracks[bssid] = track

        with track.lock:
            # Only cross-init if fusion hasn't initialised its own EKF yet
            if track.ekf is not None:
                return

            with self._origin_lock:
                origin = self._origin
            if origin is None:
                return

            ue, un, uu = latlon_to_enu(lat, lon, alt, *origin)

            # Use the phone's covariance to set the initial P
            p0_h = max(cov_xx, cov_yy, 4.0)   # at least 2m σ
            track.ekf = IEKF3D(
                x0=[ue, un, uu, 0.0],
                p0=[
                    [p0_h,  0.0,   0.0,   0.0],
                    [0.0,   p0_h,  0.0,   0.0],
                    [0.0,   0.0,   100.0, 0.0],
                    [0.0,   0.0,   0.0,   4.0],
                ],
            )
            track.contributing_drones.add(ns)
            track.samples.clear()
            self.get_logger().info(
                f"[{bssid}] EKF cross-initialised from {ns} phone estimate "
                f"at ENU=({ue:.1f}, {un:.1f}, {uu:.1f}), "
                f"n_phone={n_samples:.0f}, p0_h={p0_h:.1f}"
            )

    # ─── Core estimation pipeline ───────────────────────────────────────

    def _process_rtt(
        self,
        bssid: str,
        dist_m: float,
        sigma_m: float,
        ue: float, un: float, uu: float,
        drone_ns: str,
        t_wall: float,
        t_mono: float,
        gps_acc_m: float = -1.0,
    ):
        with self._tracks_lock:
            track = self._tracks.get(bssid)
            if track is None:
                track = AnchorTrack(bssid)
                self._tracks[bssid] = track

        with track.lock:
            self._process_rtt_inner(
                track, bssid, dist_m, sigma_m, ue, un, uu,
                drone_ns, t_wall, t_mono, gps_acc_m,
            )

    def _process_rtt_inner(
        self, track, bssid, dist_m, sigma_m, ue, un, uu, drone_ns, t_wall, t_mono,
        gps_acc_m=-1.0,
    ):
        # 0) Range bias correction
        dist_m = max(0.0, dist_m - self._range_bias)

        # 1) Local MAD gate (fusion-side, on top of phone-side gate)
        accepted, robust_sigma = track.gate.push_and_gate(t_mono, dist_m, sigma_m)
        if not accepted:
            return

        eff_sigma = max(robust_sigma, 0.12)

        # Inflate R with GPS position uncertainty: R_eff = σ_RTT² + σ_GPS²
        rtk_quality = gps_acc_m > 0 and gps_acc_m <= 2.0
        if gps_acc_m > 0:
            eff_sigma = math.sqrt(eff_sigma * eff_sigma + gps_acc_m * gps_acc_m)

        # z ground estimate
        track.push_z_ground(uu - self._agl_hint)

        sample = ENUSample(
            x=ue, y=un, z=uu, r=dist_m, sigma=eff_sigma,
            drone_ns=drone_ns, t_wall=t_wall,
        )

        # 2) Bootstrap: accumulate samples for GN initialisation
        if track.ekf is None:
            # Only use RTK-quality fixes for bootstrap (acc ≤ 2m)
            if not rtk_quality:
                return
            track.samples.append(sample)
            if len(track.samples) < self._min_samples:
                return

            init = gauss_newton_3d(
                track.samples, agl_hint=self._agl_hint, max_iters=15
            )
            if init is None:
                spread = _geometric_spread(track.samples)
                self.get_logger().info(
                    f"[{bssid}] GN init deferred: {len(track.samples)} samples, "
                    f"spread={spread:.2f} m"
                )
                return

            ax, ay, az, _ = init
            z_prior = track.z_ground_median()
            if z_prior is not None and abs(az - z_prior) > 20.0:
                az = z_prior

            track.ekf = IEKF3D(
                x0=[ax, ay, az, 0.0],
                p0=[
                    [25.0, 0.0, 0.0, 0.0],
                    [0.0, 25.0, 0.0, 0.0],
                    [0.0, 0.0, 100.0, 0.0],
                    [0.0, 0.0, 0.0, 4.0],
                ],
            )
            track.contributing_drones = set(s.drone_ns for s in track.samples)
            n_boot = len(track.samples)
            track.samples.clear()
            self.get_logger().info(
                f"[{bssid}] EKF initialised from {n_boot} samples "
                f"({len(track.contributing_drones)} drones) "
                f"at ENU=({ax:.1f}, {ay:.1f}, {az:.1f})"
            )
            return

        # 3) EKF predict + update
        ekf = track.ekf
        ekf.predict()
        ok = ekf.update(
            z_meas=dist_m, sigma=eff_sigma,
            ue=ue, un=un, uu=uu,
            iters=3, gate_gamma=9.0,
        )

        # Track NIS rolling window for divergence detection. We include
        # both accepted and rejected updates — rejection itself is a
        # strong signal of a bad state.
        nis_val = ekf.last_nis
        if math.isfinite(nis_val):
            track._nis_window.append(nis_val)
            if len(track._nis_window) > track._nis_window_max:
                track._nis_window = track._nis_window[-track._nis_window_max:]

        if not ok:
            track._consecutive_bad += 1
            # Publish raw state so the GUI can still show the pre-stable
            # EKF position even when the update is rejected.
            self._publish_state(track, bssid, drone_ns, converged=False)
            self._maybe_reset(track, bssid)
            return

        track._consecutive_bad = 0
        track.contributing_drones.add(drone_ns)

        # z prior regularisation
        z_prior = track.z_ground_median()
        if z_prior is not None:
            ekf.update_z_prior(z_prior, sigma_z=self._z_prior_sigma)

        # 4) Stability check
        sigma_h = ekf.sigma_h
        sigma_z_val = ekf.sigma_z

        # Publish raw (not-yet-stable) EKF state on /fusion/anchor/{ap}/state
        # so the GUI can visualise convergence in real time.
        self._publish_state(track, bssid, drone_ns, converged=False)

        # Log every EKF update (raw state) for convergence analysis
        with self._origin_lock:
            origin = self._origin
        if origin is not None:
            xe_r, yn_r, zu_r = ekf.x[0], ekf.x[1], ekf.x[2]
            lat_r, lon_r, alt_r = enu_to_latlon(xe_r, yn_r, zu_r, *origin)
            self._log_entry("fusion_state.jsonl", {
                "bssid": bssid,
                "lat": lat_r, "lon": lon_r, "alt": alt_r,
                "enu_x": xe_r, "enu_y": yn_r, "enu_z": zu_r,
                "sigma_h": sigma_h, "sigma_z": sigma_z_val,
                "bias_m": ekf.bias,
                "n_accepted": ekf.n_accepted,
                "n_drones": len(track.contributing_drones),
                "converging_streak": track.converging_streak,
                "nis": ekf.last_nis,
                "n_resets": track.n_resets,
                "stable": False,
                "drone": drone_ns,
                "t": time.time(),
            })

        is_converging = sigma_h < track.prev_sigma_h * 1.02
        track.converging_streak = track.converging_streak + 1 if is_converging else 0
        track.prev_sigma_h = sigma_h

        min_acc = 10 if track.converging_streak >= 5 else 15
        need_streak = 2 if track.converging_streak >= 8 else 3

        # Jump check
        lp = track.last_pub_xyz
        jump_xy = 0.0
        if lp is not None:
            jump_xy = math.sqrt((ekf.x[0] - lp[0]) ** 2 + (ekf.x[1] - lp[1]) ** 2)
        max_jump = max(1.0, sigma_h * 2.0)

        stable = (
            ekf.n_accepted >= min_acc
            and math.isfinite(sigma_h) and sigma_h < self._sigma_h_threshold
            and math.isfinite(sigma_z_val) and sigma_z_val < 8.0
            and jump_xy < max_jump
        )

        track.stable_streak = track.stable_streak + 1 if stable else 0

        # Divergence check runs even on accepted updates so a sustained
        # bias in innovations (e.g. bad geometry or a phantom peak) is
        # caught early.
        self._maybe_reset(track, bssid)

        if track.stable_streak < need_streak:
            return

        # 5) Publish
        xe, yn, zu = ekf.x[0], ekf.x[1], ekf.x[2]
        track.last_pub_xyz = (xe, yn, zu)
        track.n_published += 1

        with self._origin_lock:
            origin = self._origin
        if origin is None:
            return

        lat_e, lon_e, alt_e = enu_to_latlon(xe, yn, zu, *origin)
        cov_xx = ekf.P[0][0]
        cov_yy = ekf.P[1][1]
        cov_xy = ekf.P[0][1]
        n_acc = float(ekf.n_accepted)

        # Count contributing drones
        n_drones = len(track.contributing_drones)

        self._publish_estimate(
            bssid, lat_e, lon_e, alt_e,
            cov_xx, cov_yy, cov_xy, n_acc,
        )
        # Flag the raw state as converged so GUI clients can distinguish
        # "stable/published" from "still converging".
        self._publish_state(track, bssid, drone_ns, converged=True)

        # Log
        self._log_entry("fusion_mlat.jsonl", {
            "bssid": bssid,
            "lat": lat_e, "lon": lon_e, "alt": alt_e,
            "enu_x": xe, "enu_y": yn, "enu_z": zu,
            "cov_xx": cov_xx, "cov_yy": cov_yy, "cov_xy": cov_xy,
            "sigma_h": sigma_h, "sigma_z": sigma_z_val,
            "bias_m": ekf.bias,
            "n_accepted": ekf.n_accepted,
            "n_drones": n_drones,
            "n_published": track.n_published,
            "t": time.time(),
        })

        if track.n_published % 10 == 1:
            self.get_logger().info(
                f"[{bssid}] STABLE #{track.n_published}: "
                f"({lat_e:.7f}, {lon_e:.7f}, {alt_e:.1f}) "
                f"σ_H={sigma_h:.2f}m, n={ekf.n_accepted}, "
                f"drones={n_drones}"
            )

    # ─── Publishing ──────────────────────────────────────────────────────

    def _publish_estimate(
        self, bssid: str,
        lat: float, lon: float, alt: float,
        cov_xx: float, cov_yy: float, cov_xy: float,
        n_samples: float,
    ):
        ap_id = "ap_" + bssid.replace(":", "")
        topic = f"/fusion/anchor/{ap_id}/estimate"

        if topic not in self._anchor_pubs:
            self._anchor_pubs[topic] = self.create_publisher(
                Float64MultiArray, topic, 10
            )

        msg = Float64MultiArray()
        msg.data = [lat, lon, alt, cov_xx, cov_yy, cov_xy, n_samples, 0.0]
        self._anchor_pubs[topic].publish(msg)

    def _publish_state(
        self, track: "AnchorTrack", bssid: str,
        drone_ns: str, converged: bool,
    ):
        """Publish the current raw EKF state on /fusion/anchor/{ap}/state.

        Mirrors the phone-side `ftm/anchor/{ap}/state` topic so the GUI
        map can show the in-progress fusion estimate even before it
        becomes stable. Silently no-ops if the ENU origin or EKF are
        not ready yet.

        Payload: [lat, lon, alt, sigma_h, sigma_z, n_accepted, converged, hash]
        """
        ekf = track.ekf
        if ekf is None:
            return
        with self._origin_lock:
            origin = self._origin
        if origin is None:
            return
        lat, lon, alt = enu_to_latlon(ekf.x[0], ekf.x[1], ekf.x[2], *origin)
        ap_id = "ap_" + bssid.replace(":", "")
        topic = f"/fusion/anchor/{ap_id}/state"
        pub = self._state_pubs.get(topic)
        if pub is None:
            pub = self.create_publisher(Float64MultiArray, topic, 10)
            self._state_pubs[topic] = pub
        msg = Float64MultiArray()
        msg.data = [
            lat, lon, alt,
            float(ekf.sigma_h), float(ekf.sigma_z),
            float(ekf.n_accepted),
            1.0 if converged else 0.0,
            0.0,
        ]
        try:
            pub.publish(msg)
        except Exception:
            pass

    def _maybe_reset(self, track: "AnchorTrack", bssid: str):
        """Reset EKF back to GN bootstrap if recent updates look divergent.

        Two independent triggers:
          1. `divergence_reject_streak` consecutive rejections (NIS > γ).
          2. Mean NIS over the rolling window above `divergence_nis_mean`,
             which indicates persistent over-confidence / bias.
        """
        if not self._reset_on_divergence or track.ekf is None:
            return

        trigger = None
        if track._consecutive_bad >= self._divergence_reject_streak:
            trigger = (
                f"{track._consecutive_bad} consecutive rejections "
                f"(>= {self._divergence_reject_streak})"
            )
        elif (
            len(track._nis_window) >= track._nis_window_max
            and (sum(track._nis_window) / len(track._nis_window))
                >= self._divergence_nis_mean
        ):
            mean_nis = sum(track._nis_window) / len(track._nis_window)
            trigger = (
                f"mean NIS over last {len(track._nis_window)} = "
                f"{mean_nis:.2f} (>= {self._divergence_nis_mean:.2f})"
            )

        if trigger is None:
            return

        self.get_logger().warning(
            f"[{bssid}] Divergence detected: {trigger}. "
            f"Resetting EKF (reset #{track.n_resets + 1})."
        )
        track.reset_ekf(reason=trigger)
        self._log_entry("fusion_state.jsonl", {
            "bssid": bssid,
            "event": "reset",
            "trigger": trigger,
            "n_resets": track.n_resets,
            "t": time.time(),
        })

    def _publish_status(self):
        """Periodic status summary."""
        with self._tracks_lock:
            tracks_snap = dict(self._tracks)

        with self._drones_lock:
            n_drones = len(self._drones)
            drone_list = list(self._drones.keys())

        status = {
            "t": time.time(),
            "n_drones": n_drones,
            "drones": drone_list,
            "n_aps": len(tracks_snap),
            "aps": {},
        }

        for bssid, track in tracks_snap.items():
            ekf = track.ekf
            ap_info: dict = {
                "n_samples": len(track.samples),
                "n_drones": len(track.contributing_drones),
            }
            if ekf is not None:
                ap_info["n_accepted"] = ekf.n_accepted
                ap_info["sigma_h"] = round(ekf.sigma_h, 3)
                ap_info["sigma_z"] = round(ekf.sigma_z, 3)
                ap_info["converged"] = ekf.sigma_h < self._sigma_h_threshold
                ap_info["enu"] = [round(ekf.x[0], 2), round(ekf.x[1], 2), round(ekf.x[2], 2)]
            status["aps"][bssid] = ap_info

        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)

    # ─── Logging ─────────────────────────────────────────────────────────

    def _log_entry(self, filename: str, data: dict):
        if self._log_dir is None:
            return
        if filename not in self._log_files:
            path = self._log_dir / filename
            self._log_files[filename] = open(path, "a", buffering=1, encoding="utf-8")
        f = self._log_files[filename]
        f.write(json.dumps(data) + "\n")

    def destroy_node(self):
        for f in self._log_files.values():
            try:
                f.close()
            except Exception:
                pass
        super().destroy_node()


# ═══════════════════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="Multi-OSA Fusion — centralised multilateration from multiple drones",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # ── Flags disponibles ────────────────────────────────────────────────
    #
    # --min-samples N
    #     Número mínimo de observaciones RTT (de cualquier combinación de
    #     drones) que se necesitan para un AP antes de intentar la
    #     inicialización por Gauss-Newton 3D.  Un valor bajo arranca antes
    #     pero con peor geometría; uno alto espera a tener mejor
    #     distribución espacial.  Default: 6.
    #
    # --agl-hint M
    #     Altitud AGL (Above Ground Level) aproximada a la que vuelan los
    #     drones, en metros.  Se usa para calcular la estimación inicial
    #     de la coordenada z del AP: z_ap ≈ z_dron − agl_hint.  Si los
    #     drones vuelan a distintas alturas, usar un valor intermedio.
    #     Default: 10.0.
    #
    # --range-bias B
    #     Sesgo sistemático (m) del hardware RTT.  Se resta de cada
    #     distancia cruda antes de gating y filtro.  Un valor positivo
    #     indica que el hardware sobreestima la distancia (ej: +1.5 m
    #     medido en calibración).  Default: 0.0.
    #
    # --sigma-threshold S
    #     Umbral de incertidumbre horizontal (σ_H = √(σ_xx + σ_yy)) del
    #     IEKF por debajo del cual una estimación se considera "estable"
    #     y se publica/loguea.  Valores típicos: 1.0-3.0 m.  Default: 2.5.
    #
    # --gnss-acc-max A
    #     Descarta posiciones GNSS del dron cuya accuracy reportada sea
    #     mayor que A metros.  Evita contaminar la multilateración con
    #     fixes malos (ej: GNSS sin RTK en entorno urbano).  Default: 5.0
    #     (fix RTK de buena calidad).  Subir a 15 si se vuela sin RTK.
    #
    # --log-dir DIR
    #     Directorio donde se escribe el archivo fusion_mlat.jsonl con
    #     una línea JSON por cada estimación estable publicada.  Si no se
    #     indica, no se guarda ningún log.  Ejemplo: --log-dir ./data/fusion_logs
    #
    # --discover-hz F
    #     Frecuencia (en Hz) con la que el nodo escanea el grafo de ROS 2
    #     buscando nuevos namespaces (drones) que publiquen ftm_rtt.
    #     0.2 Hz = una búsqueda cada 5 segundos.  Subir si los drones se
    #     encienden en momentos muy distintos.  Default: 0.2.
    #
    # ─────────────────────────────────────────────────────────────────────

    parser.add_argument("--min-samples", type=int, default=6,
                        help="Mín. observaciones RTT por AP para inicializar Gauss-Newton 3D")
    parser.add_argument("--agl-hint", type=float, default=10.0,
                        help="Altitud AGL aprox. de los drones (m), para estimación inicial de z")
    parser.add_argument("--range-bias", type=float, default=0.0,
                        help="Sesgo RTT a restar de cada distancia (m). Positivo = hardware sobreestima")
    parser.add_argument("--sigma-threshold", type=float, default=2.5,
                        help="Umbral σ_H (m) para publicar estimación estable")
    parser.add_argument("--gnss-acc-max", type=float, default=5.0,
                        help="Descartar fixes GNSS con accuracy > este valor (m). "
                             "5 m ≈ fix RTK de buena calidad; subir a 15 en "
                             "entornos urbanos sin RTK (a costa de peor σ)")
    parser.add_argument("--z-prior-sigma", type=float, default=5.0,
                        help="σ del prior de altitud z (m). Menor = prior más fuerte")
    parser.add_argument("--log-dir", type=str, default=None,
                        help="Directorio para logs JSONL de fusión (fusion_mlat.jsonl)")
    parser.add_argument("--discover-hz", type=float, default=0.2,
                        help="Frecuencia (Hz) de escaneo de nuevos drones en el grafo ROS 2")
    parser.add_argument("--reset-on-divergence",
                        dest="reset_on_divergence", action="store_true",
                        default=True,
                        help="Reinicia el EKF si detecta divergencia persistente "
                             "(rachas de rechazo o NIS medio alto)")
    parser.add_argument("--no-reset-on-divergence",
                        dest="reset_on_divergence", action="store_false",
                        help="Desactiva el reset automático del EKF")
    parser.add_argument("--divergence-reject-streak", type=int, default=8,
                        help="Nº de rechazos consecutivos por gate NIS que "
                             "disparan un reset")
    parser.add_argument("--divergence-nis-mean", type=float, default=6.0,
                        help="NIS medio en la ventana reciente por encima del "
                             "cual se dispara un reset")

    args = parser.parse_args()

    rclpy.init()

    node = MultiOsaFusionNode(
        min_samples=args.min_samples,
        agl_hint=args.agl_hint,
        range_bias=args.range_bias,
        sigma_h_threshold=args.sigma_threshold,
        gnss_acc_max=args.gnss_acc_max,
        z_prior_sigma=args.z_prior_sigma,
        log_dir=args.log_dir,
        discover_hz=args.discover_hz,
        reset_on_divergence=args.reset_on_divergence,
        divergence_reject_streak=args.divergence_reject_streak,
        divergence_nis_mean=args.divergence_nis_mean,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down …")
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()