#!/usr/bin/env python3
"""
Networked-OSA Fusion Node — Online cooperative multilateration of Wi-Fi RTT
anchors across multiple independent One-Search-Agent (OSA) entities.

Concept
-------
Each OSA (One Search Agent) is an autonomous unit — typically a drone + phone
pair — that can, on its own, run an FTM session against a Wi-Fi anchor and
estimate its location from range observations along its trajectory. The OSA
treats every measurement instant as a *virtual position of the FTM initiator*
relative to the same physical responder (the anchor).

Because every OSA expresses its samples in the same geodetic frame (lat/lon/
alt), the virtual-initiator positions of *different* OSAs are mutually
comparable. This node implements the **network layer** of the OSA framework:
it ingests the per-OSA RTT streams and trajectories online — in principle
over the Internet — and fuses them into a single, joint estimate of the
anchor location. The interconnection of OSAs gives access to much richer
geometry than any single agent could obtain.

This implementation provides:

* Per-OSA range bias states. The IEKF jointly estimates the anchor
  position plus one scalar bias per contributing OSA, instead of a
  single shared bias.
* Robust weighting for both the Gauss-Newton bootstrap and the IEKF
  updates: ``wls``, ``huber``, ``trim``, ``sigma_clip``.
* Explicit configurable minimum number of contributing OSAs
  (``--require-n-osa``) for bootstrap, divergence reset and publish.
* OSA-pose source selector (``--pose-source``): phone GNSS, drone telemetry
    via ``/<ns>/location`` (NavSatFix / Float64MultiArray), or the default
    ``auto`` mode (prefer drone telemetry, fall back to phone GNSS).
* Adaptive Z prior: σ_z of the prior shrinks as the EKF accumulates
  evidence, controlled by ``--z-prior-sigma`` (initial) and
  ``--z-prior-min-sigma`` (asymptotic).
* Scalable vertical handling via ``--z-mode``. ``auto`` infers the initial
    anchor altitude from range geometry and applies no fixed AGL prior;
    ``fixed_agl`` keeps the explicit ``drone_z - agl_hint`` prior for known
    terrain profiles; ``none`` disables synthetic vertical priors.
* Separate horizontal and vertical convergence flags. The published
  estimate carries ``stable_2d`` and ``stable_3d`` so downstream
  consumers can decide whether to use a 2D-only fix.
* Richer logging (per-drone bias, residual RMSE, NIS, rejection counts,
  contributing drone list).

Topics consumed (per OSA namespace ``<ns>``, one namespace per agent):
    /<ns>/ftm_rtt              std_msgs/String          JSON RTT sample
    /<ns>/phone/location       Float64MultiArray        phone GNSS
    /<ns>/location             sensor_msgs/NavSatFix or
                               std_msgs/Float64MultiArray (drone GNSS)

Topics published:
    /fusion/anchor/<ap>/estimate   Float64MultiArray
        [lat, lon, alt, cov_xx, cov_yy, cov_xy, n_acc, sigma_h, sigma_z,
         stable_2d, stable_3d, n_osa]
    /fusion/anchor/<ap>/state      Float64MultiArray
        [lat, lon, alt, sigma_h, sigma_z, n_acc, stable_2d, stable_3d]
    /fusion/status                 std_msgs/String  (JSON summary)
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
import threading
import time
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from std_msgs.msg import String, Float64MultiArray
except ImportError:
    print("ERROR: rclpy not found. Source your ROS 2 workspace first:")
    print("  source /opt/ros/<distro>/setup.bash")
    sys.exit(1)

try:
    from sensor_msgs.msg import NavSatFix  # type: ignore
    HAS_NAVSAT = True
except Exception:
    HAS_NAVSAT = False


# ═══════════════════════════════════════════════════════════════════════════
#  Maths — ENU projection
# ═══════════════════════════════════════════════════════════════════════════

R_EARTH = 6_378_137.0


def deg2rad(d: float) -> float:
    return d * math.pi / 180.0


def rad2deg(r: float) -> float:
    return r * 180.0 / math.pi


def latlon_to_enu(lat: float, lon: float, alt: float,
                  lat0: float, lon0: float, alt0: float) -> tuple[float, float, float]:
    d_lat = deg2rad(lat - lat0)
    d_lon = deg2rad(lon - lon0)
    mean_lat = deg2rad((lat + lat0) / 2.0)
    x = d_lon * math.cos(mean_lat) * R_EARTH
    y = d_lat * R_EARTH
    z = alt - alt0
    return x, y, z


def enu_to_latlon(x: float, y: float, z: float,
                  lat0: float, lon0: float, alt0: float) -> tuple[float, float, float]:
    d_lat = y / R_EARTH
    d_lon = x / (R_EARTH * math.cos(deg2rad(lat0)))
    return lat0 + rad2deg(d_lat), lon0 + rad2deg(d_lon), alt0 + z


# ═══════════════════════════════════════════════════════════════════════════
#  Linear algebra helpers (variable-size)
# ═══════════════════════════════════════════════════════════════════════════

def _quad_form(h: list[float], P: list[list[float]]) -> float:
    return sum(h[i] * P[i][j] * h[j]
               for i in range(len(h)) for j in range(len(h)))


def _mat_vec(M: list[list[float]], v: list[float]) -> list[float]:
    n = len(v)
    return [sum(M[i][j] * v[j] for j in range(n)) for i in range(n)]


def _mat_mul(A: list[list[float]], B: list[list[float]]) -> list[list[float]]:
    n = len(A)
    return [[sum(A[i][k] * B[k][j] for k in range(n)) for j in range(n)]
            for i in range(n)]


def _transpose(M: list[list[float]]) -> list[list[float]]:
    n = len(M)
    return [[M[j][i] for j in range(n)] for i in range(n)]


def _invert_3x3(m: list[list[float]]) -> list[list[float]] | None:
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
#  Robust weighting
# ═══════════════════════════════════════════════════════════════════════════

ROBUST_METHODS = ("wls", "huber", "trim", "sigma_clip")

Z_MODE_AUTO = "auto"
Z_MODE_FIXED_AGL = "fixed_agl"
Z_MODE_NONE = "none"
Z_MODES = (Z_MODE_AUTO, Z_MODE_FIXED_AGL, Z_MODE_NONE)


def _robust_weights(residuals: list[float], sigmas: list[float],
                    method: str, param: float) -> list[float]:
    """Return per-sample weight multipliers (1.0 = full WLS).

    Weights are applied multiplicatively on ``1 / σ²``. Setting a weight to
    0 effectively drops the sample.
    """
    n = len(residuals)
    if n == 0:
        return []
    if method == "wls":
        return [1.0] * n

    # Standardised residuals (robust scale: MAD of residuals).
    abs_r = sorted(abs(r) for r in residuals)
    med = abs_r[len(abs_r) // 2]
    mad = max(med, 1e-3)
    scale = max(mad * 1.4826, min(s for s in sigmas if s > 0) if sigmas else 1.0)

    if method == "huber":
        k = max(param, 0.5)
        out = []
        for r in residuals:
            z = abs(r) / scale
            out.append(1.0 if z <= k else k / max(z, 1e-9))
        return out
    if method == "trim":
        frac = min(max(param, 0.0), 0.9)
        if frac <= 0.0 or n < 3:
            return [1.0] * n
        n_drop = max(1, int(round(frac * n)))
        if n_drop >= n:
            return [1.0] * n
        ranks = sorted(range(n), key=lambda i: abs(residuals[i]))
        keep = set(ranks[: n - n_drop])
        return [1.0 if i in keep else 0.0 for i in range(n)]
    if method == "sigma_clip":
        k = max(param, 1.0)
        out = []
        for r in residuals:
            z = abs(r) / scale
            out.append(1.0 if z <= k else 0.0)
        return out
    return [1.0] * n


# ═══════════════════════════════════════════════════════════════════════════
#  Data structures
# ═══════════════════════════════════════════════════════════════════════════

@dataclass
class ENUSample:
    x: float
    y: float
    z: float
    r: float
    sigma: float
    drone_ns: str
    t_wall: float


_POSE_BUF_MAX = 50


@dataclass
class DroneState:
    """Latest known position of a drone, with interpolation buffer."""
    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0
    acc_m: float = -1.0
    t_ms: float = 0.0
    updated: bool = False
    source: str = ""           # which topic populated the pose
    _pose_buf: list = field(default_factory=list)

    def push_pose(self, t_ms: float, lat: float, lon: float, alt: float,
                  source: str) -> None:
        self._pose_buf.append((t_ms, lat, lon, alt))
        if len(self._pose_buf) > _POSE_BUF_MAX:
            self._pose_buf = self._pose_buf[-_POSE_BUF_MAX:]
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.t_ms = t_ms
        self.updated = True
        self.source = source

    def interp_pose(self, t_ms: float) -> tuple[float, float, float] | None:
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
        a = (t_ms - t0) / dt
        return (lat0 + a * (lat1 - lat0),
                lon0 + a * (lon1 - lon0),
                alt0 + a * (alt1 - alt0))


# ═══════════════════════════════════════════════════════════════════════════
#  Gauss-Newton 3D with optional robust weighting
# ═══════════════════════════════════════════════════════════════════════════

def _geometric_spread(samples: list[ENUSample]) -> float:
    if not samples:
        return 0.0
    xs = [s.x for s in samples]; ys = [s.y for s in samples]
    return math.hypot(max(xs) - min(xs), max(ys) - min(ys))


def _vertical_spread(samples: list[ENUSample]) -> float:
    if not samples:
        return 0.0
    zs = [s.z for s in samples]
    return max(zs) - min(zs)


def _z_candidate_score(samples: list[ENUSample], ax: float, ay: float, az: float) -> float:
    residuals = []
    for s in samples:
        dx = ax - s.x; dy = ay - s.y; dz = az - s.z
        residuals.append(abs(math.sqrt(dx * dx + dy * dy + dz * dz + 1e-12) - s.r))
    if not residuals:
        return float("inf")
    vals = sorted(residuals)
    median = vals[len(vals) // 2]
    p80 = vals[int(round(0.8 * (len(vals) - 1)))]
    return median + 0.25 * p80


def _auto_initial_z(samples: list[ENUSample], ax: float, ay: float,
                    agl_hint: float) -> float:
    """Infer an initial anchor Z from range geometry, without a fixed AGL.

    For a provisional horizontal position, each range observation implies two
    vertical candidates, one below and one above the OSA. We score the lower
    and upper candidate clusters by their range residuals and keep the best.
    The fixed-AGL value is only a last-resort fallback when the range geometry
    cannot produce any real vertical candidate.
    """
    candidates: list[float] = []
    for sign in (-1.0, 1.0):
        vals = []
        for s in samples:
            dx = ax - s.x; dy = ay - s.y
            rem = s.r * s.r - dx * dx - dy * dy
            if rem >= 0.0:
                vals.append(s.z + sign * math.sqrt(rem))
        if vals:
            vals.sort()
            candidates.append(vals[len(vals) // 2])
    if not candidates:
        vals = sorted(s.z - agl_hint for s in samples)
        return vals[len(vals) // 2]
    return min(candidates, key=lambda z: _z_candidate_score(samples, ax, ay, z))


def gauss_newton_3d(
    samples: list[ENUSample],
    agl_hint: float = 0.0,
    z_mode: str = Z_MODE_AUTO,
    max_iters: int = 15,
    tol: float = 1e-4,
    min_spread: float = 15.0,
    robust_method: str = "wls",
    robust_param: float = 0.0,
) -> tuple[float, float, float, float] | None:
    n = len(samples)
    if n < 4:
        return None
    if _geometric_spread(samples) < min_spread:
        return None

    # Linear 2D init.
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
    if z_mode == Z_MODE_FIXED_AGL:
        z_candidates = sorted(s.z - agl_hint for s in samples)
        az = z_candidates[len(z_candidates) // 2]
    else:
        az = _auto_initial_z(samples, ax, ay, agl_hint)

    for _ in range(max_iters):
        # Residuals at current estimate.
        residuals: list[float] = []
        for s in samples:
            dx = ax - s.x; dy = ay - s.y; dz = az - s.z
            di = math.sqrt(dx * dx + dy * dy + dz * dz + 1e-12)
            residuals.append(di - s.r)
        sigmas = [s.sigma for s in samples]
        rw = _robust_weights(residuals, sigmas, robust_method, robust_param)

        j = [[0.0] * 3 for _ in range(3)]
        g = [0.0, 0.0, 0.0]
        for k, s in enumerate(samples):
            dx = ax - s.x; dy = ay - s.y; dz = az - s.z
            di = math.sqrt(dx * dx + dy * dy + dz * dz + 1e-12)
            jac = [dx / di, dy / di, dz / di]
            w = rw[k] / (s.sigma * s.sigma)
            if w <= 0.0:
                continue
            for a in range(3):
                for b in range(a, 3):
                    j[a][b] += w * jac[a] * jac[b]
                g[a] += w * jac[a] * residuals[k]
        j[1][0] = j[0][1]; j[2][0] = j[0][2]; j[2][1] = j[1][2]

        inv = _invert_3x3(j)
        if inv is None:
            return None
        step = [-sum(inv[a][b] * g[b] for b in range(3)) for a in range(3)]
        ax += step[0]; ay += step[1]; az += step[2]
        if math.sqrt(sum(s * s for s in step)) < tol:
            break

    dof = max(1, n - 3)
    chi2 = 0.0
    for s in samples:
        dx = ax - s.x; dy = ay - s.y; dz = az - s.z
        di = math.sqrt(dx * dx + dy * dy + dz * dz + 1e-12)
        chi2 += ((di - s.r) / s.sigma) ** 2

    cx = sum(s.x for s in samples) / n
    cy = sum(s.y for s in samples) / n
    max_range = max(s.r for s in samples) * 2.0
    if math.hypot(ax - cx, ay - cy) > max_range:
        return None
    return ax, ay, az, chi2 / dof


# ═══════════════════════════════════════════════════════════════════════════
#  IEKF with per-drone bias
# ═══════════════════════════════════════════════════════════════════════════

class IEKFMultiBias:
    """Iterated EKF with state ``[x, y, z, b_drone_1, b_drone_2, ...]``.

    Range observations are modelled as ``z_meas = ||p_anchor - p_drone|| + b_drone``.
    The drone-specific bias index is resolved via ``drone_index`` at update time.
    """

    def __init__(self, x0: list[float], p0: list[list[float]],
                 q_xyz: tuple[float, float, float] = (0.01, 0.01, 0.04),
                 q_bias: float = 1e-4):
        self.x = list(x0)
        self.P = [row[:] for row in p0]
        self.n_accepted = 0
        self.n_rejected = 0
        self.last_nis = math.nan
        self.last_innov = math.nan
        self.q_xyz = q_xyz
        self.q_bias = q_bias

    @property
    def n_states(self) -> int:
        return len(self.x)

    @property
    def n_biases(self) -> int:
        return self.n_states - 3

    def bias_for(self, drone_idx: int) -> float:
        if drone_idx < 0 or drone_idx >= self.n_biases:
            return 0.0
        return self.x[3 + drone_idx]

    @property
    def sigma_h(self) -> float:
        return math.sqrt(max(self.P[0][0] + self.P[1][1], 0.0))

    @property
    def sigma_z(self) -> float:
        return math.sqrt(max(self.P[2][2], 0.0))

    def add_bias_state(self, sigma_init: float = 2.0) -> int:
        """Grow the state with a new per-drone bias term. Returns its index."""
        n_old = self.n_states
        new_n = n_old + 1
        # extend x with 0 bias
        self.x.append(0.0)
        # extend P
        new_P = [row + [0.0] for row in self.P]
        new_P.append([0.0] * new_n)
        new_P[-1][-1] = sigma_init * sigma_init
        self.P = new_P
        return new_n - 1 - 3   # index of newly added bias slot

    def predict(self) -> None:
        self.P[0][0] += self.q_xyz[0]
        self.P[1][1] += self.q_xyz[1]
        self.P[2][2] += self.q_xyz[2]
        for k in range(self.n_biases):
            self.P[3 + k][3 + k] += self.q_bias

    def update_range(
        self,
        z_meas: float,
        sigma: float,
        ue: float, un: float, uu: float,
        drone_idx: int,
        iters: int = 3,
        gate_gamma: float = 9.0,
        robust_method: str = "wls",
        robust_param: float = 0.0,
    ) -> tuple[bool, float, float]:
        """IEKF update. Returns ``(accepted, innov, robust_weight)``."""
        if not math.isfinite(z_meas) or not math.isfinite(sigma) or sigma <= 0:
            return False, math.nan, 0.0
        R = sigma * sigma
        n = self.n_states
        b = self.bias_for(drone_idx)
        # Pre-gate
        dx = self.x[0] - ue; dy = self.x[1] - un; dz = self.x[2] - uu
        h = max(math.sqrt(dx * dx + dy * dy + dz * dz), 1e-6)
        innov = z_meas - (h + b)
        H = [0.0] * n
        H[0] = dx / h; H[1] = dy / h; H[2] = dz / h
        if 0 <= drone_idx < self.n_biases:
            H[3 + drone_idx] = 1.0

        # Robust weight for the single innovation: use Huber-style attenuation
        # of σ (cheap online surrogate for IRLS).
        rw = _robust_weights([innov], [sigma], robust_method, robust_param)[0]
        if rw <= 0.0:
            self.n_rejected += 1
            self.last_innov = innov
            self.last_nis = math.inf
            return False, innov, 0.0
        R_eff = R / max(rw, 1e-6)
        S = _quad_form(H, self.P) + R_eff
        if not math.isfinite(S) or S <= 0:
            return False, innov, rw
        nis = (innov * innov) / S
        self.last_innov = innov
        self.last_nis = nis
        if nis > gate_gamma:
            self.n_rejected += 1
            return False, innov, rw

        for _ in range(max(1, iters)):
            b = self.bias_for(drone_idx)
            dx = self.x[0] - ue; dy = self.x[1] - un; dz = self.x[2] - uu
            h = max(math.sqrt(dx * dx + dy * dy + dz * dz), 1e-6)
            r = z_meas - (h + b)
            H = [0.0] * n
            H[0] = dx / h; H[1] = dy / h; H[2] = dz / h
            if 0 <= drone_idx < self.n_biases:
                H[3 + drone_idx] = 1.0
            S = _quad_form(H, self.P) + R_eff
            if not math.isfinite(S) or S <= 0:
                return False, innov, rw
            PHt = _mat_vec(self.P, H)
            K = [ph / S for ph in PHt]
            for i in range(n):
                self.x[i] += K[i] * r
            # Joseph
            IKH = [[(1.0 if i == j else 0.0) - K[i] * H[j]
                    for j in range(n)] for i in range(n)]
            P_new = _mat_mul(IKH, _mat_mul(self.P, _transpose(IKH)))
            for i in range(n):
                for j in range(n):
                    P_new[i][j] += K[i] * R_eff * K[j]
            self.P = P_new
        self.n_accepted += 1
        return True, innov, rw

    def update_z_prior(self, z_prior: float, sigma_z: float) -> None:
        if not math.isfinite(z_prior) or sigma_z <= 0:
            return
        R = sigma_z * sigma_z
        n = self.n_states
        H = [0.0] * n
        H[2] = 1.0
        S = _quad_form(H, self.P) + R
        if not math.isfinite(S) or S <= 0:
            return
        innov = z_prior - self.x[2]
        PHt = _mat_vec(self.P, H)
        K = [ph / S for ph in PHt]
        for i in range(n):
            self.x[i] += K[i] * innov
        IKH = [[(1.0 if i == j else 0.0) - K[i] * H[j]
                for j in range(n)] for i in range(n)]
        P_new = _mat_mul(IKH, _mat_mul(self.P, _transpose(IKH)))
        for i in range(n):
            for j in range(n):
                P_new[i][j] += K[i] * R * K[j]
        self.P = P_new


# ═══════════════════════════════════════════════════════════════════════════
#  MAD-based RTT gate
# ═══════════════════════════════════════════════════════════════════════════

class RttGate:
    def __init__(self, window_ms: int = 1500, max_n: int = 30,
                 min_n: int = 8, k: float = 3.5):
        self._window_ms = window_ms
        self._max_n = max_n
        self._min_n = min_n
        self._k = k
        self._sigma_floor = 0.12
        self._buf: list[tuple[float, float]] = []

    def push_and_gate(self, t_ms: float, d: float, sigma: float) -> tuple[bool, float]:
        cutoff = t_ms - self._window_ms
        self._buf = [(t, v) for t, v in self._buf if t >= cutoff]
        self._buf.append((t_ms, d))
        if len(self._buf) > self._max_n:
            self._buf = self._buf[-self._max_n:]
        if len(self._buf) < self._min_n:
            return True, max(sigma, self._sigma_floor)
        vals = sorted(v for _, v in self._buf)
        median = vals[len(vals) // 2]
        abs_devs = sorted(abs(v - median) for v in vals)
        mad = abs_devs[len(abs_devs) // 2] * 1.4826
        robust_sigma = max(mad, self._sigma_floor)
        if abs(d - median) > self._k * robust_sigma:
            return False, robust_sigma
        return True, robust_sigma


# ═══════════════════════════════════════════════════════════════════════════
#  Per-AP Anchor Track
# ═══════════════════════════════════════════════════════════════════════════

class AnchorTrack:
    def __init__(self, bssid: str):
        self.bssid = bssid
        self.samples: list[ENUSample] = []
        self.ekf: IEKFMultiBias | None = None
        self.gate = RttGate()
        self.lock = threading.Lock()
        self.contributing_drones: set[str] = set()
        self.drone_to_bias_idx: dict[str, int] = {}

        # 2D / 3D convergence tracking
        self.stable_streak_2d = 0
        self.stable_streak_3d = 0
        self.converging_streak = 0
        self.prev_sigma_h = float("inf")
        self.last_pub_xyz: tuple[float, float, float] | None = None
        self.n_published_2d = 0
        self.n_published_3d = 0

        # Divergence detection
        self._nis_window: list[float] = []
        self._nis_window_max = 20
        self._consecutive_bad = 0
        self.n_resets = 0

        # Z ground prior buffer
        self._z_ground: list[float] = []

        # Residuals window (for RMSE in logs)
        self._resid_window: list[float] = []
        self._resid_window_max = 50

    # bias index allocation -----------------------------------------------
    def bias_index_for(self, drone_ns: str) -> int:
        if drone_ns not in self.drone_to_bias_idx:
            if self.ekf is None:
                self.drone_to_bias_idx[drone_ns] = len(self.drone_to_bias_idx)
            else:
                idx = self.ekf.add_bias_state(sigma_init=2.0)
                self.drone_to_bias_idx[drone_ns] = idx
        return self.drone_to_bias_idx[drone_ns]

    def reset_ekf(self, reason: str = "") -> None:
        self.ekf = None
        self.samples.clear()
        self.stable_streak_2d = 0
        self.stable_streak_3d = 0
        self.converging_streak = 0
        self.prev_sigma_h = float("inf")
        self.last_pub_xyz = None
        self._nis_window.clear()
        self._consecutive_bad = 0
        self.drone_to_bias_idx.clear()
        self._resid_window.clear()
        self.n_resets += 1

    def z_ground_median(self) -> float | None:
        if not self._z_ground:
            return None
        s = sorted(self._z_ground)
        return s[len(s) // 2]

    def push_z_ground(self, z: float) -> None:
        self._z_ground.append(z)
        if len(self._z_ground) > 25:
            self._z_ground = self._z_ground[-25:]

    def push_residual(self, r: float) -> None:
        self._resid_window.append(r)
        if len(self._resid_window) > self._resid_window_max:
            self._resid_window = self._resid_window[-self._resid_window_max:]

    @property
    def residual_rmse(self) -> float:
        if not self._resid_window:
            return math.nan
        return math.sqrt(sum(r * r for r in self._resid_window) / len(self._resid_window))


# ═══════════════════════════════════════════════════════════════════════════
#  ROS 2 Fusion Node
# ═══════════════════════════════════════════════════════════════════════════

POSE_SOURCE_PHONE = "phone"
POSE_SOURCE_DRONE = "drone"
POSE_SOURCE_AUTO = "auto"
POSE_SOURCES = (POSE_SOURCE_PHONE, POSE_SOURCE_DRONE, POSE_SOURCE_AUTO)


class MultiOsaFusionNode(Node):

    def __init__(
        self,
        min_samples: int = 6,
        agl_hint: float = 0.0,
        range_bias: float = 0.0,
        z_mode: str = Z_MODE_AUTO,
        sigma_h_threshold: float = 2.5,
        sigma_z_threshold: float = 20.0,
        gnss_acc_max: float = 5.0,
        z_prior_sigma: float = 10.0,
        z_prior_min_sigma: float = 5.0,
        log_dir: str | None = None,
        discover_hz: float = 0.2,
        reset_on_divergence: bool = True,
        divergence_reject_streak: int = 8,
        divergence_nis_mean: float = 6.0,
        robust_method: str = "sigma_clip",
        robust_param: float = 2.5,
        require_n_osa: int = 2,
        require_n_osa_publish: int = 2,
        min_spread_m: float = 15.0,
        min_vertical_spread_m: float = 0.0,
        pose_source: str = POSE_SOURCE_AUTO,
        publish_2d_only: bool = True,
    ):
        super().__init__("networked_osa_fusion")
        self.get_logger().info("Networked-OSA Fusion Node starting …")

        if robust_method not in ROBUST_METHODS:
            raise ValueError(f"robust_method must be one of {ROBUST_METHODS}")
        if pose_source not in POSE_SOURCES:
            raise ValueError(f"pose_source must be one of {POSE_SOURCES}")
        if z_mode not in Z_MODES:
            raise ValueError(f"z_mode must be one of {Z_MODES}")

        self._min_samples = max(4, min_samples)
        self._agl_hint = agl_hint
        self._range_bias = range_bias
        self._z_mode = z_mode
        self._sigma_h_threshold = sigma_h_threshold
        self._sigma_z_threshold = sigma_z_threshold
        self._gnss_acc_max = gnss_acc_max
        self._z_prior_sigma0 = z_prior_sigma
        self._z_prior_min_sigma = max(0.5, z_prior_min_sigma)
        self._reset_on_divergence = reset_on_divergence
        self._divergence_reject_streak = max(3, divergence_reject_streak)
        self._divergence_nis_mean = max(1.0, divergence_nis_mean)
        self._robust_method = robust_method
        self._robust_param = robust_param
        self._require_n_osa = max(1, require_n_osa)
        self._require_n_osa_publish = max(self._require_n_osa,
                                             require_n_osa_publish)
        self._min_spread = min_spread_m
        self._min_vertical_spread = min_vertical_spread_m
        self._pose_source = pose_source
        self._publish_2d_only = publish_2d_only

        self._origin: tuple[float, float, float] | None = None
        self._origin_lock = threading.Lock()

        # Per-drone phone pose and drone (telemetry) pose, separate buffers
        self._phone_poses: dict[str, DroneState] = defaultdict(DroneState)
        self._drone_poses: dict[str, DroneState] = defaultdict(DroneState)
        self._poses_lock = threading.Lock()

        self._tracks: dict[str, AnchorTrack] = {}
        self._tracks_lock = threading.Lock()

        self._subscribed_ns_rtt: set[str] = set()
        self._subscribed_phone_pose: set[str] = set()
        self._subscribed_drone_pose: set[str] = set()
        self._subscribed_estimate_topics: set[str] = set()
        self._subs: list = []

        self._anchor_pubs: dict[str, object] = {}
        self._state_pubs: dict[str, object] = {}
        self._status_pub = self.create_publisher(String, "/fusion/status", 10)

        self._log_dir: Path | None = None
        self._log_files: dict[str, object] = {}
        if log_dir:
            self._log_dir = Path(log_dir)
            self._log_dir.mkdir(parents=True, exist_ok=True)

        self._qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        self._discover_timer = self.create_timer(
            1.0 / discover_hz, self._discover_namespaces
        )
        self._status_timer = self.create_timer(2.0, self._publish_status)

        self.get_logger().info(
            "Config: "
            f"min_samples={self._min_samples} "
            f"z_mode={self._z_mode} "
            f"agl_hint={self._agl_hint:.1f} "
            f"range_bias={self._range_bias:.2f} "
            f"sigma_h_thr={self._sigma_h_threshold:.2f} "
            f"sigma_z_thr={self._sigma_z_threshold:.2f} "
            f"robust={self._robust_method}({self._robust_param}) "
            f"need_osa={self._require_n_osa}/{self._require_n_osa_publish} "
            f"min_spread={self._min_spread:.1f} "
            f"pose_source={self._pose_source}"
        )

    # ─── Discovery ───────────────────────────────────────────────────────

    def _discover_namespaces(self) -> None:
        topic_list = self.get_topic_names_and_types()
        rtt_re = re.compile(r"^/([^/]+)/ftm_rtt$")
        phone_re = re.compile(r"^/([^/]+)/phone/location$")
        drone_re = re.compile(r"^/([^/]+)/location$")
        est_re = re.compile(r"^/([^/]+)/ftm/anchor/(ap_[0-9a-f]+)/estimate$")
        for topic_name, types in topic_list:
            m = rtt_re.match(topic_name)
            if m and m.group(1) != "fusion" and m.group(1) not in self._subscribed_ns_rtt:
                self._subscribe_rtt(m.group(1))
            m = phone_re.match(topic_name)
            if m and m.group(1) != "fusion" and m.group(1) not in self._subscribed_phone_pose:
                self._subscribe_phone_pose(m.group(1))
            m = drone_re.match(topic_name)
            if m and m.group(1) not in {"fusion"} and m.group(1) not in self._subscribed_drone_pose:
                self._subscribe_drone_pose(m.group(1), types)
            em = est_re.match(topic_name)
            if em and em.group(1) != "fusion" and topic_name not in self._subscribed_estimate_topics:
                self._subscribed_estimate_topics.add(topic_name)
                ap_id = em.group(2)
                ns_est = em.group(1)
                sub = self.create_subscription(
                    Float64MultiArray, topic_name,
                    lambda msg, _ns=ns_est, _ap=ap_id: self._on_phone_estimate(msg, _ns, _ap),
                    self._qos,
                )
                self._subs.append(sub)

    def _subscribe_rtt(self, ns: str) -> None:
        self._subscribed_ns_rtt.add(ns)
        self.get_logger().info(f"Subscribing RTT: /{ns}/ftm_rtt")
        sub = self.create_subscription(
            String, f"/{ns}/ftm_rtt",
            lambda msg, _ns=ns: self._on_rtt(msg, _ns),
            self._qos,
        )
        self._subs.append(sub)

    def _subscribe_phone_pose(self, ns: str) -> None:
        self._subscribed_phone_pose.add(ns)
        self.get_logger().info(f"Subscribing phone pose: /{ns}/phone/location")
        sub = self.create_subscription(
            Float64MultiArray, f"/{ns}/phone/location",
            lambda msg, _ns=ns: self._on_phone_location(msg, _ns),
            self._qos,
        )
        self._subs.append(sub)

    def _subscribe_drone_pose(self, ns: str, types: list[str]) -> None:
        if self._pose_source == POSE_SOURCE_PHONE:
            return
        self._subscribed_drone_pose.add(ns)
        is_navsat = HAS_NAVSAT and any("NavSatFix" in t for t in types)
        topic = f"/{ns}/location"
        self.get_logger().info(
            f"Subscribing drone pose: {topic} ({'NavSatFix' if is_navsat else 'Float64MultiArray'})"
        )
        if is_navsat:
            sub = self.create_subscription(
                NavSatFix, topic,
                lambda msg, _ns=ns: self._on_drone_navsat(msg, _ns),
                self._qos,
            )
        else:
            sub = self.create_subscription(
                Float64MultiArray, topic,
                lambda msg, _ns=ns: self._on_drone_loc_array(msg, _ns),
                self._qos,
            )
        self._subs.append(sub)

    # ─── Pose callbacks ──────────────────────────────────────────────────

    def _maybe_set_origin(self, lat: float, lon: float, alt: float,
                          acc: float, source_ns: str) -> None:
        with self._origin_lock:
            if self._origin is not None:
                return
            if acc <= 0 or acc > 2.0:
                self.get_logger().warning(
                    f"Waiting for RTK-quality fix for ENU origin "
                    f"(acc={acc:.1f}m, need 0<acc<=2.0m, from {source_ns})",
                    throttle_duration_sec=5.0,
                )
                return
            self._origin = (lat, lon, alt)
            self.get_logger().info(
                f"ENU origin set: lat={lat:.7f} lon={lon:.7f} alt={alt:.1f} "
                f"acc={acc:.2f}m (from {source_ns})"
            )

    def _on_phone_location(self, msg: Float64MultiArray, ns: str) -> None:
        d = list(msg.data)
        if len(d) < 5:
            return
        lat, lon, alt, acc, t_ms = d[0], d[1], d[2], d[3], d[4]
        if acc > 0 and acc > self._gnss_acc_max:
            return
        self._maybe_set_origin(lat, lon, alt, acc, ns)
        with self._poses_lock:
            ds = self._phone_poses[ns]
            ds.acc_m = acc
            ds.push_pose(t_ms, lat, lon, alt, source="phone")

    def _on_drone_navsat(self, msg, ns: str) -> None:
        t_ms = float(time.time() * 1000)
        # Drone GNSS is typically RTK on the M350; set acc small.
        with self._poses_lock:
            ds = self._drone_poses[ns]
            ds.acc_m = 0.5
            ds.push_pose(t_ms, float(msg.latitude), float(msg.longitude),
                         float(msg.altitude), source="drone")
        self._maybe_set_origin(float(msg.latitude), float(msg.longitude),
                               float(msg.altitude), 0.5, ns)

    def _on_drone_loc_array(self, msg: Float64MultiArray, ns: str) -> None:
        d = list(msg.data)
        if len(d) < 3:
            return
        lat, lon, alt = d[0], d[1], d[2]
        acc = d[3] if len(d) >= 4 else 0.5
        t_ms = d[4] if len(d) >= 5 else float(time.time() * 1000)
        with self._poses_lock:
            ds = self._drone_poses[ns]
            ds.acc_m = acc
            ds.push_pose(t_ms, lat, lon, alt, source="drone")
        self._maybe_set_origin(lat, lon, alt, acc, ns)

    def _select_pose(self, ns: str, t_ms: float) -> tuple[float, float, float, float, str] | None:
        """Return (lat, lon, alt, acc, source) or None."""
        with self._poses_lock:
            drone = self._drone_poses.get(ns) if self._pose_source != POSE_SOURCE_PHONE else None
            phone = self._phone_poses.get(ns) if self._pose_source != POSE_SOURCE_DRONE else None
            if self._pose_source == POSE_SOURCE_DRONE:
                cand = [(drone, "drone")] if drone and drone.updated else []
            elif self._pose_source == POSE_SOURCE_PHONE:
                cand = [(phone, "phone")] if phone and phone.updated else []
            else:
                cand = []
                if drone and drone.updated:
                    cand.append((drone, "drone"))
                if phone and phone.updated:
                    cand.append((phone, "phone"))
            for ds, label in cand:
                pose = ds.interp_pose(t_ms)
                if pose is None:
                    continue
                lat, lon, alt = pose
                return lat, lon, alt, ds.acc_m, label
            return None

    # ─── RTT callback ────────────────────────────────────────────────────

    def _on_rtt(self, msg: String, ns: str) -> None:
        try:
            j = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        bssid = j.get("bssid", "")
        if not bssid:
            return
        accepted = j.get("accepted", True)
        if accepted is False or accepted == "false":
            return
        try:
            dist_m = float(j.get("distance_m", -1.0))
            sigma_m = float(j.get("sigma_used_m", j.get("std_m", 1.0)))
        except Exception:
            return
        if dist_m <= 0 or sigma_m <= 0:
            return
        t_wall = float(j.get("t_ms", time.time() * 1000))
        t_mono = float(j.get("t_mono_ms", t_wall))

        pose = self._select_pose(ns, t_wall)
        if pose is None:
            return
        drone_lat, drone_lon, drone_alt, acc_m, pose_label = pose

        with self._origin_lock:
            origin = self._origin
        if origin is None:
            return
        ue, un, uu = latlon_to_enu(drone_lat, drone_lon, drone_alt, *origin)

        self._process_rtt(
            bssid=bssid, dist_m=dist_m, sigma_m=sigma_m,
            ue=ue, un=un, uu=uu,
            drone_ns=ns, t_wall=t_wall / 1000.0, t_mono=t_mono,
            gps_acc_m=acc_m, pose_label=pose_label,
        )

    # ─── Phone-side estimate cross-init ──────────────────────────────────

    def _on_phone_estimate(self, msg: Float64MultiArray, ns: str, ap_id: str) -> None:
        data = list(msg.data)
        if len(data) < 7:
            return
        lat, lon, alt = data[0], data[1], data[2]
        cov_xx, cov_yy = data[3], data[4]
        n_samples = data[6]
        if n_samples < 10:
            return
        hex_part = ap_id.replace("ap_", "")
        bssid = ":".join(hex_part[i:i+2] for i in range(0, 12, 2)) if len(hex_part) == 12 else hex_part
        with self._tracks_lock:
            track = self._tracks.get(bssid)
            if track is None:
                track = AnchorTrack(bssid)
                self._tracks[bssid] = track
        with track.lock:
            if track.ekf is not None:
                return
            with self._origin_lock:
                origin = self._origin
            if origin is None:
                return
            ue, un, uu = latlon_to_enu(lat, lon, alt, *origin)
            p0_h = max(cov_xx, cov_yy, 4.0)
            # Allocate one bias slot for the bootstrapping drone.
            track.drone_to_bias_idx[ns] = 0
            n_bias = 1
            n = 3 + n_bias
            x0 = [ue, un, uu, 0.0]
            p0 = [[0.0] * n for _ in range(n)]
            p0[0][0] = p0_h; p0[1][1] = p0_h
            p0[2][2] = 100.0
            p0[3][3] = 4.0
            track.ekf = IEKFMultiBias(x0=x0, p0=p0)
            track.contributing_drones.add(ns)
            track.samples.clear()
            self.get_logger().info(
                f"[{bssid}] EKF cross-initialised from {ns} phone estimate "
                f"at ENU=({ue:.1f},{un:.1f},{uu:.1f}) n_phone={n_samples:.0f}"
            )

    # ─── Core RTT pipeline ───────────────────────────────────────────────

    def _process_rtt(
        self, bssid: str, dist_m: float, sigma_m: float,
        ue: float, un: float, uu: float,
        drone_ns: str, t_wall: float, t_mono: float,
        gps_acc_m: float, pose_label: str,
    ) -> None:
        with self._tracks_lock:
            track = self._tracks.get(bssid)
            if track is None:
                track = AnchorTrack(bssid)
                self._tracks[bssid] = track
        with track.lock:
            self._process_rtt_locked(
                track, bssid, dist_m, sigma_m,
                ue, un, uu, drone_ns, t_wall, t_mono, gps_acc_m, pose_label,
            )

    def _process_rtt_locked(
        self, track: AnchorTrack, bssid: str,
        dist_m: float, sigma_m: float,
        ue: float, un: float, uu: float,
        drone_ns: str, t_wall: float, t_mono: float,
        gps_acc_m: float, pose_label: str,
    ) -> None:
        dist_m = max(0.0, dist_m - self._range_bias)
        accepted, robust_sigma = track.gate.push_and_gate(t_mono, dist_m, sigma_m)
        if not accepted:
            return
        eff_sigma = max(robust_sigma, 0.12)
        rtk_quality = 0 < gps_acc_m <= 2.0
        if gps_acc_m > 0:
            eff_sigma = math.sqrt(eff_sigma * eff_sigma + gps_acc_m * gps_acc_m)

        if self._z_mode == Z_MODE_FIXED_AGL:
            track.push_z_ground(uu - self._agl_hint)
        sample = ENUSample(x=ue, y=un, z=uu, r=dist_m, sigma=eff_sigma,
                           drone_ns=drone_ns, t_wall=t_wall)

        if track.ekf is None:
            if not rtk_quality:
                return
            track.samples.append(sample)
            unique_drones = {s.drone_ns for s in track.samples}
            if len(track.samples) < self._min_samples:
                return
            if len(unique_drones) < self._require_n_osa:
                return
            if _vertical_spread(track.samples) < self._min_vertical_spread:
                return
            init = gauss_newton_3d(
                track.samples, agl_hint=self._agl_hint, z_mode=self._z_mode,
                max_iters=15,
                min_spread=self._min_spread,
                robust_method=self._robust_method, robust_param=self._robust_param,
            )
            if init is None:
                return
            ax, ay, az, _ = init
            z_prior = track.z_ground_median()
            if self._z_mode == Z_MODE_FIXED_AGL and z_prior is not None and abs(az - z_prior) > 20.0:
                az = z_prior

            # allocate bias state per unique drone in bootstrap samples
            unique = sorted(unique_drones)
            track.drone_to_bias_idx = {ns: i for i, ns in enumerate(unique)}
            n_bias = len(unique)
            n_state = 3 + n_bias
            x0 = [ax, ay, az] + [0.0] * n_bias
            p0 = [[0.0] * n_state for _ in range(n_state)]
            p0[0][0] = 25.0; p0[1][1] = 25.0; p0[2][2] = 100.0
            for k in range(n_bias):
                p0[3 + k][3 + k] = 4.0
            track.ekf = IEKFMultiBias(x0=x0, p0=p0)
            track.contributing_drones = set(unique)
            n_boot = len(track.samples)
            track.samples.clear()
            self.get_logger().info(
                f"[{bssid}] EKF init: {n_boot} obs, drones={sorted(track.contributing_drones)} "
                f"ENU=({ax:.1f},{ay:.1f},{az:.1f}) robust={self._robust_method}"
            )
            return

        ekf = track.ekf
        ekf.predict()
        bias_idx = track.bias_index_for(drone_ns)
        ok, innov, rw = ekf.update_range(
            z_meas=dist_m, sigma=eff_sigma,
            ue=ue, un=un, uu=uu, drone_idx=bias_idx,
            iters=3, gate_gamma=9.0,
            robust_method=self._robust_method, robust_param=self._robust_param,
        )
        if math.isfinite(innov):
            track.push_residual(innov)
        if math.isfinite(ekf.last_nis):
            track._nis_window.append(ekf.last_nis)
            if len(track._nis_window) > track._nis_window_max:
                track._nis_window = track._nis_window[-track._nis_window_max:]
        if not ok:
            track._consecutive_bad += 1
            self._publish_state(track, bssid, drone_ns, stable_2d=False, stable_3d=False)
            self._maybe_reset(track, bssid)
            return
        track._consecutive_bad = 0
        track.contributing_drones.add(drone_ns)

        # Fixed-AGL mode applies an external vertical prior. Auto/none modes let
        # the ranges determine Z, so they do not double-count a synthetic prior.
        z_prior = track.z_ground_median()
        if self._z_mode == Z_MODE_FIXED_AGL and z_prior is not None:
            decay = 1.0 / math.sqrt(1.0 + max(ekf.n_accepted - 1, 0) / 20.0)
            sigma_z = max(self._z_prior_min_sigma, self._z_prior_sigma0 * decay)
            ekf.update_z_prior(z_prior, sigma_z=sigma_z)

        sigma_h = ekf.sigma_h
        sigma_z_val = ekf.sigma_z

        is_converging = sigma_h < track.prev_sigma_h * 1.02
        track.converging_streak = track.converging_streak + 1 if is_converging else 0
        track.prev_sigma_h = sigma_h
        min_acc = 10 if track.converging_streak >= 5 else 15
        need_streak = 2 if track.converging_streak >= 8 else 3

        lp = track.last_pub_xyz
        jump_xy = math.hypot(ekf.x[0] - lp[0], ekf.x[1] - lp[1]) if lp is not None else 0.0
        max_jump = max(1.0, sigma_h * 2.0)

        meet_drones = len(track.contributing_drones) >= self._require_n_osa_publish
        stable_2d = (
            ekf.n_accepted >= min_acc
            and math.isfinite(sigma_h) and sigma_h < self._sigma_h_threshold
            and jump_xy < max_jump
            and meet_drones
        )
        stable_3d = (
            stable_2d
            and math.isfinite(sigma_z_val) and sigma_z_val < self._sigma_z_threshold
        )

        self._publish_state(track, bssid, drone_ns, stable_2d=stable_2d, stable_3d=stable_3d)
        self._log_entry("fusion_state.jsonl", self._state_log_payload(
            track, bssid, drone_ns, sigma_h, sigma_z_val, stable_2d, stable_3d, pose_label,
        ))
        self._maybe_reset(track, bssid)

        if stable_3d:
            track.stable_streak_3d += 1
        else:
            track.stable_streak_3d = 0
        if stable_2d:
            track.stable_streak_2d += 1
        else:
            track.stable_streak_2d = 0

        publish_now_3d = track.stable_streak_3d >= need_streak
        publish_now_2d = track.stable_streak_2d >= need_streak and self._publish_2d_only
        if not (publish_now_3d or publish_now_2d):
            return

        with self._origin_lock:
            origin = self._origin
        if origin is None:
            return
        xe, yn, zu = ekf.x[0], ekf.x[1], ekf.x[2]
        track.last_pub_xyz = (xe, yn, zu)
        if publish_now_3d:
            track.n_published_3d += 1
        if publish_now_2d and not publish_now_3d:
            track.n_published_2d += 1

        lat_e, lon_e, alt_e = enu_to_latlon(xe, yn, zu, *origin)
        cov_xx = ekf.P[0][0]; cov_yy = ekf.P[1][1]; cov_xy = ekf.P[0][1]
        n_acc = float(ekf.n_accepted)
        n_drones = len(track.contributing_drones)

        self._publish_estimate(
            bssid, lat_e, lon_e, alt_e,
            cov_xx, cov_yy, cov_xy, n_acc,
            sigma_h, sigma_z_val, stable_2d, stable_3d, n_drones,
        )
        self._log_entry("fusion_mlat.jsonl", self._estimate_log_payload(
            track, bssid, drone_ns, lat_e, lon_e, alt_e,
            cov_xx, cov_yy, cov_xy, sigma_h, sigma_z_val,
            stable_2d, stable_3d, pose_label,
        ))

        if (track.n_published_3d + track.n_published_2d) % 10 == 1:
            self.get_logger().info(
                f"[{bssid}] {'3D' if publish_now_3d else '2D'} #"
                f"{track.n_published_3d + track.n_published_2d}: "
                f"({lat_e:.7f},{lon_e:.7f},{alt_e:.1f}) "
                f"σH={sigma_h:.2f} σZ={sigma_z_val:.2f} "
                f"n={ekf.n_accepted} drones={n_drones} "
                f"bias={ {d: round(ekf.bias_for(i), 2) for d, i in track.drone_to_bias_idx.items()} }"
            )

    # ─── Reset on divergence ─────────────────────────────────────────────

    def _maybe_reset(self, track: AnchorTrack, bssid: str) -> None:
        if not self._reset_on_divergence or track.ekf is None:
            return
        trigger: str | None = None
        if track._consecutive_bad >= self._divergence_reject_streak:
            trigger = f"{track._consecutive_bad} consecutive rejections"
        elif (len(track._nis_window) >= track._nis_window_max
              and sum(track._nis_window) / len(track._nis_window) >= self._divergence_nis_mean):
            trigger = (f"mean NIS = "
                       f"{sum(track._nis_window) / len(track._nis_window):.2f}")
        if trigger is None:
            return
        self.get_logger().warning(
            f"[{bssid}] divergence: {trigger}. Reset #{track.n_resets + 1}."
        )
        track.reset_ekf(reason=trigger)
        self._log_entry("fusion_state.jsonl", {
            "bssid": bssid, "event": "reset", "trigger": trigger,
            "n_resets": track.n_resets, "t": time.time(),
        })

    # ─── Publishing ──────────────────────────────────────────────────────

    def _publish_estimate(self, bssid, lat, lon, alt,
                          cov_xx, cov_yy, cov_xy, n_acc,
                          sigma_h, sigma_z, stable_2d, stable_3d, n_drones) -> None:
        ap_id = "ap_" + bssid.replace(":", "")
        topic = f"/fusion/anchor/{ap_id}/estimate"
        if topic not in self._anchor_pubs:
            self._anchor_pubs[topic] = self.create_publisher(Float64MultiArray, topic, 10)
        msg = Float64MultiArray()
        msg.data = [
            lat, lon, alt, cov_xx, cov_yy, cov_xy, n_acc,
            float(sigma_h), float(sigma_z),
            1.0 if stable_2d else 0.0, 1.0 if stable_3d else 0.0,
            float(n_drones),
        ]
        self._anchor_pubs[topic].publish(msg)

    def _publish_state(self, track: AnchorTrack, bssid: str, drone_ns: str,
                       stable_2d: bool, stable_3d: bool) -> None:
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
            1.0 if stable_2d else 0.0,
            1.0 if stable_3d else 0.0,
        ]
        try:
            pub.publish(msg)
        except Exception:
            pass

    def _publish_status(self) -> None:
        with self._tracks_lock:
            tracks_snap = dict(self._tracks)
        with self._poses_lock:
            drones = sorted(set(self._phone_poses) | set(self._drone_poses))
        status = {
            "t": time.time(),
            "n_drones": len(drones),
            "drones": drones,
            "n_aps": len(tracks_snap),
            "robust_method": self._robust_method,
            "robust_param": self._robust_param,
            "aps": {},
        }
        for bssid, track in tracks_snap.items():
            ekf = track.ekf
            ap = {
                "n_samples": len(track.samples),
                "n_drones": len(track.contributing_drones),
                "drones": sorted(track.contributing_drones),
                "n_resets": track.n_resets,
                "residual_rmse": (round(track.residual_rmse, 3)
                                  if math.isfinite(track.residual_rmse) else None),
            }
            if ekf is not None:
                ap.update({
                    "n_accepted": ekf.n_accepted,
                    "n_rejected": ekf.n_rejected,
                    "sigma_h": round(ekf.sigma_h, 3),
                    "sigma_z": round(ekf.sigma_z, 3),
                    "enu": [round(ekf.x[0], 2), round(ekf.x[1], 2), round(ekf.x[2], 2)],
                    "bias_m": {d: round(ekf.bias_for(i), 3)
                               for d, i in track.drone_to_bias_idx.items()},
                })
            status["aps"][bssid] = ap
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)

    # ─── Logging ─────────────────────────────────────────────────────────

    def _state_log_payload(self, track, bssid, drone_ns,
                           sigma_h, sigma_z, stable_2d, stable_3d, pose_label):
        ekf = track.ekf
        if ekf is None:
            return {"bssid": bssid, "t": time.time(), "drone": drone_ns}
        with self._origin_lock:
            origin = self._origin
        lat = lon = alt = math.nan
        if origin is not None:
            lat, lon, alt = enu_to_latlon(ekf.x[0], ekf.x[1], ekf.x[2], *origin)
        return {
            "bssid": bssid, "t": time.time(), "drone": drone_ns,
            "pose_source": pose_label,
            "lat": lat, "lon": lon, "alt": alt,
            "enu_x": ekf.x[0], "enu_y": ekf.x[1], "enu_z": ekf.x[2],
            "sigma_h": sigma_h, "sigma_z": sigma_z,
            "n_accepted": ekf.n_accepted, "n_rejected": ekf.n_rejected,
            "n_drones": len(track.contributing_drones),
            "bias_m": {d: ekf.bias_for(i) for d, i in track.drone_to_bias_idx.items()},
            "last_nis": ekf.last_nis, "last_innov_m": ekf.last_innov,
            "residual_rmse": track.residual_rmse,
            "stable_2d": stable_2d, "stable_3d": stable_3d,
            "n_resets": track.n_resets,
        }

    def _estimate_log_payload(self, track, bssid, drone_ns,
                              lat, lon, alt,
                              cov_xx, cov_yy, cov_xy,
                              sigma_h, sigma_z, stable_2d, stable_3d, pose_label):
        ekf = track.ekf
        return {
            "bssid": bssid, "t": time.time(), "drone": drone_ns,
            "pose_source": pose_label,
            "lat": lat, "lon": lon, "alt": alt,
            "cov_xx": cov_xx, "cov_yy": cov_yy, "cov_xy": cov_xy,
            "sigma_h": sigma_h, "sigma_z": sigma_z,
            "n_accepted": ekf.n_accepted if ekf else 0,
            "n_rejected": ekf.n_rejected if ekf else 0,
            "n_drones": len(track.contributing_drones),
            "bias_m": ({d: ekf.bias_for(i) for d, i in track.drone_to_bias_idx.items()}
                       if ekf else {}),
            "residual_rmse": track.residual_rmse,
            "stable_2d": stable_2d, "stable_3d": stable_3d,
            "n_published_2d": track.n_published_2d,
            "n_published_3d": track.n_published_3d,
        }

    def _log_entry(self, filename: str, data: dict) -> None:
        if self._log_dir is None:
            return
        if filename not in self._log_files:
            path = self._log_dir / filename
            self._log_files[filename] = open(path, "a", buffering=1, encoding="utf-8")
        f = self._log_files[filename]
        f.write(json.dumps(data, default=str) + "\n")

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

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Networked-OSA Fusion — online cooperative multilateration "
                    "across One-Search-Agent (OSA) entities",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--min-samples", type=int, default=6,
                        help="Minimum RTT observations to bootstrap GN")
    parser.add_argument("--z-mode", choices=Z_MODES, default=Z_MODE_AUTO,
                        help="Vertical handling: auto infers Z from range geometry; fixed_agl uses drone_z-agl_hint; none disables vertical priors")
    parser.add_argument("--agl-hint", type=float, default=0.0,
                        help="Approx OSA altitude above anchors (m); only used by fixed_agl or as auto fallback")
    parser.add_argument("--range-bias", type=float, default=0.0,
                        help="Known RTT bias subtracted before per-OSA bias estimation (m)")
    parser.add_argument("--sigma-threshold", type=float, default=2.5,
                        help="σ_H threshold (m) for 2D-stable publication")
    parser.add_argument("--sigma-z-threshold", type=float, default=20.0,
                        help="σ_Z threshold (m) for 3D-stable publication")
    parser.add_argument("--gnss-acc-max", type=float, default=5.0,
                        help="Discard GNSS fixes with acc > this (m)")
    parser.add_argument("--z-prior-sigma", type=float, default=10.0,
                        help="Initial σ of z prior (m). Decays with n_accepted")
    parser.add_argument("--z-prior-min-sigma", type=float, default=5.0,
                        help="Asymptotic minimum σ of z prior (m)")
    parser.add_argument("--log-dir", type=str, default=None,
                        help="Directory for JSONL logs")
    parser.add_argument("--discover-hz", type=float, default=0.2,
                        help="Topic-discovery frequency (Hz)")
    parser.add_argument("--reset-on-divergence", dest="reset_on_divergence",
                        action="store_true", default=True,
                        help="Reset EKF on sustained divergence")
    parser.add_argument("--no-reset-on-divergence", dest="reset_on_divergence",
                        action="store_false")
    parser.add_argument("--divergence-reject-streak", type=int, default=8)
    parser.add_argument("--divergence-nis-mean", type=float, default=6.0)
    parser.add_argument("--robust-method", choices=ROBUST_METHODS, default="sigma_clip",
                        help="Robust weighting for GN bootstrap and IEKF")
    parser.add_argument("--robust-param", type=float, default=2.5,
                        help="Huber k / trim fraction / sigma_clip k")
    parser.add_argument("--require-n-osa", type=int, default=2,
                        help="Min unique OSAs contributing for bootstrap")
    parser.add_argument("--require-n-osa-publish", type=int, default=2,
                        help="Min unique OSAs contributing for publish")
    parser.add_argument("--min-spread-m", type=float, default=15.0,
                        help="Min 2D bbox diag of bootstrap geometry (m)")
    parser.add_argument("--min-vertical-spread-m", type=float, default=0.0,
                        help="Min vertical spread of bootstrap geometry (m)")
    parser.add_argument("--pose-source", choices=POSE_SOURCES, default=POSE_SOURCE_AUTO,
                        help="Where to read drone pose from")
    parser.add_argument("--no-publish-2d-only", dest="publish_2d_only",
                        action="store_false", default=True,
                        help="Disable 2D-only estimate publication (only publish when 3D stable)")
    args = parser.parse_args()

    rclpy.init()
    node = MultiOsaFusionNode(
        min_samples=args.min_samples,
        agl_hint=args.agl_hint,
        range_bias=args.range_bias,
        z_mode=args.z_mode,
        sigma_h_threshold=args.sigma_threshold,
        sigma_z_threshold=args.sigma_z_threshold,
        gnss_acc_max=args.gnss_acc_max,
        z_prior_sigma=args.z_prior_sigma,
        z_prior_min_sigma=args.z_prior_min_sigma,
        log_dir=args.log_dir,
        discover_hz=args.discover_hz,
        reset_on_divergence=args.reset_on_divergence,
        divergence_reject_streak=args.divergence_reject_streak,
        divergence_nis_mean=args.divergence_nis_mean,
        robust_method=args.robust_method,
        robust_param=args.robust_param,
        require_n_osa=args.require_n_osa,
        require_n_osa_publish=args.require_n_osa_publish,
        min_spread_m=args.min_spread_m,
        min_vertical_spread_m=args.min_vertical_spread_m,
        pose_source=args.pose_source,
        publish_2d_only=args.publish_2d_only,
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
