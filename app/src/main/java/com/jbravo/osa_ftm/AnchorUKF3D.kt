package com.jbravo.osa_ftm

import kotlin.math.sqrt

/**
 * 3D Unscented Kalman Filter for anchor position estimation.
 *
 * Unlike the IEKF which linearises via Jacobians, the UKF propagates
 * deterministic "sigma points" through the non-linear measurement model
 * h(x) = ‖x − u‖, capturing the curvature without derivatives.
 *
 * State: x = [E, N, U]  (anchor position in ENU)
 * Measurement: z = ‖x − u‖ + noise  (range from drone at u)
 *
 * Uses the standard Merwe scaled sigma-point formulation:
 *   α = 1e-3, β = 2.0, κ = 0.0
 */
class AnchorUKF3D(
    x0: DoubleArray,
    p0: Array<DoubleArray>
) {
    private val n = 3  // state dimension

    val x: DoubleArray = x0.copyOf()
    var P: Array<DoubleArray> = p0.map { it.copyOf() }.toTypedArray()

    var nAccepted: Long = 0L

    // Process noise (same as IEKF3D for fair comparison)
    private val qx = 0.01
    private val qy = 0.01
    private val qz = 0.04

    // UKF parameters (Merwe scaled)
    private val alpha = 1e-3
    private val beta = 2.0
    private val kappa = 0.0
    private val lambda = alpha * alpha * (n + kappa) - n  // ≈ -2.999999

    fun predict() {
        P[0][0] += qx
        P[1][1] += qy
        P[2][2] += qz
    }

    /**
     * UKF measurement update with range observation.
     *
     * @param z     Measured range (metres).
     * @param sigma Measurement std dev (metres).
     * @param uE    Drone East coordinate.
     * @param uN    Drone North coordinate.
     * @param uU    Drone Up coordinate.
     * @param gateGamma NIS gate threshold (same as IEKF: 9.0 ≈ 3σ for 1-DOF).
     * @return true if the measurement was accepted.
     */
    fun update(
        z: Double,
        sigma: Double,
        uE: Double,
        uN: Double,
        uU: Double,
        gateGamma: Double = 9.0
    ): Boolean {
        if (!z.isFinite() || !sigma.isFinite() || sigma <= 0.0) return false

        val R = sigma * sigma

        // 1) Generate 2n+1 = 7 sigma points
        val sqrtP = choleskyLower(scaleMat(P, n.toDouble() + lambda)) ?: return false
        val sigPts = Array(2 * n + 1) { DoubleArray(n) }
        sigPts[0] = x.copyOf()
        // L = cholesky(P*(n+λ)); sigma points use columns of L
        for (i in 0 until n) {
            sigPts[1 + i] = doubleArrayOf(
                x[0] + sqrtP[0][i], x[1] + sqrtP[1][i], x[2] + sqrtP[2][i]
            )
            sigPts[1 + n + i] = doubleArrayOf(
                x[0] - sqrtP[0][i], x[1] - sqrtP[1][i], x[2] - sqrtP[2][i]
            )
        }

        // 2) Weights
        val wm0 = lambda / (n + lambda)
        val wc0 = wm0 + (1.0 - alpha * alpha + beta)
        val wi = 0.5 / (n + lambda)

        // 3) Transform sigma points through measurement model h(x) = ‖x − u‖
        val zPts = DoubleArray(2 * n + 1)
        for (i in sigPts.indices) {
            val dx = sigPts[i][0] - uE
            val dy = sigPts[i][1] - uN
            val dz = sigPts[i][2] - uU
            zPts[i] = sqrt(dx * dx + dy * dy + dz * dz).coerceAtLeast(1e-6)
        }

        // 4) Predicted measurement mean
        var zMean = wm0 * zPts[0]
        for (i in 1 until 2 * n + 1) zMean += wi * zPts[i]

        // 5) Innovation covariance Pzz and cross-covariance Pxz
        var Pzz = wc0 * (zPts[0] - zMean) * (zPts[0] - zMean) + R
        val Pxz = DoubleArray(n)
        for (j in 0 until n) Pxz[j] = wc0 * (sigPts[0][j] - x[j]) * (zPts[0] - zMean)

        for (i in 1 until 2 * n + 1) {
            val dz2 = zPts[i] - zMean
            Pzz += wi * dz2 * dz2
            for (j in 0 until n) Pxz[j] += wi * (sigPts[i][j] - x[j]) * dz2
        }

        if (!Pzz.isFinite() || Pzz <= 0.0) return false

        // 6) Innovation gating
        val innovation = z - zMean
        val nis = (innovation * innovation) / Pzz
        if (nis > gateGamma) return false

        // 7) Kalman gain K = Pxz / Pzz
        val K = DoubleArray(n)
        for (j in 0 until n) K[j] = Pxz[j] / Pzz

        // 8) State update
        for (j in 0 until n) x[j] += K[j] * innovation

        // 9) Covariance update: P = P − K·Pzz·K^T
        for (i in 0 until n) for (j in 0 until n) {
            P[i][j] -= K[i] * Pzz * K[j]
        }

        // 10) Enforce symmetry
        for (i in 0 until n) for (j in i + 1 until n) {
            val avg = 0.5 * (P[i][j] + P[j][i])
            P[i][j] = avg
            P[j][i] = avg
        }

        nAccepted++
        return true
    }

    /**
     * Z-prior regularization (same as IEKF3D, but using UKF update).
     * Since h(x) = x[2] is linear, this is equivalent to a standard KF update.
     */
    fun updateZPrior(zPrior: Double, sigmaZ: Double = 10.0): Boolean {
        if (!zPrior.isFinite() || !sigmaZ.isFinite() || sigmaZ <= 0.0) return false

        val R = sigmaZ * sigmaZ
        val r = zPrior - x[2]
        val S = P[2][2] + R
        if (!S.isFinite() || S <= 0.0) return false

        val K = doubleArrayOf(P[0][2] / S, P[1][2] / S, P[2][2] / S)
        x[0] += K[0] * r
        x[1] += K[1] * r
        x[2] += K[2] * r

        // P -= K * S * K^T
        for (i in 0 until n) for (j in 0 until n) {
            P[i][j] -= K[i] * S * K[j]
        }
        // Enforce symmetry
        for (i in 0 until n) for (j in i + 1 until n) {
            val avg = 0.5 * (P[i][j] + P[j][i])
            P[i][j] = avg
            P[j][i] = avg
        }
        return true
    }

    // --------- helpers ---------

    /**
     * Cholesky decomposition: A = L L^T where L is lower-triangular.
     * Returns L or null if A is not positive definite.
     */
    internal fun choleskyLower(A: Array<DoubleArray>): Array<DoubleArray>? {
        val L = Array(n) { DoubleArray(n) }
        for (i in 0 until n) {
            for (j in 0..i) {
                var sum = A[i][j]
                for (k in 0 until j) sum -= L[i][k] * L[j][k]
                if (i == j) {
                    if (sum <= 0.0) return null  // Not positive definite
                    L[i][j] = sqrt(sum)
                } else {
                    L[i][j] = sum / L[j][j]
                }
            }
        }
        return L
    }

    private fun scaleMat(A: Array<DoubleArray>, s: Double): Array<DoubleArray> {
        return Array(n) { i -> DoubleArray(n) { j -> A[i][j] * s } }
    }
}
