package com.jbravo.osa_ftm

import kotlin.math.sqrt

/**
 * Iterated Extended Kalman Filter for anchor position estimation.
 *
 * State vector  x = [xE, yN, zU, b]  (ENU coordinates + range bias in metres).
 * Measurement model:  h(x) = ‖p_drone − p_AP‖ + b
 * Jacobian:           H = [dx/d, dy/d, dz/d, 1]
 *
 * The bias state `b` is estimated online (auto-calibration).  It captures the
 * systematic RTT range offset for this AP (hardware delay, multipath baseline,
 * etc.) without requiring a prior calibration step.
 */
class AnchorIEKF3D(
    x0: DoubleArray,          // length 3 or 4: [xE, yN, zU, (b0)]
    p0: Array<DoubleArray>    // 3x3 or 4x4 initial covariance
) {
    companion object {
        private const val N = 4
        private const val LEGACY_BIAS_VARIANCE = 25.0
    }

    init {
        require(x0.size == 3 || x0.size == N) { "x0 must contain 3 or 4 elements" }
        require(p0.size == x0.size && p0.all { it.size == x0.size }) {
            "p0 must be a square matrix matching x0"
        }
    }

    val x: DoubleArray = x0.copyOf(N)      // [xE, yN, zU, b]
    var P: Array<DoubleArray> = Array(N) { row ->
        DoubleArray(N) { column ->
            when {
                row < p0.size && column < p0.size -> p0[row][column]
                row == 3 && column == 3 -> LEGACY_BIAS_VARIANCE
                else -> 0.0
            }
        }
    }

    var nAccepted: Long = 0L

    /** Estimated range bias (convenience accessor). */
    val bias: Double get() = x[3]

    // Ruido de proceso (anchor casi estático, bias random-walk lento)
    private val qx = 0.01   // m² por step
    private val qy = 0.01
    private val qz = 0.04
    private val qb = 0.0001 // bias drift ≈ 0.01 m/√step

    fun predict() {
        P[0][0] += qx
        P[1][1] += qy
        P[2][2] += qz
        P[3][3] += qb
    }

    /**
     * Update iterativo con medición de rango:
     * h(x) = sqrt((x-uE)² + (y-uN)² + (z-uU)²) + b
     */
    fun updateIEKF(
        z: Double,
        sigma: Double,
        uE: Double,
        uN: Double,
        uU: Double,
        iters: Int = 3,
        gateGamma: Double = 9.0
    ): Boolean {
        if (!z.isFinite() || !sigma.isFinite() || sigma <= 0.0) return false

        val R = sigma * sigma

        // Gating con el estado actual (antes de iterar)
        run {
            val dx = x[0] - uE
            val dy = x[1] - uN
            val dz = x[2] - uU
            val d = sqrt(dx * dx + dy * dy + dz * dz).coerceAtLeast(1e-6)
            val h = d + x[3]       // h(x) = distance + bias
            val innov = z - h
            val H = doubleArrayOf(dx / d, dy / d, dz / d, 1.0)

            val S = quadForm(H, P) + R
            if (!S.isFinite() || S <= 0.0) return false

            val nis = (innov * innov) / S
            if (nis > gateGamma) return false
        }

        // Iteraciones IEKF
        repeat(iters.coerceAtLeast(1)) {
            val dx = x[0] - uE
            val dy = x[1] - uN
            val dz = x[2] - uU
            val d = sqrt(dx * dx + dy * dy + dz * dz).coerceAtLeast(1e-6)
            val h = d + x[3]
            val innov = z - h

            val H = doubleArrayOf(dx / d, dy / d, dz / d, 1.0)
            val S = quadForm(H, P) + R
            if (!S.isFinite() || S <= 0.0) return false

            // K = P Hᵀ / S
            val PHt = matVec(P, H)
            val K = DoubleArray(N) { PHt[it] / S }

            // x = x + K * innovation
            for (i in 0 until N) x[i] += K[i] * innov

            // Joseph form: P = (I-KH) P (I-KH)ᵀ + K R Kᵀ
            val IKH = eye()
            for (i in 0 until N) for (j in 0 until N) IKH[i][j] -= K[i] * H[j]

            val tmp = matMul(IKH, matMul(P, transpose(IKH)))
            for (i in 0 until N) for (j in 0 until N) tmp[i][j] += K[i] * R * K[j]

            P = tmp
        }

        nAccepted++
        return true
    }

    /**
     * Pseudo-medición para regularizar z hacia una "cota de suelo" aproximada:
     * z_prior ~= ground_median  (sigmaZ grande => prior débil)
     */
    fun updateZPrior(zPrior: Double, sigmaZ: Double = 10.0): Boolean {
        if (!zPrior.isFinite() || !sigmaZ.isFinite() || sigmaZ <= 0.0) return false

        val R = sigmaZ * sigmaZ
        val innov = zPrior - x[2]
        val H = DoubleArray(N).also { it[2] = 1.0 }   // [0, 0, 1, 0]

        val S = quadForm(H, P) + R
        if (!S.isFinite() || S <= 0.0) return false

        val PHt = matVec(P, H)
        val K = DoubleArray(N) { PHt[it] / S }

        for (i in 0 until N) x[i] += K[i] * innov

        val IKH = eye()
        for (i in 0 until N) for (j in 0 until N) IKH[i][j] -= K[i] * H[j]
        val tmp = matMul(IKH, matMul(P, transpose(IKH)))
        for (i in 0 until N) for (j in 0 until N) tmp[i][j] += K[i] * R * K[j]

        P = tmp
        return true
    }

    // --------- N×N helpers ─────────────────────────────────────

    private fun eye(): Array<DoubleArray> {
        val m = Array(N) { DoubleArray(N) }
        for (i in 0 until N) m[i][i] = 1.0
        return m
    }

    private fun quadForm(H: DoubleArray, P: Array<DoubleArray>): Double {
        val v = matVec(P, H)
        var s = 0.0
        for (i in 0 until N) s += H[i] * v[i]
        return s
    }

    private fun matVec(A: Array<DoubleArray>, v: DoubleArray): DoubleArray {
        val r = DoubleArray(N)
        for (i in 0 until N) for (j in 0 until N) r[i] += A[i][j] * v[j]
        return r
    }

    private fun transpose(A: Array<DoubleArray>): Array<DoubleArray> {
        val r = Array(N) { DoubleArray(N) }
        for (i in 0 until N) for (j in 0 until N) r[i][j] = A[j][i]
        return r
    }

    private fun matMul(A: Array<DoubleArray>, B: Array<DoubleArray>): Array<DoubleArray> {
        val r = Array(N) { DoubleArray(N) }
        for (i in 0 until N) for (j in 0 until N)
            for (k in 0 until N) r[i][j] += A[i][k] * B[k][j]
        return r
    }
}
