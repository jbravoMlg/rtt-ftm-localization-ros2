package com.jbravo.osa_ftm

import kotlin.math.sqrt

// (IEKF 2D con dz = AGL)


class AnchorIEKF2D(x0: DoubleArray, p0: Array<DoubleArray>) {
    var x = x0.copyOf()                       // [E, N]
    var P = p0.map { it.copyOf() }.toTypedArray() // 2x2

    var nAccepted: Long = 0
        private set

    private val q = 1e-6 // anchor casi estático (m^2 por update)

    fun predict() {
        P[0][0] += q
        P[1][1] += q
    }

    fun updateIEKF(
        z: Double,
        sigma: Double,
        uE: Double, uN: Double,
        dz: Double,
        iters: Int = 3,
        gateGamma: Double = 9.0
    ): Boolean {
        val R = sigma * sigma

        var xIt = x.copyOf()
        var pIt = P.map { it.copyOf() }.toTypedArray()

        for (k in 0 until iters) {
            val dE = xIt[0] - uE
            val dN = xIt[1] - uN
            val h = sqrt(dE*dE + dN*dN + dz*dz).coerceAtLeast(1e-6)

            val H0 = dE / h
            val H1 = dN / h

            val S =
                (H0*(pIt[0][0]*H0 + pIt[0][1]*H1) +
                        H1*(pIt[1][0]*H0 + pIt[1][1]*H1)) + R

            val v = z - h

            if (k == 0 && v*v > gateGamma * S) return false  // gating por innovación

            val K0 = (pIt[0][0]*H0 + pIt[0][1]*H1) / S
            val K1 = (pIt[1][0]*H0 + pIt[1][1]*H1) / S

            xIt[0] += K0 * v
            xIt[1] += K1 * v

            val I00 = 1.0 - K0*H0
            val I01 =      - K0*H1
            val I10 =      - K1*H0
            val I11 = 1.0 - K1*H1

            val P00 = I00*pIt[0][0] + I01*pIt[1][0]
            val P01 = I00*pIt[0][1] + I01*pIt[1][1]
            val P10 = I10*pIt[0][0] + I11*pIt[1][0]
            val P11 = I10*pIt[0][1] + I11*pIt[1][1]

            pIt[0][0] = P00; pIt[0][1] = P01
            pIt[1][0] = P10; pIt[1][1] = P11
        }

        x = xIt
        P = pIt
        nAccepted++
        return true
    }
}
