package com.jbravo.osa_ftm

import org.junit.Assert.*
import org.junit.Test
import kotlin.math.sqrt

class AnchorIEKF3DTest {

    @Test
    fun predictIncreasesCovariance() {
        val ekf = AnchorIEKF3D(
            x0 = doubleArrayOf(0.0, 0.0, 0.0),
            p0 = arrayOf(
                doubleArrayOf(1.0, 0.0, 0.0),
                doubleArrayOf(0.0, 1.0, 0.0),
                doubleArrayOf(0.0, 0.0, 1.0)
            )
        )
        val pBefore = ekf.P[0][0]
        ekf.predict()
        assertTrue("Predict should increase P[0][0]", ekf.P[0][0] > pBefore)
    }

    @Test
    fun updateConvergesToTarget() {
        // Anchor at (10, 5, 0), drone measurements from various positions
        val anchorX = 10.0
        val anchorY = 5.0
        val anchorZ = 0.0

        val ekf = AnchorIEKF3D(
            x0 = doubleArrayOf(8.0, 3.0, 2.0),  // initial guess close but not exact
            p0 = arrayOf(
                doubleArrayOf(25.0, 0.0, 0.0),
                doubleArrayOf(0.0, 25.0, 0.0),
                doubleArrayOf(0.0, 0.0, 25.0)
            )
        )

        val dronePositions = listOf(
            Triple(30.0, 5.0, 20.0),
            Triple(-10.0, 5.0, 20.0),
            Triple(10.0, 25.0, 20.0),
            Triple(10.0, -15.0, 20.0),
            Triple(20.0, 15.0, 25.0),
            Triple(0.0, -5.0, 25.0),
        )

        for ((uE, uN, uU) in dronePositions) {
            val dx = anchorX - uE
            val dy = anchorY - uN
            val dz = anchorZ - uU
            val trueRange = sqrt(dx * dx + dy * dy + dz * dz)

            ekf.predict()
            ekf.updateIEKF(
                z = trueRange,
                sigma = 1.0,
                uE = uE,
                uN = uN,
                uU = uU
            )
        }

        assertEquals(anchorX, ekf.x[0], 3.0)
        assertEquals(anchorY, ekf.x[1], 3.0)
        assertEquals(anchorZ, ekf.x[2], 5.0)
    }

    @Test
    fun gatingRejectsLargeOutlier() {
        val ekf = AnchorIEKF3D(
            x0 = doubleArrayOf(0.0, 0.0, 0.0),
            p0 = arrayOf(
                doubleArrayOf(1.0, 0.0, 0.0),
                doubleArrayOf(0.0, 1.0, 0.0),
                doubleArrayOf(0.0, 0.0, 1.0)
            )
        )

        ekf.predict()
        // True range would be ~14.14m, but we give 500m — huge outlier
        val accepted = ekf.updateIEKF(
            z = 500.0,
            sigma = 1.0,
            uE = 10.0,
            uN = 10.0,
            uU = 0.0,
            gateGamma = 9.0
        )
        assertFalse("Large outlier should be rejected by chi2 gate", accepted)
    }

    @Test
    fun updateZPriorPullsZToward() {
        val ekf = AnchorIEKF3D(
            x0 = doubleArrayOf(0.0, 0.0, 50.0),  // z=50, way off
            p0 = arrayOf(
                doubleArrayOf(1.0, 0.0, 0.0),
                doubleArrayOf(0.0, 1.0, 0.0),
                doubleArrayOf(0.0, 0.0, 100.0)    // high uncertainty in z
            )
        )

        val ok = ekf.updateZPrior(zPrior = 0.0, sigmaZ = 5.0)
        assertTrue(ok)
        // z should move significantly toward 0
        assertTrue("z should be pulled toward prior", ekf.x[2] < 50.0)
    }

    @Test
    fun invalidInputReturnsFailure() {
        val ekf = AnchorIEKF3D(
            x0 = doubleArrayOf(0.0, 0.0, 0.0),
            p0 = arrayOf(
                doubleArrayOf(1.0, 0.0, 0.0),
                doubleArrayOf(0.0, 1.0, 0.0),
                doubleArrayOf(0.0, 0.0, 1.0)
            )
        )

        assertFalse(ekf.updateIEKF(z = Double.NaN, sigma = 1.0, uE = 0.0, uN = 0.0, uU = 0.0))
        assertFalse(ekf.updateIEKF(z = 10.0, sigma = 0.0, uE = 0.0, uN = 0.0, uU = 0.0))
        assertFalse(ekf.updateIEKF(z = 10.0, sigma = -1.0, uE = 0.0, uN = 0.0, uU = 0.0))
    }

    @Test
    fun nAcceptedIncrements() {
        val ekf = AnchorIEKF3D(
            x0 = doubleArrayOf(0.0, 0.0, 0.0),
            p0 = arrayOf(
                doubleArrayOf(100.0, 0.0, 0.0),
                doubleArrayOf(0.0, 100.0, 0.0),
                doubleArrayOf(0.0, 0.0, 100.0)
            )
        )
        assertEquals(0L, ekf.nAccepted)

        ekf.predict()
        val ok = ekf.updateIEKF(z = 10.0, sigma = 1.0, uE = 10.0, uN = 0.0, uU = 0.0)
        assertTrue(ok)
        assertEquals(1L, ekf.nAccepted)
    }
}
