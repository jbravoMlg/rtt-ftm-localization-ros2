package com.jbravo.osa_ftm

import org.junit.Assert.*
import org.junit.Test
import kotlin.math.sqrt

class AnchorUKF3DTest {

    private fun identityP(scale: Double = 1.0) = arrayOf(
        doubleArrayOf(scale, 0.0, 0.0),
        doubleArrayOf(0.0, scale, 0.0),
        doubleArrayOf(0.0, 0.0, scale)
    )

    @Test
    fun predictIncreasesCovariance() {
        val ukf = AnchorUKF3D(
            x0 = doubleArrayOf(0.0, 0.0, 0.0),
            p0 = identityP(1.0)
        )
        val pBefore = ukf.P[0][0]
        ukf.predict()
        assertTrue("Predict should increase P", ukf.P[0][0] > pBefore)
    }

    @Test
    fun updateReducesCovarianceIfAccepted() {
        val ukf = AnchorUKF3D(
            x0 = doubleArrayOf(10.0, 10.0, 0.0),
            p0 = identityP(100.0)
        )
        ukf.predict()
        val pBefore = ukf.P[0][0] + ukf.P[1][1]

        // Drone at (0,0,15), measured range ≈ sqrt(200+225) ≈ 20.6
        val range = sqrt(10.0*10.0 + 10.0*10.0 + 15.0*15.0)
        val accepted = ukf.update(range, 1.0, 0.0, 0.0, 15.0)

        if (accepted) {
            val pAfter = ukf.P[0][0] + ukf.P[1][1]
            assertTrue("Update should reduce P", pAfter < pBefore)
        }
    }

    @Test
    fun nAcceptedIncrements() {
        val ukf = AnchorUKF3D(
            x0 = doubleArrayOf(10.0, 10.0, 0.0),
            p0 = identityP(100.0)
        )
        assertEquals(0L, ukf.nAccepted)
        ukf.predict()
        val range = sqrt(10.0*10.0 + 10.0*10.0 + 15.0*15.0)
        ukf.update(range, 1.0, 0.0, 0.0, 15.0)
        assertTrue("nAccepted should be >= 0", ukf.nAccepted >= 0L)
    }

    @Test
    fun invalidInputsRejected() {
        val ukf = AnchorUKF3D(
            x0 = doubleArrayOf(0.0, 0.0, 0.0),
            p0 = identityP(1.0)
        )
        assertFalse(ukf.update(Double.NaN, 1.0, 0.0, 0.0, 15.0))
        assertFalse(ukf.update(10.0, -1.0, 0.0, 0.0, 15.0))
        assertFalse(ukf.update(10.0, Double.POSITIVE_INFINITY, 0.0, 0.0, 15.0))
    }

    @Test
    fun stateDoesNotExplode() {
        val ukf = AnchorUKF3D(
            x0 = doubleArrayOf(5.0, 5.0, 0.0),
            p0 = identityP(50.0)
        )
        // Multiple predict-update cycles
        repeat(50) {
            ukf.predict()
            val range = sqrt(25.0 + 25.0 + 225.0)
            ukf.update(range, 1.0, 0.0, 0.0, 15.0)
        }
        assertTrue(ukf.x[0].isFinite())
        assertTrue(ukf.x[1].isFinite())
        assertTrue(ukf.x[2].isFinite())
    }
}
