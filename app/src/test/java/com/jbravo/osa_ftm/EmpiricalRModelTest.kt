package com.jbravo.osa_ftm

import org.junit.Assert.*
import org.junit.Test

class EmpiricalRModelTest {

    @Test
    fun closeRangeStrongSignalReturnsFloor() {
        val model = EmpiricalRModel()
        val r = model.computeR(1.0, -30, 0.5)
        // At d=1, RSSI=-30: sigma0Sq + alpha*1 + beta*10^0 = 0.09 + 0.0004 + 0.05 ≈ 0.14
        // gateSigma² = 0.25, so should return gateSigma² as floor
        assertEquals(0.25, r, 0.01)
    }

    @Test
    fun farRangeIncreaseR() {
        val model = EmpiricalRModel()
        val rClose = model.computeR(5.0, -50, 0.1)
        val rFar = model.computeR(50.0, -50, 0.1)
        assertTrue("R should grow with distance", rFar > rClose)
    }

    @Test
    fun weakSignalIncreasesR() {
        val model = EmpiricalRModel()
        val rStrong = model.computeR(10.0, -40, 0.1)
        val rWeak = model.computeR(10.0, -70, 0.1)
        assertTrue("R should grow with weaker signal", rWeak > rStrong)
    }

    @Test
    fun neverBelowGateSigmaSquared() {
        val model = EmpiricalRModel()
        val gateSigma = 5.0
        val r = model.computeR(1.0, -30, gateSigma)
        assertTrue("R should never be below gateSigma²", r >= gateSigma * gateSigma)
    }

    @Test
    fun zeroDistanceDoesNotCrash() {
        val model = EmpiricalRModel()
        val r = model.computeR(0.0, -50, 0.1)
        assertTrue(r.isFinite() && r > 0)
    }
}
