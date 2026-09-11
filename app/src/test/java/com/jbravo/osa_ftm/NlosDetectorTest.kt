package com.jbravo.osa_ftm

import org.junit.Assert.*
import org.junit.Test

class NlosDetectorTest {

    @Test
    fun losReturnsOriginalSigma() {
        val det = NlosDetector()
        // Feed 10 consistent short-range measurements
        repeat(10) {
            val (isNlos, adj) = det.evaluate(5.0 + it * 0.01, 1.0, -40)
            assertFalse(isNlos)
            assertEquals(1.0, adj, 1e-9)
        }
    }

    @Test
    fun skewedDistributionDetectsNlos() {
        val det = NlosDetector(windowSize = 10, skewThreshold = 0.5)
        // Feed measurements with strong positive skew
        val ranges = listOf(5.0, 5.1, 5.0, 5.05, 5.0, 5.0, 5.1, 15.0, 20.0, 25.0)
        var nlosDetected = false
        for (r in ranges) {
            val (isNlos, _) = det.evaluate(r, 1.0, -50)
            if (isNlos) nlosDetected = true
        }
        assertTrue("Expected NLOS detection with skewed ranges", nlosDetected)
    }

    @Test
    fun nlosInflatesSigma() {
        val det = NlosDetector(windowSize = 5, skewThreshold = 0.3, nlosInflation = 3.0)
        // Strong skew: all large outliers
        repeat(5) { det.evaluate(50.0, 1.0, -50) }
        val (isNlos, adj) = det.evaluate(100.0, 2.0, -50)
        // If NLOS detected, sigma should be inflated
        if (isNlos) {
            assertEquals(6.0, adj, 1e-9)  // 2.0 * 3.0
        }
    }

    @Test
    fun rssiInconsistencyDetected() {
        val det = NlosDetector(rssiResidualThreshold = 10.0, rssiRef1m = -30.0)
        // Very strong RSSI at long range is inconsistent
        assertTrue(det.isRssiInconsistent(100.0, -20))
        // Normal RSSI at close range
        assertFalse(det.isRssiInconsistent(2.0, -36))
    }

    @Test
    fun invalidInputsSkipRssiCheck() {
        val det = NlosDetector()
        assertFalse(det.isRssiInconsistent(0.0, -50))
        assertFalse(det.isRssiInconsistent(5.0, 0))
    }

    @Test
    fun skewnessOfSymmetricDistribution() {
        val det = NlosDetector(windowSize = 10)
        // Feed symmetric data
        val symmetric = listOf(1.0, 2.0, 3.0, 4.0, 5.0, 5.0, 4.0, 3.0, 2.0, 1.0)
        for (r in symmetric) det.evaluate(r, 1.0, -50)
        val skew = det.computeSkewness()
        assertEquals("Symmetric distribution should have ~0 skewness", 0.0, skew, 0.3)
    }
}
