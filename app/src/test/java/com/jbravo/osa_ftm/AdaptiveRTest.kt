package com.jbravo.osa_ftm

import org.junit.Assert.*
import org.junit.Test

class AdaptiveRTest {

    @Test
    fun initialRIsOne() {
        val ar = AdaptiveR()
        assertEquals(1.0, ar.currentR(), 1e-9)
    }

    @Test
    fun largeInnovationIncreasesR() {
        val ar = AdaptiveR(alpha = 0.5)
        val before = ar.currentR()
        ar.update(innovation = 5.0, hpht = 0.1)  // ν²=25, hpht=0.1 → rSample=24.9
        val after = ar.currentR()
        assertTrue("Large innovation should increase R", after > before)
    }

    @Test
    fun smallInnovationDecreasesR() {
        val ar = AdaptiveR(alpha = 0.5)
        // Start at R=1, innovation is very small
        ar.update(innovation = 0.1, hpht = 0.0)  // ν²=0.01 → rSample≈0.01
        val after = ar.currentR()
        assertTrue("Small innovation should decrease R", after < 1.0)
    }

    @Test
    fun rNeverBelowFloor() {
        val ar = AdaptiveR(alpha = 1.0, rFloor = 0.01)
        repeat(100) {
            ar.update(innovation = 0.0, hpht = 0.0)
        }
        assertTrue(ar.currentR() >= 0.01)
    }

    @Test
    fun rNeverAboveCeiling() {
        val ar = AdaptiveR(alpha = 1.0, rCeiling = 25.0)
        repeat(100) {
            ar.update(innovation = 100.0, hpht = 0.0)
        }
        assertTrue(ar.currentR() <= 25.0)
    }

    @Test
    fun updateReturnsCurrentEstimate() {
        val ar = AdaptiveR()
        val returned = ar.update(innovation = 1.0, hpht = 0.0)
        assertEquals(ar.currentR(), returned, 1e-9)
    }
}
