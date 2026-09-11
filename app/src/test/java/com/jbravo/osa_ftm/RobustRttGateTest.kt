package com.jbravo.osa_ftm

import org.junit.Assert.*
import org.junit.Test
import kotlin.math.sqrt

class RobustRttGateTest {

    @Test
    fun warmupAcceptsAll() {
        val gate = RobustRttGate(windowMs = 5_000L, minN = 5)
        for (i in 0 until 5) {
            val (accepted, obs) = gate.pushAndGate(
                RttObs(tMonoMs = 1000L + i * 100, d = 10.0, sigma = 1.0, rssi = -50)
            )
            assertTrue("Sample $i should be accepted during warmup", accepted)
            assertNotNull(obs)
        }
    }

    @Test
    fun rejectsOutlierAfterWarmup() {
        val gate = RobustRttGate(windowMs = 10_000L, minN = 5, k = 3.0)
        // Fill with consistent samples at d=10.0
        for (i in 0 until 10) {
            gate.pushAndGate(
                RttObs(tMonoMs = 1000L + i * 100, d = 10.0, sigma = 1.0, rssi = -50)
            )
        }
        // An extreme outlier at d=100 should be rejected
        val (accepted, _) = gate.pushAndGate(
            RttObs(tMonoMs = 2500L, d = 100.0, sigma = 1.0, rssi = -50)
        )
        assertFalse("Extreme outlier should be rejected", accepted)
    }

    @Test
    fun acceptsNormalVariation() {
        val gate = RobustRttGate(windowMs = 10_000L, minN = 5, k = 3.5)
        // Fill with samples around d=10.0 ± 0.5
        for (i in 0 until 10) {
            gate.pushAndGate(
                RttObs(tMonoMs = 1000L + i * 100, d = 10.0 + (i % 3) * 0.3, sigma = 1.0, rssi = -50)
            )
        }
        // A sample at 10.5 should still be accepted
        val (accepted, obs) = gate.pushAndGate(
            RttObs(tMonoMs = 2500L, d = 10.5, sigma = 1.0, rssi = -50)
        )
        assertTrue("Normal variation should be accepted", accepted)
        assertNotNull(obs)
    }

    @Test
    fun replaceSigmaWithRobust() {
        val gate = RobustRttGate(windowMs = 10_000L, minN = 5, sigmaFloor = 0.12)
        for (i in 0 until 10) {
            gate.pushAndGate(
                RttObs(tMonoMs = 1000L + i * 100, d = 10.0, sigma = 5.0, rssi = -50)
            )
        }
        val (_, obs) = gate.pushAndGate(
            RttObs(tMonoMs = 2500L, d = 10.0, sigma = 5.0, rssi = -50)
        )
        assertNotNull(obs)
        // Sigma should be replaced with robust estimate
        // All samples are d=10.0, MAD=0 → sigma = max(1.4826*0, 0.12) = 0.12
        assertEquals(0.12, obs!!.sigma, 1e-6)
    }

    @Test
    fun prunesOldSamples() {
        val gate = RobustRttGate(windowMs = 1_000L, minN = 3)
        for (i in 0 until 5) {
            gate.pushAndGate(RttObs(tMonoMs = i.toLong() * 100, d = 10.0, sigma = 1.0, rssi = -50))
        }
        // After 2 seconds, all old samples should be pruned; this is warmup again
        val (accepted, _) = gate.pushAndGate(
            RttObs(tMonoMs = 5_000L, d = 50.0, sigma = 1.0, rssi = -50)  // outlier, but warmup
        )
        assertTrue("After prune, should be in warmup and accept", accepted)
    }
}
