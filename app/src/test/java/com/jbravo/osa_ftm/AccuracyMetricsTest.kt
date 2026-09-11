package com.jbravo.osa_ftm

import org.junit.Assert.*
import org.junit.Test
import kotlin.math.sqrt

class AccuracyMetricsTest {

    @Test
    fun `empty metrics returns null summary`() {
        val m = AccuracyMetrics()
        assertNull(m.computeSummary())
        assertEquals(0, m.count)
    }

    @Test
    fun `single sample at exact ground truth has zero error`() {
        val m = AccuracyMetrics()
        m.addSample("aa:bb:cc:dd:ee:ff", 36.72, -4.42, 42.0, 36.72, -4.42, 42.0, 1000L)

        val s = m.computeSummary()!!
        assertEquals(1, s.nSamples)
        assertEquals(1, s.nAps)
        assertEquals(0.0, s.mean2D, 1e-6)
        assertEquals(0.0, s.mean3D, 1e-6)
        assertEquals(0.0, s.meanZ, 1e-6)
        assertEquals(0.0, s.cep50, 1e-6)
        assertEquals(0.0, s.cep95, 1e-6)
    }

    @Test
    fun `known offset produces expected horizontal error`() {
        val m = AccuracyMetrics()
        // Shift ~1 degree lat ≈ 111319.49 m
        // Instead use a tiny offset for sanity: 0.00001° lat ≈ 1.11m
        val dLat = 0.00001
        m.addSample("aa:bb:cc:dd:ee:ff",
            36.72 + dLat, -4.42, 42.0,
            36.72, -4.42, 42.0, 1000L)

        val s = m.computeSummary()!!
        // Expected: dLat * 111319.49 ≈ 1.113m
        val expected = dLat * 111319.49
        assertEquals(expected, s.mean2D, 0.01)
        assertEquals(0.0, s.meanZ, 1e-6)
        assertEquals(expected, s.mean3D, 0.01)
    }

    @Test
    fun `vertical error is signed`() {
        val m = AccuracyMetrics()
        // Estimate is 5m above ground truth
        m.addSample("aa:bb:cc:dd:ee:ff",
            36.72, -4.42, 47.0,
            36.72, -4.42, 42.0, 1000L)

        val s = m.computeSummary()!!
        assertEquals(5.0, s.meanZ, 1e-6)

        // Estimate is 3m below ground truth
        m.reset()
        m.addSample("aa:bb:cc:dd:ee:ff",
            36.72, -4.42, 39.0,
            36.72, -4.42, 42.0, 2000L)

        val s2 = m.computeSummary()!!
        assertEquals(-3.0, s2.meanZ, 1e-6)
    }

    @Test
    fun `per-AP summary separates bssids correctly`() {
        val m = AccuracyMetrics()
        // AP1: 2 samples
        m.addSample("aa:bb:cc:dd:ee:01", 36.72001, -4.42, 42.0, 36.72, -4.42, 42.0, 1000L)
        m.addSample("aa:bb:cc:dd:ee:01", 36.72002, -4.42, 42.0, 36.72, -4.42, 42.0, 2000L)
        // AP2: 1 sample
        m.addSample("aa:bb:cc:dd:ee:02", 36.72, -4.42001, 42.0, 36.72, -4.42, 42.0, 3000L)

        val s = m.computeSummary()!!
        assertEquals(3, s.nSamples)
        assertEquals(2, s.nAps)
        assertEquals(2, s.perAp["aa:bb:cc:dd:ee:01"]!!.nSamples)
        assertEquals(1, s.perAp["aa:bb:cc:dd:ee:02"]!!.nSamples)
    }

    @Test
    fun `cep95 is greater or equal to cep50`() {
        val m = AccuracyMetrics()
        // Add many samples with varying errors
        for (i in 1..20) {
            val dLat = i * 0.00001
            m.addSample("aa:bb:cc:dd:ee:ff",
                36.72 + dLat, -4.42, 42.0,
                36.72, -4.42, 42.0, (1000 + i).toLong())
        }

        val s = m.computeSummary()!!
        assertTrue(s.cep95 >= s.cep50)
        assertTrue(s.rmse2D > 0)
    }

    @Test
    fun `reset clears all data`() {
        val m = AccuracyMetrics()
        m.addSample("aa:bb:cc:dd:ee:ff", 36.72, -4.42, 42.0, 36.72, -4.42, 42.0, 1000L)
        assertEquals(1, m.count)

        m.reset()
        assertEquals(0, m.count)
        assertNull(m.computeSummary())
    }

    @Test
    fun `summaryToJson produces valid JSON string`() {
        val m = AccuracyMetrics()
        m.addSample("aa:bb:cc:dd:ee:ff", 36.72001, -4.42, 42.5, 36.72, -4.42, 42.0, 1000L)

        val json = m.summaryToJson("exp_123", "flight_30m", 30.0)
        assertNotNull(json)
        assertTrue(json!!.contains("\"type\":\"experiment_summary\""))
        assertTrue(json.contains("\"exp_id\":\"exp_123\""))
        assertTrue(json.contains("\"label\":\"flight_30m\""))
        assertTrue(json.contains("\"agl_m\":30.0"))
        assertTrue(json.contains("\"per_ap\":["))
    }

    @Test
    fun `summaryToJson returns null when empty`() {
        val m = AccuracyMetrics()
        assertNull(m.summaryToJson("exp_123", "test", 10.0))
    }
}
