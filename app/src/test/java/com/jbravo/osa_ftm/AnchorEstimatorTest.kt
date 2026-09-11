package com.jbravo.osa_ftm

import org.junit.Assert.*
import org.junit.Test
import kotlin.math.sqrt
import kotlin.math.cos
import kotlin.math.PI

class AnchorEstimatorTest {

    // Origin near Madrid for tests
    private val lat0 = 40.0
    private val lon0 = -3.7
    private val alt0 = 600.0
    private val agl = 30.0  // drone at 30m AGL

    @Test
    fun latLonToEnuOriginIsZero() {
        val est = AnchorEstimator(lat0, lon0, alt0, agl)
        val (x, y, z) = est.latLonToENU(lat0, lon0, alt0)
        assertEquals(0.0, x, 1e-6)
        assertEquals(0.0, y, 1e-6)
        assertEquals(0.0, z, 1e-6)
    }

    @Test
    fun latLonToEnuNorthIsPositiveY() {
        val est = AnchorEstimator(lat0, lon0, alt0, agl)
        // 0.001° north ≈ 111 m
        val (x, y, z) = est.latLonToENU(lat0 + 0.001, lon0, alt0)
        assertEquals(0.0, x, 1.0)           // East ~ 0
        assertTrue("y should be ~111m north", y > 100 && y < 120)
        assertEquals(0.0, z, 1e-6)
    }

    @Test
    fun latLonToEnuEastIsPositiveX() {
        val est = AnchorEstimator(lat0, lon0, alt0, agl)
        val (x, y, z) = est.latLonToENU(lat0, lon0 + 0.001, alt0)
        assertTrue("x should be positive east", x > 50 && x < 120)
        assertEquals(0.0, y, 1.0)
        assertEquals(0.0, z, 1e-6)
    }

    @Test
    fun latLonToEnuAltIsUp() {
        val est = AnchorEstimator(lat0, lon0, alt0, agl)
        val (_, _, z) = est.latLonToENU(lat0, lon0, alt0 + 50.0)
        assertEquals(50.0, z, 1e-6)
    }

    @Test
    fun enuToLatLonRoundTrip() {
        val est = AnchorEstimator(lat0, lon0, alt0, agl)
        val testLat = lat0 + 0.002
        val testLon = lon0 - 0.001
        val testAlt = alt0 + 25.0

        val (x, y, z) = est.latLonToENU(testLat, testLon, testAlt)
        val (lat2, lon2, alt2) = est.enuToLatLon(x, y, z)

        assertEquals(testLat, lat2, 1e-6)
        assertEquals(testLon, lon2, 1e-6)
        assertEquals(testAlt, alt2, 1e-6)
    }

    @Test
    fun notEnoughSamplesReturnsNull() {
        val est = AnchorEstimator(lat0, lon0, alt0, agl)
        // Only 3 samples — need at least 4 for 3D
        for (i in 0 until 3) {
            est.addSampleFromGpsAndRtt(
                GpsPose(lat0, lon0, alt0 + agl, 0, 0, 1.0, "test"),
                RttSample(0, 30.0, 1.0, -50)
            )
        }
        assertNull(est.refineGaussNewton3D())
    }

    @Test
    fun estimateConvergesWithGoodGeometry() {
        val est = AnchorEstimator(lat0, lon0, alt0, agl)

        // Simulated anchor at ENU (20, 15, 0)
        val anchorX = 20.0
        val anchorY = 15.0
        val anchorZ = 0.0

        // Generate 20 drone positions in a circle at altitude alt0+agl
        val nSamples = 20
        val radius = 40.0  // 40m circle radius

        for (i in 0 until nSamples) {
            val angle = 2.0 * PI * i / nSamples
            val droneX = anchorX + radius * cos(angle)
            val droneY = anchorY + radius * kotlin.math.sin(angle)
            val droneZ = agl  // z in ENU = droneAlt - alt0 = agl

            // True 3D distance from drone to anchor
            val dx = droneX - anchorX
            val dy = droneY - anchorY
            val dz = droneZ - anchorZ
            val trueD = sqrt(dx * dx + dy * dy + dz * dz)

            // Convert drone ENU back to lat/lon for the GpsPose
            val (dLat, dLon, dAlt) = est.enuToLatLon(droneX, droneY, droneZ)

            est.addSampleFromGpsAndRtt(
                GpsPose(dLat, dLon, dAlt, 0, 0, 0.5, "test"),
                RttSample(0, trueD, 1.0, -50)
            )
        }

        val result = est.refineGaussNewton3D()
        assertNotNull("Should converge with good geometry", result)
        assertEquals(anchorX, result!!.x, 2.0)
        assertEquals(anchorY, result.y, 2.0)
        assertEquals(anchorZ, result.z, 5.0)
    }

    @Test
    fun clearResetsSamples() {
        val est = AnchorEstimator(lat0, lon0, alt0, agl)
        for (i in 0 until 10) {
            est.addSampleFromGpsAndRtt(
                GpsPose(lat0, lon0, alt0 + agl, 0, 0, 1.0, "test"),
                RttSample(0, 30.0, 1.0, -50)
            )
        }
        assertEquals(10, est.sampleCount())
        est.clear()
        assertEquals(0, est.sampleCount())
    }

    @Test
    fun setWeightingChangesScheme() {
        val est = AnchorEstimator(lat0, lon0, alt0, agl)
        // Should not throw
        est.setWeighting(Weighting.Huber(1.5))
        est.setWeighting(Weighting.Trim(0.1))
        est.setWeighting(Weighting.SigmaClip(2.5))
        est.setWeighting(Weighting.WLS)
    }

    @Test
    fun insufficientSpreadReturnsNull() {
        val est = AnchorEstimator(lat0, lon0, alt0, agl)

        // 8 positions clustered within ~3m spread at 57m range.
        // This geometry is too poor for GN — should return null.
        val anchorX = 40.0
        val anchorY = 15.0
        val anchorZ = 0.0

        for (i in 0 until 8) {
            // Tiny cluster: droneX ∈ [0, 3], droneY ∈ [0, 0]
            val droneX = i * 0.4
            val droneY = 0.0
            val droneZ = agl
            val dx = droneX - anchorX
            val dy = droneY - anchorY
            val dz = droneZ - anchorZ
            val trueD = sqrt(dx * dx + dy * dy + dz * dz)

            val (dLat, dLon, dAlt) = est.enuToLatLon(droneX, droneY, droneZ)
            est.addSampleFromGpsAndRtt(
                GpsPose(dLat, dLon, dAlt, 0, 0, 0.5, "test"),
                RttSample(0, trueD, 1.0, -50)
            )
        }
        assertNull("GN should refuse with <15m spread", est.refineGaussNewton3D())
        assertEquals(8, est.sampleCount()) // samples still accumulated
    }
}
