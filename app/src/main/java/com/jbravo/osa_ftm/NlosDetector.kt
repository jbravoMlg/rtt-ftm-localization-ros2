package com.jbravo.osa_ftm

import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * NLOS (Non-Line-of-Sight) detector for WiFi FTM range measurements.
 *
 * Uses two complementary heuristics:
 * 1. **Skewness**: NLOS errors are systematically positive (longer path),
 *    so the distribution of recent ranges skews right (γ₁ > threshold).
 * 2. **RSSI-range consistency**: Low RSSI with short range (or vice versa)
 *    is incoherent and suggests multipath / NLOS.
 *
 * When NLOS is detected, the returned sigma is inflated (not rejected)
 * so the EKF naturally down-weights the measurement.
 */
class NlosDetector(
    /** Rolling window size for skewness computation. */
    private val windowSize: Int = 20,
    /** Skewness threshold: γ₁ > this → suspect NLOS. */
    private val skewThreshold: Double = 1.0,
    /** Sigma inflation factor when NLOS is detected. */
    private val nlosInflation: Double = 3.0,
    /**
     * Expected path-loss exponent for RSSI-range model.
     * Free-space = 2.0; outdoor ground-to-air ≈ 2.5-3.0; indoor ≈ 3.0-4.0.
     * Using 3.0 for outdoor drone scenarios to avoid false NLOS at long range.
     */
    private val pathLossExponent: Double = 3.0,
    /**
     * RSSI residual threshold (dB): if |RSSI_measured - RSSI_expected| > this,
        * the measurement is inconsistent. Raised to 20 dB for outdoor variability
     * (ground reflections, antenna patterns, body shadowing).
     */
    private val rssiResidualThreshold: Double = 20.0,
    /** Reference RSSI at 1 m distance (typical WiFi: -30 to -40 dBm). */
    private val rssiRef1m: Double = -35.0
) {
    private val recentRanges = ArrayDeque<Double>()

    /**
     * Evaluate an observation and return the adjusted sigma.
     *
     * @param distance Range measurement in metres.
     * @param sigma    Current robust sigma from the RTT gate.
     * @param rssi     Received signal strength (dBm).
     * @return Pair of (isNlos: Boolean, adjustedSigma: Double).
     */
    fun evaluate(distance: Double, sigma: Double, rssi: Int): Pair<Boolean, Double> {
        recentRanges.addLast(distance)
        while (recentRanges.size > windowSize) recentRanges.removeFirst()

        val skewNlos = recentRanges.size >= 5 && computeSkewness() > skewThreshold
        val rssiNlos = isRssiInconsistent(distance, rssi)

        val isNlos = skewNlos || rssiNlos
        val adjusted = if (isNlos) sigma * nlosInflation else sigma
        return isNlos to adjusted
    }

    /**
     * Fisher-Pearson skewness coefficient:
     *   γ₁ = (1/n) Σ((xᵢ - μ) / σ)³
     */
    internal fun computeSkewness(): Double {
        val n = recentRanges.size
        if (n < 3) return 0.0

        val data = recentRanges.toList()
        val mean = data.sum() / n
        val variance = data.sumOf { (it - mean).pow(2) } / n
        if (variance < 1e-12) return 0.0

        val std = sqrt(variance)
        val m3 = data.sumOf { ((it - mean) / std).pow(3) } / n
        return m3
    }

    /**
     * Check if RSSI is inconsistent with the measured distance.
     *
     * Expected RSSI at distance d: RSSI_ref - 10·n·log₁₀(d)
     * If the measured RSSI deviates too much, the path is likely NLOS.
     */
    internal fun isRssiInconsistent(distance: Double, rssi: Int): Boolean {
        if (distance <= 0.0 || rssi >= 0) return false  // Invalid data, skip check

        val dClamped = distance.coerceAtLeast(0.5)  // Avoid log(0)
        val expectedRssi = rssiRef1m - 10.0 * pathLossExponent * kotlin.math.log10(dClamped)
        return abs(rssi - expectedRssi) > rssiResidualThreshold
    }
}
