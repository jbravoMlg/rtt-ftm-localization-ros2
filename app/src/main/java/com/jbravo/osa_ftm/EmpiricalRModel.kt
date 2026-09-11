package com.jbravo.osa_ftm

import kotlin.math.max

/**
 * Empirical measurement-noise model for WiFi FTM ranges.
 *
 * Instead of using a flat R = σ², this model captures the observation that
 * FTM error grows with distance and degrades with weak signal:
 *
 *   R(d, RSSI) = σ₀² + α·d² + β·max(0, |RSSI| - rssiRef)²
 *
 * The RSSI term operates in log-domain (dB²) to avoid the exponential
 * blow-up of the previous linear-power formulation at outdoor RSSI levels.
 *
 * Calibrated values:
 *   At d=50m, RSSI=-74dBm: R ≈ 0.09 + 1.0 + 3.5 = 4.6 → σ ≈ 2.1m
 *   At d=20m, RSSI=-50dBm: R ≈ 0.09 + 0.16 + 0.3 = 0.55 → σ ≈ 0.74m
 *
 * @param sigma0Sq   Base variance floor (m²). Even at close range, FTM has
 *                   ~0.3 m noise from clock quantization.
 * @param alpha      Distance-proportional coefficient. Models multipath and
 *                   bandwidth-limited resolution at longer ranges.
 * @param beta       RSSI excess coefficient (dB²→m²). Converts signal
 *                   degradation above rssiRef into variance.
 * @param rssiRef    RSSI magnitude (positive dB value) below which the
 *                   signal is considered "good" and adds no RSSI variance.
 *                   Typical: 40 (i.e. RSSI ≥ -40 dBm adds nothing).
 */
class EmpiricalRModel(
    private val sigma0Sq: Double = 0.09,      // (0.3 m)²
    private val alpha: Double = 0.0004,        // grows with d²
    private val beta: Double = 0.003,          // dB² coefficient
    private val rssiRef: Double = 40.0         // |RSSI| reference (dB)
) {
    /**
     * Compute measurement variance R given measured distance and RSSI.
     *
     * @param distance Measured range in metres (must be > 0).
     * @param rssi     Received signal strength in dBm (negative value).
     * @param gateSigma Robust sigma from the RTT gate (used as a floor).
     * @return R = max(empiricalR, gateSigma²) to never underestimate.
     */
    fun computeR(distance: Double, rssi: Int, gateSigma: Double): Double {
        val d = distance.coerceAtLeast(0.0)
        val rssiExcess = max(0.0, -rssi.toDouble() - rssiRef)
        val empiricalR = sigma0Sq + alpha * d * d + beta * rssiExcess * rssiExcess
        val gateR = gateSigma * gateSigma
        return empiricalR.coerceAtLeast(gateR)
    }
}
