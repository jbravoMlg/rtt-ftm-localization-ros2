package com.jbravo.osa_ftm

/**
 * Adaptive measurement-noise estimator (Mohamed & Schwarz method).
 *
 * Monitors recent innovations (νₖ = zₖ − h(x̂ₖ⁻)) and updates
 * the measurement noise R to match the actual innovation variance:
 *
 *   R̂ₖ = (1/W) Σ νⱼ² − H P⁻ Hᵀ
 *
 * This allows the EKF to self-tune without manual calibration of R.
 *
 * In practice we use a simplified exponential moving average estimator
 * that is more suitable for real-time applications:
 *
 *   R̂ₖ = (1 − α) R̂ₖ₋₁ + α (νₖ² − H P Hᵀ)
 *
 * A floor is enforced to prevent R from collapsing to zero.
 *
 * @param alpha    EMA factor for R adaptation (0.05 = slow, conservative).
 * @param rFloor   Minimum allowed R value (m²).
 * @param rCeiling Maximum allowed R value (m²) to prevent divergence.
 */
class AdaptiveR(
    private val alpha: Double = 0.05,
    private val rFloor: Double = 0.01,     // (0.1 m)²
    private val rCeiling: Double = 25.0    // (5 m)²
) {
    private var rEstimate: Double = 1.0  // Start at 1 m²

    /**
     * Update the R estimate given the latest innovation.
     *
     * @param innovation νₖ = z − h(x̂⁻) — the measurement residual.
     * @param hpht       HᵀPH — the predicted measurement variance contribution from state.
     * @return The current adaptive R estimate.
     */
    fun update(innovation: Double, hpht: Double): Double {
        // Mohamed & Schwarz: R_new = ν² − HPH^T
        val rSample = innovation * innovation - hpht
        // Clamp the sample to prevent negative R (can happen due to estimation error)
        val rClamped = rSample.coerceIn(rFloor, rCeiling)

        rEstimate = (1.0 - alpha) * rEstimate + alpha * rClamped
        rEstimate = rEstimate.coerceIn(rFloor, rCeiling)

        return rEstimate
    }

    /** Return the current R estimate without updating. */
    fun currentR(): Double = rEstimate
}
