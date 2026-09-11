package com.jbravo.osa_ftm

/**
 * Exponential Moving Average for 2D output smoothing.
 *
 * Unlike Median2D which introduces ~N/2 samples of latency,
 * EMA responds immediately to each new estimate with controlled smoothing:
 *   x̂ₖ = α · xₖ + (1 − α) · x̂ₖ₋₁
 *
 * @param alpha Smoothing factor in (0, 1]. Higher = less smoothing.
 *              0.3 is a good balance for ~4 Hz FTM measurements.
 */
class EmaSmooth2D(private val alpha: Double = 0.3) {
    private var xSmooth: Double? = null
    private var ySmooth: Double? = null

    fun push(x: Double, y: Double): Pair<Double, Double> {
        val xs = xSmooth
        val ys = ySmooth
        return if (xs == null || ys == null) {
            xSmooth = x
            ySmooth = y
            x to y
        } else {
            val nx = alpha * x + (1.0 - alpha) * xs
            val ny = alpha * y + (1.0 - alpha) * ys
            xSmooth = nx
            ySmooth = ny
            nx to ny
        }
    }
}
