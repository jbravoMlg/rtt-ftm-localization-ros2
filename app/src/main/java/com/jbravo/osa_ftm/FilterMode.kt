package com.jbravo.osa_ftm

/**
 * Selectable filtering/estimation pipeline modes for pre-flight configuration.
 *
 * Each mode changes exactly ONE aspect from the BASELINE, enabling clean A/B comparison.
 *
 * | Mode         | Gate               | R model              | Estimator | Output      |
 * |--------------|--------------------|----------------------|-----------|-------------|
 * | BASELINE     | MAD                | σ²                   | IEKF3D    | Median2D(7) |
 * | NLOS         | MAD + skewness/RSSI| σ² (inflated on NLOS)| IEKF3D    | Median2D(7) |
 * | EMPIRICAL_R  | MAD                | f(d, RSSI)           | IEKF3D    | Median2D(7) |
 * | UKF          | MAD                | σ²                   | UKF3D     | Median2D(7) |
 * | EMA          | MAD                | σ²                   | IEKF3D    | EMA(0.3)    |
 * | ADAPTIVE_R   | MAD                | adaptive (Mohamed)   | IEKF3D    | Median2D(7) |
 */
enum class FilterMode(val label: String) {
    BASELINE("Baseline (IEKF3D + Median)"),
    NLOS("NLOS detection (skewness + RSSI)"),
    EMPIRICAL_R("Empirical R(d, RSSI)"),
    UKF("UKF3D (Unscented Kalman)"),
    EMA("EMA output (α=0.3)"),
    ADAPTIVE_R("Adaptive R (Mohamed & Schwarz)");

    companion object {
        fun fromString(s: String): FilterMode =
            entries.firstOrNull { it.name.equals(s, ignoreCase = true) } ?: BASELINE
    }
}
