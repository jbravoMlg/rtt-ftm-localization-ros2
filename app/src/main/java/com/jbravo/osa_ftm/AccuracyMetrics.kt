package com.jbravo.osa_ftm

import kotlin.math.abs
import kotlin.math.sqrt

/**
 * Real-time accuracy metrics computed when ground truth is available.
 *
 * Fed by each stable estimate publication in processRttMeasurement.
 * On experiment stop, produces a summary (JSON) for the log files.
 *
 * Metrics (per-AP and aggregate):
 *  - 2D horizontal error (m)
 *  - 3D error (m)
 *  - Vertical error (signed, m)
 *  - Mean / RMSE / Max error
 *  - CEP50 (Circular Error Probable 50%) — median 2D error
 *  - CEP95 — 95th percentile 2D error
 *  - Number of estimates
 */
class AccuracyMetrics {

    /** Single error sample. */
    data class ErrorSample(
        val bssid: String,
        val err2D: Double,   // horizontal error (m)
        val err3D: Double,   // full 3D error (m)
        val errZ: Double,    // signed vertical error (m)
        val tMs: Long        // wall-clock timestamp
    )

    private val samples = mutableListOf<ErrorSample>()

    /** Total samples collected so far. */
    val count: Int get() = samples.size

    /** Add an error sample computed from estimated vs ground-truth position. */
    fun addSample(
        bssid: String,
        estLat: Double, estLon: Double, estAlt: Double,
        gtLat: Double, gtLon: Double, gtAlt: Double,
        tMs: Long
    ) {
        val dLatM = (estLat - gtLat) * DEG_TO_M
        val dLonM = (estLon - gtLon) * DEG_TO_M * kotlin.math.cos(Math.toRadians((estLat + gtLat) / 2.0))
        val dAltM = estAlt - gtAlt

        val err2D = sqrt(dLatM * dLatM + dLonM * dLonM)
        val err3D = sqrt(dLatM * dLatM + dLonM * dLonM + dAltM * dAltM)

        samples.add(ErrorSample(bssid, err2D, err3D, dAltM, tMs))
    }

    /** Clear all collected metrics. */
    fun reset() = samples.clear()

    // -----------------------------------------------------------------------
    //  Aggregate statistics
    // -----------------------------------------------------------------------

    data class Summary(
        val nSamples: Int,
        val nAps: Int,
        val mean2D: Double,
        val rmse2D: Double,
        val max2D: Double,
        val cep50: Double,
        val cep95: Double,
        val mean3D: Double,
        val rmse3D: Double,
        val meanZ: Double,
        val rmseZ: Double,
        val perAp: Map<String, ApSummary>
    )

    data class ApSummary(
        val bssid: String,
        val nSamples: Int,
        val mean2D: Double,
        val rmse2D: Double,
        val cep50: Double,
        val meanZ: Double,
        val rmseZ: Double
    )

    /**
     * Compute aggregate metrics.  Returns null if no samples.
     */
    fun computeSummary(): Summary? {
        if (samples.isEmpty()) return null

        val n = samples.size
        val e2d = samples.map { it.err2D }
        val e3d = samples.map { it.err3D }
        val ez = samples.map { it.errZ }

        val sorted2D = e2d.sorted()

        fun percentile(sorted: List<Double>, p: Double): Double {
            val idx = ((p / 100.0) * (sorted.size - 1)).coerceIn(0.0, (sorted.size - 1).toDouble())
            val lo = idx.toInt()
            val hi = (lo + 1).coerceAtMost(sorted.size - 1)
            val frac = idx - lo
            return sorted[lo] * (1 - frac) + sorted[hi] * frac
        }

        // Per-AP
        val byAp = samples.groupBy { it.bssid }
        val apSummaries = byAp.map { (bssid, apSamples) ->
            val ap2d = apSamples.map { it.err2D }
            val apZ = apSamples.map { it.errZ }
            val sorted = ap2d.sorted()
            bssid to ApSummary(
                bssid = bssid,
                nSamples = apSamples.size,
                mean2D = ap2d.average(),
                rmse2D = sqrt(ap2d.sumOf { it * it } / ap2d.size),
                cep50 = percentile(sorted, 50.0),
                meanZ = apZ.average(),
                rmseZ = sqrt(apZ.sumOf { it * it } / apZ.size)
            )
        }.toMap()

        return Summary(
            nSamples = n,
            nAps = byAp.size,
            mean2D = e2d.average(),
            rmse2D = sqrt(e2d.sumOf { it * it } / n),
            max2D = e2d.max(),
            cep50 = percentile(sorted2D, 50.0),
            cep95 = percentile(sorted2D, 95.0),
            mean3D = e3d.average(),
            rmse3D = sqrt(e3d.sumOf { it * it } / n),
            meanZ = ez.average(),
            rmseZ = sqrt(ez.sumOf { it * it } / n),
            perAp = apSummaries
        )
    }

    /** Build a JSON string from the summary for logging. */
    fun summaryToJson(
        expId: String,
        expLabel: String,
        aglMeters: Double,
        agentId: String = "",
        peerAgentId: String = "",
        cooperativeMode: String = "independent",
        geomSpread2DM: Double = Double.NaN,
        geomSpread3DM: Double = Double.NaN,
        azimuthCoverageDeg: Double = Double.NaN,
        perAgent: Map<String, AgentStats> = emptyMap()
    ): String? {
        val s = computeSummary() ?: return null

        val apJsonParts = s.perAp.values.joinToString(",") { ap ->
            """{"bssid":"${ap.bssid}","n":${ap.nSamples},"mean2d":${"%.3f".format(ap.mean2D)},"rmse2d":${"%.3f".format(ap.rmse2D)},"cep50":${"%.3f".format(ap.cep50)},"mean_z":${"%.3f".format(ap.meanZ)},"rmse_z":${"%.3f".format(ap.rmseZ)}}"""
        }

        val perAgentJson = perAgent.values.joinToString(",") { a ->
            """{"agent_id":"${a.agentId}","n_rtt":${a.nRtt},"mean_sigma_used":${"%.3f".format(a.meanSigmaUsed)},"gnss_acc_mean":${"%.3f".format(a.gnssAccMean)},"z_spread_m":${"%.3f".format(a.zSpreadM)}}"""
        }

        fun numOrNull(v: Double): String = if (v.isFinite()) "%.3f".format(v) else "null"

        return buildString {
            append('{')
            append("\"type\":\"experiment_summary\"")
            append(",\"exp_id\":\"$expId\"")
            append(",\"label\":\"$expLabel\"")
            append(",\"agent_id\":\"$agentId\"")
            append(",\"peer_agent_id\":\"$peerAgentId\"")
            append(",\"cooperative_mode\":\"$cooperativeMode\"")
            append(",\"agl_m\":$aglMeters")
            append(",\"geom_spread_2d_m\":${numOrNull(geomSpread2DM)}")
            append(",\"geom_spread_3d_m\":${numOrNull(geomSpread3DM)}")
            append(",\"azimuth_coverage_deg\":${numOrNull(azimuthCoverageDeg)}")
            append(",\"z_observability_ok\":${
                if (geomSpread3DM.isFinite())
                    (geomSpread3DM >= 6.0).toString()
                else "null"
            }")
            append(",\"n_samples\":${s.nSamples}")
            append(",\"n_aps\":${s.nAps}")
            append(",\"mean_2d_m\":${"%.3f".format(s.mean2D)}")
            append(",\"rmse_2d_m\":${"%.3f".format(s.rmse2D)}")
            append(",\"max_2d_m\":${"%.3f".format(s.max2D)}")
            append(",\"cep50_m\":${"%.3f".format(s.cep50)}")
            append(",\"cep95_m\":${"%.3f".format(s.cep95)}")
            append(",\"mean_3d_m\":${"%.3f".format(s.mean3D)}")
            append(",\"rmse_3d_m\":${"%.3f".format(s.rmse3D)}")
            append(",\"mean_z_m\":${"%.3f".format(s.meanZ)}")
            append(",\"rmse_z_m\":${"%.3f".format(s.rmseZ)}")
            append(",\"per_ap\":[$apJsonParts]")
            append(",\"per_agent\":[$perAgentJson]")
            append(",\"t_ms\":${System.currentTimeMillis()}")
            append('}')
        }
    }

    /** Aggregate diagnostics per agent (for cooperative experiments). */
    data class AgentStats(
        val agentId: String,
        val nRtt: Long,
        val meanSigmaUsed: Double,
        val gnssAccMean: Double,
        val zSpreadM: Double
    )

    companion object {
        private const val DEG_TO_M = 111_319.49 // approx metres per degree latitude
    }
}
