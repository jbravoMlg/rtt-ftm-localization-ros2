package com.jbravo.osa_ftm

import android.content.Context

/**
 * Pre-flight experiment configuration.
 *
 * The operator fills this in before each flight — altitude, label,
 * known AP ground-truth positions, and experiment metadata.
 *
 * Persisted to SharedPreferences so the last config is restored on app restart.
 *
 * Usage from ADB:
 *   adb shell am broadcast -a com.jbravo.osa_ftm.EXPERIMENT_CONFIG \
 *       --es label "flight_30m_run2" --es agl "30.0" --es filter "UKF"
 */
data class ExperimentConfig(
    /** Human-readable experiment label, e.g. "flight_30m_run1". */
    val label: String = "",

    /**
     * Drone flight altitude AGL in metres for this experiment.
     *
     * Only consulted when [altitudeMode] == [ALT_MODE_MANUAL_AGL]. In
     * [ALT_MODE_AUTO] mode the anchor Z is estimated purely from the
     * vertical diversity of the drone trajectory, and this value is
     * ignored.
     */
    val aglMeters: Double = 1.5,

    /**
     * Altitude handling mode. See [ALT_MODE_AUTO] (default) and
     * [ALT_MODE_MANUAL_AGL].
     *
     * AUTO: no AGL seed is required. The anchor Z is recovered from
     * the natural vertical excursion of one or more drones. Recommended
     * for multi-drone cooperative experiments where the trajectories
     * span a few metres of altitude.
     *
     * MANUAL_AGL: legacy mode. The operator provides [aglMeters] and
     * the estimator uses it both as a range-to-horizontal-distance
     * correction and as a loose Z prior for the filters.
     */
    val altitudeMode: String = ALT_MODE_AUTO,

    /** Optional notes (weather, AP deployment, etc.). */
    val notes: String = "",

    /** Filtering/estimation pipeline mode. See [FilterMode]. */
    val filterMode: FilterMode = FilterMode.BASELINE,

    /**
     * RTT range bias correction in metres.
     *
     * Systematic offset subtracted from every raw RTT distance before
     * gating, bootstrap, and EKF update.  Positive value means the
     * hardware over-reports distance (e.g. +1.5 m observed in calibration).
     */
    val rangeBiasM: Double = 0.0,

    // ------------------------------------------------------------------
    //  Multi-agent / cooperative OSA fields
    // ------------------------------------------------------------------

    /**
     * Explicit agent identifier (e.g. "osa1", "osa2").
     *
     * If non-empty, it is used as the ROS 2 namespace prefix and as the
     * "agent_id" field in every JSONL record.  If empty, the app falls
     * back to a device-derived namespace (model + UUID).
     *
     * NOTE: ROS 2 topic names are fixed at node creation, so changing
     * agentId at runtime only affects JSONL logs until the app is
     * restarted.
     */
    val agentId: String = "",

    /**
     * Optional peer agent identifier used for cooperative experiments.
     *
     * When set AND [cooperativeMode] != "independent", the app subscribes
     * to /<peerAgentId>/phone/location and /<peerAgentId>/ftm_rtt so that
     * both drones' measurements can be fused either on-board or off-board.
     */
    val peerAgentId: String = "",

    /**
     * Cooperative fusion mode. See [Companion.COOP_INDEPENDENT],
     * [Companion.COOP_FUSED_OFFBOARD], [Companion.COOP_FUSED_ONBOARD].
     */
    val cooperativeMode: String = "independent",

    /**
     * Minimum period between RTT ranging bursts, in ms.
     * Use ≥120 ms combined with a per-agent [rangingPhaseMs] offset to avoid
     * FTM scheduler collisions between two OSAs ranging the same AP.
     */
    val rangingPeriodMs: Long = 120L,

    /**
     * Phase offset applied to the first RTT burst after start, in ms.
     * Typical pattern: OSA1 phase=0, OSA2 phase=60.
     */
    val rangingPhaseMs: Long = 0L
) {
    companion object {
        private const val PREFS = "experiment_config"

        const val COOP_INDEPENDENT = "independent"
        const val COOP_FUSED_OFFBOARD = "fused_offboard"
        const val COOP_FUSED_ONBOARD = "fused_onboard"

        const val ALT_MODE_AUTO = "auto"
        const val ALT_MODE_MANUAL_AGL = "manual_agl"

        fun sanitizeCoopMode(s: String?): String = when ((s ?: "").trim().lowercase()) {
            COOP_FUSED_OFFBOARD -> COOP_FUSED_OFFBOARD
            COOP_FUSED_ONBOARD -> COOP_FUSED_ONBOARD
            else -> COOP_INDEPENDENT
        }

        fun sanitizeAltitudeMode(s: String?): String = when ((s ?: "").trim().lowercase()) {
            ALT_MODE_MANUAL_AGL, "manual", "agl" -> ALT_MODE_MANUAL_AGL
            else -> ALT_MODE_AUTO
        }

        /** Lower-case, alphanumeric+underscore, max 32 chars. Empty if invalid. */
        fun sanitizeAgentId(s: String?): String {
            val t = (s ?: "").trim().lowercase().replace("[^a-z0-9_]".toRegex(), "_").trim('_')
            return if (t.isBlank()) "" else t.take(32)
        }

        fun loadFromPrefs(ctx: Context): ExperimentConfig {
            val p = ctx.getSharedPreferences(PREFS, Context.MODE_PRIVATE)
            return ExperimentConfig(
                label = p.getString("label", "") ?: "",
                aglMeters = p.getFloat("agl", 1.5f).toDouble(),
                notes = p.getString("notes", "") ?: "",
                filterMode = FilterMode.fromString(p.getString("filter", "BASELINE") ?: "BASELINE"),
                rangeBiasM = p.getFloat("range_bias", 0.0f).toDouble(),
                agentId = p.getString("agent_id", "") ?: "",
                peerAgentId = p.getString("peer_agent_id", "") ?: "",
                cooperativeMode = sanitizeCoopMode(p.getString("cooperative_mode", COOP_INDEPENDENT)),
                rangingPeriodMs = p.getLong("ranging_period_ms", 120L),
                rangingPhaseMs = p.getLong("ranging_phase_ms", 0L),
                altitudeMode = sanitizeAltitudeMode(p.getString("altitude_mode", ALT_MODE_AUTO))
            )
        }

        fun saveToPrefs(ctx: Context, cfg: ExperimentConfig) {
            ctx.getSharedPreferences(PREFS, Context.MODE_PRIVATE).edit()
                .putString("label", cfg.label)
                .putFloat("agl", cfg.aglMeters.toFloat())
                .putString("notes", cfg.notes)
                .putString("filter", cfg.filterMode.name)
                .putFloat("range_bias", cfg.rangeBiasM.toFloat())
                .putString("agent_id", cfg.agentId)
                .putString("peer_agent_id", cfg.peerAgentId)
                .putString("cooperative_mode", cfg.cooperativeMode)
                .putLong("ranging_period_ms", cfg.rangingPeriodMs)
                .putLong("ranging_phase_ms", cfg.rangingPhaseMs)
                .putString("altitude_mode", cfg.altitudeMode)
                .apply()
        }
    }
}
