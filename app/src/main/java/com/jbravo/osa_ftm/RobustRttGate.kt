package com.jbravo.osa_ftm

import kotlin.math.abs
import kotlin.math.max

data class RttObs(val tMonoMs: Long, val d: Double, val sigma: Double, val rssi: Int)

class RobustRttGate(
    private val windowMs: Long = 1500L,
    private val maxN: Int = 30,
    private val minN: Int = 8,
    private val k: Double = 3.5,
    private val sigmaFloor: Double = 0.12
) {
    private val q = ArrayDeque<RttObs>()

    fun pushAndGate(obs: RttObs): Pair<Boolean, RttObs?> {
        prune(obs.tMonoMs)

        // warm-up: aceptamos hasta tener estadística
        if (q.size < minN) {
            q.addLast(obs); trim()
            return true to obs
        }

        val ds = q.map { it.d }.sorted()
        val med = ds[ds.size / 2]
        val absDev = ds.map { abs(it - med) }.sorted()
        val mad = absDev[absDev.size / 2]
        val sigmaRob = max(1.4826 * mad, sigmaFloor)

        val ok = abs(obs.d - med) <= k * sigmaRob
        if (!ok) return false to null

        val filtered = obs.copy(sigma = sigmaRob) // sustituye std por robust sigma
        q.addLast(filtered); trim()
        return true to filtered
    }

    private fun prune(nowMonoMs: Long) {
        while (q.isNotEmpty() && nowMonoMs - q.first().tMonoMs > windowMs) q.removeFirst()
    }

    private fun trim() {
        while (q.size > maxN) q.removeFirst()
    }
}
