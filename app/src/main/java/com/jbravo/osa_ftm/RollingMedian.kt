package com.jbravo.osa_ftm

class RollingMedian(private val maxN: Int = 25) {
    private val q = ArrayDeque<Double>()

    fun push(v: Double) {
        q.addLast(v)
        while (q.size > maxN) q.removeFirst()
    }

    fun medianOrNull(): Double? {
        if (q.isEmpty()) return null
        val s = q.toList().sorted()
        return s[s.size / 2]
    }
}
