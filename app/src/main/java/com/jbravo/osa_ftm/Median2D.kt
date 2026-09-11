package com.jbravo.osa_ftm

// Mediana del output XY, "cosmética" pero útil

class Median2D(private val maxN: Int = 7) {
    private val q = ArrayDeque<Pair<Double, Double>>()

    fun push(x: Double, y: Double): Pair<Double, Double> {
        q.addLast(x to y)
        while (q.size > maxN) q.removeFirst()

        val xs = q.map { it.first }.sorted()
        val ys = q.map { it.second }.sorted()
        return xs[xs.size / 2] to ys[ys.size / 2]
    }
}
