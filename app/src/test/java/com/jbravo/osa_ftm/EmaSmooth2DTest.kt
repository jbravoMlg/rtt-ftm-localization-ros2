package com.jbravo.osa_ftm

import org.junit.Assert.*
import org.junit.Test

class EmaSmooth2DTest {

    @Test
    fun firstPushReturnsInput() {
        val ema = EmaSmooth2D(alpha = 0.3)
        val (x, y) = ema.push(10.0, 20.0)
        assertEquals(10.0, x, 1e-9)
        assertEquals(20.0, y, 1e-9)
    }

    @Test
    fun smoothsTowardsNewValue() {
        val ema = EmaSmooth2D(alpha = 0.3)
        ema.push(0.0, 0.0)
        val (x, y) = ema.push(10.0, 20.0)
        // 0.3 * 10 + 0.7 * 0 = 3.0
        assertEquals(3.0, x, 1e-9)
        assertEquals(6.0, y, 1e-9)
    }

    @Test
    fun convergesWithRepeatedInput() {
        val ema = EmaSmooth2D(alpha = 0.3)
        ema.push(0.0, 0.0)
        var x = 0.0
        var y = 0.0
        repeat(100) {
            val (nx, ny) = ema.push(10.0, 20.0)
            x = nx; y = ny
        }
        assertEquals(10.0, x, 0.01)
        assertEquals(20.0, y, 0.01)
    }

    @Test
    fun alphaOnePassesThrough() {
        val ema = EmaSmooth2D(alpha = 1.0)
        ema.push(0.0, 0.0)
        val (x, y) = ema.push(5.0, 7.0)
        assertEquals(5.0, x, 1e-9)
        assertEquals(7.0, y, 1e-9)
    }
}
