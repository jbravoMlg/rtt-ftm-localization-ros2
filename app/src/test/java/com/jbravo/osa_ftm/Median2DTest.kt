package com.jbravo.osa_ftm

import org.junit.Assert.*
import org.junit.Test

class Median2DTest {

    @Test
    fun singleElement() {
        val m = Median2D(5)
        val (x, y) = m.push(3.0, 7.0)
        assertEquals(3.0, x, 1e-9)
        assertEquals(7.0, y, 1e-9)
    }

    @Test
    fun medianOfThree() {
        val m = Median2D(5)
        m.push(10.0, 30.0)
        m.push(20.0, 10.0)
        val (x, y) = m.push(5.0, 20.0)
        // sorted x: [5, 10, 20] → median 10
        // sorted y: [10, 20, 30] → median 20
        assertEquals(10.0, x, 1e-9)
        assertEquals(20.0, y, 1e-9)
    }

    @Test
    fun windowEvictsOld() {
        val m = Median2D(3)
        m.push(100.0, 100.0)
        m.push(1.0, 1.0)
        m.push(2.0, 2.0)
        val (x, y) = m.push(3.0, 3.0)
        // Window: [1, 2, 3] for both
        assertEquals(2.0, x, 1e-9)
        assertEquals(2.0, y, 1e-9)
    }
}
