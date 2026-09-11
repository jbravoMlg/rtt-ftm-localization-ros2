package com.jbravo.osa_ftm

import org.junit.Assert.*
import org.junit.Test

class RollingMedianTest {

    @Test
    fun singleElement() {
        val rm = RollingMedian(5)
        rm.push(7.0)
        assertEquals(7.0, rm.medianOrNull()!!, 1e-9)
    }

    @Test
    fun oddCount() {
        val rm = RollingMedian(5)
        rm.push(3.0)
        rm.push(1.0)
        rm.push(2.0)
        // sorted: [1, 2, 3] → median index 1 → 2.0
        assertEquals(2.0, rm.medianOrNull()!!, 1e-9)
    }

    @Test
    fun evenCount() {
        val rm = RollingMedian(5)
        rm.push(4.0)
        rm.push(1.0)
        rm.push(3.0)
        rm.push(2.0)
        // sorted: [1, 2, 3, 4] → index 2 → 3.0 (integer division)
        assertEquals(3.0, rm.medianOrNull()!!, 1e-9)
    }

    @Test
    fun evictsOldSamples() {
        val rm = RollingMedian(3)
        rm.push(10.0)
        rm.push(20.0)
        rm.push(30.0)
        rm.push(1.0)
        // Window: [20, 30, 1] → sorted [1, 20, 30] → median = 20
        assertEquals(20.0, rm.medianOrNull()!!, 1e-9)
    }

    @Test
    fun emptyReturnsNull() {
        val rm = RollingMedian(5)
        assertNull(rm.medianOrNull())
    }
}
