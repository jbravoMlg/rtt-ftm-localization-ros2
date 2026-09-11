package com.jbravo.osa_ftm

import org.junit.Assert.*
import org.junit.Test

class FilterModeTest {

    @Test
    fun fromStringParsesAllModes() {
        for (mode in FilterMode.entries) {
            assertEquals(mode, FilterMode.fromString(mode.name))
        }
    }

    @Test
    fun fromStringIsCaseInsensitive() {
        assertEquals(FilterMode.UKF, FilterMode.fromString("ukf"))
        assertEquals(FilterMode.NLOS, FilterMode.fromString("nlos"))
        assertEquals(FilterMode.ADAPTIVE_R, FilterMode.fromString("adaptive_r"))
    }

    @Test
    fun fromStringFallsBackToBaseline() {
        assertEquals(FilterMode.BASELINE, FilterMode.fromString(""))
        assertEquals(FilterMode.BASELINE, FilterMode.fromString("UNKNOWN"))
    }

    @Test
    fun enumHasSixEntries() {
        assertEquals(6, FilterMode.entries.size)
    }
}
