package com.jbravo.osa_ftm

import org.junit.Assert.*
import org.junit.Test

class NmeaParserTest {

    @Test
    fun latitudeNorth() {
        // 4807.038 N → 48 + 7.038/60 = 48.1173
        val lat = NmeaParser.latitude("4807.038", "N")
        assertEquals(48.1173, lat, 0.001)
    }

    @Test
    fun latitudeSouth() {
        val lat = NmeaParser.latitude("4807.038", "S")
        assertEquals(-48.1173, lat, 0.001)
    }

    @Test
    fun longitudeEast() {
        // 01131.000 E → 11 + 31.0/60 = 11.51667
        val lon = NmeaParser.longitude("01131.000", "E")
        assertEquals(11.51667, lon, 0.001)
    }

    @Test
    fun longitudeWest() {
        val lon = NmeaParser.longitude("01131.000", "W")
        assertEquals(-11.51667, lon, 0.001)
    }

    @Test
    fun nullInputReturnsZero() {
        assertEquals(0.0, NmeaParser.latitude(null, "N"), 1e-9)
        assertEquals(0.0, NmeaParser.longitude(null, "E"), 1e-9)
    }

    @Test
    fun shortInputReturnsZero() {
        assertEquals(0.0, NmeaParser.latitude("48", "N"), 1e-9)
        assertEquals(0.0, NmeaParser.longitude("011", "E"), 1e-9)
    }

    @Test
    fun highPrecisionCoordinate() {
        // 4002.82810 N → 40 + 2.82810/60 = 40.04713500
        val lat = NmeaParser.latitude("4002.82810", "N")
        assertEquals(40.04713500, lat, 1e-5)
    }
}
