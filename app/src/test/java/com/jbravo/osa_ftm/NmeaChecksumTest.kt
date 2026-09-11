package com.jbravo.osa_ftm

import org.junit.Assert.*
import org.junit.Test

class NmeaChecksumTest {

    @Test
    fun validGgaSentence() {
        // Real GGA with correct checksum
        val sentence = "\$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*51"
        assertTrue(NmeaChecksum.ok(sentence))
    }

    @Test
    fun invalidChecksum() {
        val sentence = "\$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*00"
        assertFalse(NmeaChecksum.ok(sentence))
    }

    @Test
    fun missingAsterisk() {
        val sentence = "\$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,"
        assertFalse(NmeaChecksum.ok(sentence))
    }

    @Test
    fun truncatedChecksum() {
        val sentence = "\$GNGGA,123519*4"
        assertFalse(NmeaChecksum.ok(sentence))
    }

    @Test
    fun emptyString() {
        assertFalse(NmeaChecksum.ok(""))
    }

    @Test
    fun minimalValidSentence() {
        // $GP*checksum — just dollar + data + checksum
        // XOR of 'G' ^ 'P' = 0x47 ^ 0x50 = 0x17
        val sentence = "\$GP*17"
        assertTrue(NmeaChecksum.ok(sentence))
    }
}
