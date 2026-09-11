package com.jbravo.osa_ftm

import org.junit.Assert.*
import org.junit.Test

class ConfigTest {

    @Test
    fun `from() parses full map`() {
        val map = mapOf(
            "host" to "caster.example.com",
            "port" to "2102",
            "mountpoint" to "RTCM3",
            "user" to "testuser",
            "pass" to "testpass",
            "baud" to "460800"
        )
        val cfg = Config.from(map)
        assertEquals("caster.example.com", cfg.ntripHost)
        assertEquals(2102, cfg.ntripPort)
        assertEquals("RTCM3", cfg.mountpoint)
        assertEquals("testuser", cfg.ntripUser)
        assertEquals("testpass", cfg.ntripPass)
        assertEquals(460800, cfg.serialBaud)
    }

    @Test
    fun `from() uses defaults for missing keys`() {
        val cfg = Config.from(emptyMap())
        assertEquals("", cfg.ntripHost)
        assertEquals(2101, cfg.ntripPort)
        assertEquals("", cfg.mountpoint)
        assertEquals("", cfg.ntripUser)
        assertEquals("", cfg.ntripPass)
        assertEquals(115200, cfg.serialBaud)
    }

    @Test
    fun `from() handles invalid port gracefully`() {
        val map = mapOf("port" to "not_a_number")
        val cfg = Config.from(map)
        assertEquals(2101, cfg.ntripPort)
    }

    @Test
    fun `isNtripReady requires host and mountpoint`() {
        assertFalse(Config().isNtripReady)
        assertFalse(Config(ntripHost = "host").isNtripReady)
        assertFalse(Config(mountpoint = "mount").isNtripReady)
        assertTrue(Config(ntripHost = "host", mountpoint = "mount").isNtripReady)
    }

    @Test
    fun `from() with partial map fills missing fields with defaults`() {
        val map = mapOf("host" to "192.168.1.1", "mountpoint" to "VRS")
        val cfg = Config.from(map)
        assertEquals("192.168.1.1", cfg.ntripHost)
        assertEquals("VRS", cfg.mountpoint)
        assertEquals(2101, cfg.ntripPort)
        assertEquals("", cfg.ntripUser)
        assertTrue(cfg.isNtripReady)
    }
}
