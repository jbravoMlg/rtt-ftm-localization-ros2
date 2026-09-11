package com.jbravo.osa_ftm

import org.junit.Assert.*
import org.junit.Test

class GroundTruthRegistryTest {

    @Test
    fun `put and get AP truth`() {
        val r = GroundTruthRegistry()
        val ap = GroundTruthRegistry.ApTruth("AA:BB:CC:DD:EE:FF", 36.72, -4.42, 42.0, "surface")
        r.put(ap)

        val got = r.get("aa:bb:cc:dd:ee:ff")
        assertNotNull(got)
        assertEquals(36.72, got!!.lat, 1e-8)
        assertEquals(-4.42, got.lon, 1e-8)
        assertEquals(42.0, got.alt, 1e-8)
        assertEquals("surface", got.description)
    }

    @Test
    fun `get returns null for unknown bssid`() {
        val r = GroundTruthRegistry()
        assertNull(r.get("aa:bb:cc:dd:ee:ff"))
    }

    @Test
    fun `put overwrites existing entry`() {
        val r = GroundTruthRegistry()
        r.put(GroundTruthRegistry.ApTruth("aa:bb:cc:dd:ee:ff", 36.72, -4.42, 42.0))
        r.put(GroundTruthRegistry.ApTruth("aa:bb:cc:dd:ee:ff", 36.73, -4.43, 43.0))

        assertEquals(1, r.size)
        assertEquals(36.73, r.get("aa:bb:cc:dd:ee:ff")!!.lat, 1e-8)
    }

    @Test
    fun `remove deletes entry`() {
        val r = GroundTruthRegistry()
        r.put(GroundTruthRegistry.ApTruth("aa:bb:cc:dd:ee:ff", 36.72, -4.42, 42.0))
        assertEquals(1, r.size)

        r.remove("AA:BB:CC:DD:EE:FF")
        assertEquals(0, r.size)
        assertNull(r.get("aa:bb:cc:dd:ee:ff"))
    }

    @Test
    fun `all returns snapshot list`() {
        val r = GroundTruthRegistry()
        r.put(GroundTruthRegistry.ApTruth("aa:bb:cc:dd:ee:01", 36.72, -4.42, 42.0))
        r.put(GroundTruthRegistry.ApTruth("aa:bb:cc:dd:ee:02", 36.73, -4.43, 43.0))

        val all = r.all()
        assertEquals(2, all.size)
    }

    @Test
    fun `clear removes all entries`() {
        val r = GroundTruthRegistry()
        r.put(GroundTruthRegistry.ApTruth("aa:bb:cc:dd:ee:01", 36.72, -4.42, 42.0))
        r.put(GroundTruthRegistry.ApTruth("aa:bb:cc:dd:ee:02", 36.73, -4.43, 43.0))
        assertEquals(2, r.size)

        r.clear()
        assertEquals(0, r.size)
    }

    @Test
    fun `bssid lookup is case-insensitive`() {
        val r = GroundTruthRegistry()
        r.put(GroundTruthRegistry.ApTruth("AA:BB:CC:DD:EE:FF", 36.72, -4.42, 42.0))

        assertNotNull(r.get("aa:bb:cc:dd:ee:ff"))
        assertNotNull(r.get("AA:BB:CC:DD:EE:FF"))
        assertNotNull(r.get("Aa:Bb:Cc:Dd:Ee:Ff"))
    }
}
