package com.jbravo.osa_ftm

import org.junit.Assert.*
import org.junit.Test

class NtripClientTest {

    @Test
    fun `sendGga stores sentence for next cycle`() {
        val client = NtripClient("host", 2101, "mount", "u", "p")
        // Initially no GGA
        client.sendGga("\$GPGGA,120000.00,3600.00000,N,00200.00000,E,4,12,0.9,100.0,M,47.0,M,,*XX")
        // The client stores it for sending — no crash expected
    }

    @Test
    fun `isConnected false before start`() {
        val client = NtripClient("host", 2101, "mount", "u", "p")
        assertFalse(client.isConnected)
    }

    @Test
    fun `totalBytesReceived starts at zero`() {
        val client = NtripClient("host", 2101, "mount", "u", "p")
        assertEquals(0L, client.totalBytesReceived.get())
    }

    @Test
    fun `stop on non-started client does not crash`() {
        val client = NtripClient("host", 2101, "mount", "u", "p")
        client.stop()  // should be a no-op
        assertFalse(client.isConnected)
    }

    @Test
    fun `callbacks can be set`() {
        val client = NtripClient("host", 2101, "mount", "u", "p")
        var statusMsg = ""
        client.onStatus = { statusMsg = it }
        client.onRtcmData = { _, _ -> }
        // Just checking no crash
        assertNotNull(client.onStatus)
        assertNotNull(client.onRtcmData)
    }
}
