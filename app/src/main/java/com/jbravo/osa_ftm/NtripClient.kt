package com.jbravo.osa_ftm

import android.util.Log
import java.io.BufferedInputStream
import java.io.OutputStream
import java.net.HttpURLConnection
import java.net.Socket
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong

/**
 * NTRIP v1 client that connects to a caster, authenticates, and streams
 * RTCM3 correction data.
 *
 * Usage:
 *  1. Create an instance with host/port/mountpoint/credentials.
 *  2. Set [onRtcmData] to receive raw RTCM bytes (forward to F9P serial).
 *  3. Call [start] — runs a background thread that reconnects automatically.
 *  4. Call [sendGga] periodically with the rover's GGA sentence (for VRS).
 *  5. Call [stop] when done.
 */
class NtripClient(
    private val host: String,
    private val port: Int,
    private val mountpoint: String,
    private val user: String,
    private val password: String
) {
    companion object {
        private const val TAG = "NtripClient"
        private const val CONNECT_TIMEOUT_MS = 8_000
        private const val READ_TIMEOUT_MS = 15_000
        private const val RECONNECT_BASE_MS = 2_000L
        private const val RECONNECT_MAX_MS = 30_000L
        private const val READ_BUF_SIZE = 4096
    }

    /** Callback for raw RTCM data chunks received from the caster. */
    var onRtcmData: ((ByteArray, Int) -> Unit)? = null

    /** Callback for status messages (for UI/logging). */
    var onStatus: ((String) -> Unit)? = null

    private val running = AtomicBoolean(false)
    private var workerThread: Thread? = null
    private var currentSocket: Socket? = null

    val totalBytesReceived = AtomicLong(0L)
    val isConnected: Boolean get() = running.get() && currentSocket?.isConnected == true

    @Volatile
    private var lastGga: String? = null

    /**
     * Provide a GGA sentence to be sent to the caster on the next cycle
     * (needed for VRS/nearest-base selection).  The sentence should include
     * the leading '$' and the '*XX' checksum, but no trailing CRLF.
     */
    fun sendGga(gga: String) {
        lastGga = gga
    }

    /**
     * Start the NTRIP connection in a background thread.
     * Automatically reconnects on failure with exponential backoff.
     */
    fun start() {
        if (!running.compareAndSet(false, true)) {
            Log.w(TAG, "Already running")
            return
        }

        workerThread = Thread(::connectLoop, "ntrip-client").apply {
            isDaemon = true
            start()
        }
    }

    /** Stop the NTRIP connection and release resources. */
    fun stop() {
        if (!running.compareAndSet(true, false)) return

        closeSocket()
        workerThread?.interrupt()
        workerThread = null
        onStatus?.invoke("NTRIP stopped")
        Log.i(TAG, "Stopped")
    }

    // -----------------------------------------------------------------------
    //  Internal connection loop
    // -----------------------------------------------------------------------

    private fun connectLoop() {
        var backoffMs = RECONNECT_BASE_MS

        while (running.get()) {
            try {
                onStatus?.invoke("Connecting to $host:$port/$mountpoint…")
                doSession()
                // If doSession returns normally, treat it as a soft disconnect
                backoffMs = RECONNECT_BASE_MS
            } catch (e: InterruptedException) {
                break
            } catch (e: Exception) {
                if (!running.get()) break
                Log.w(TAG, "Connection error: ${e.message}")
                onStatus?.invoke("NTRIP error: ${e.message}")
            }

            if (!running.get()) break

            onStatus?.invoke("Reconnecting in ${backoffMs / 1000}s…")
            try {
                Thread.sleep(backoffMs)
            } catch (_: InterruptedException) {
                break
            }
            backoffMs = (backoffMs * 2).coerceAtMost(RECONNECT_MAX_MS)
        }
    }

    /**
     * Single NTRIP session: connect → authenticate → read RTCM stream.
     * Returns when the stream ends or an error occurs.
     */
    private fun doSession() {
        val socket = Socket()
        currentSocket = socket

        try {
            socket.connect(java.net.InetSocketAddress(host, port), CONNECT_TIMEOUT_MS)
            socket.soTimeout = READ_TIMEOUT_MS

            val out = socket.getOutputStream()

            // --- NTRIP v1 request (HTTP-like) ---
            val auth = android.util.Base64.encodeToString(
                "$user:$password".toByteArray(Charsets.UTF_8),
                android.util.Base64.NO_WRAP
            )

            val request = buildString {
                append("GET /$mountpoint HTTP/1.0\r\n")
                append("User-Agent: NTRIP AndroidRTT/1.0\r\n")
                append("Authorization: Basic $auth\r\n")
                append("Accept: */*\r\n")
                append("\r\n")
            }

            out.write(request.toByteArray(Charsets.US_ASCII))
            out.flush()

            // --- Read response header ---
            val inp = BufferedInputStream(socket.getInputStream(), READ_BUF_SIZE)
            val header = readHttpHeader(inp)

            if (!header.startsWith("ICY 200 OK") &&
                !header.contains("200 OK")
            ) {
                val firstLine = header.lineSequence().firstOrNull() ?: header
                onStatus?.invoke("NTRIP rejected: $firstLine")
                Log.w(TAG, "Caster response: $header")
                return
            }

            onStatus?.invoke("NTRIP connected to $host:$port/$mountpoint")
            Log.i(TAG, "Connected: ${header.lineSequence().first()}")

            // --- Send initial GGA if available ---
            lastGga?.let { gga ->
                sendGgaToServer(out, gga)
            }

            // --- Read RTCM stream ---
            val buf = ByteArray(READ_BUF_SIZE)
            var ggaIntervalMs = 10_000L
            var lastGgaSentMs = System.currentTimeMillis()

            while (running.get()) {
                val n = inp.read(buf)
                if (n < 0) {
                    onStatus?.invoke("NTRIP stream ended (server closed)")
                    break
                }
                if (n > 0) {
                    totalBytesReceived.addAndGet(n.toLong())
                    onRtcmData?.invoke(buf, n)
                }

                // Periodically send GGA for VRS
                val now = System.currentTimeMillis()
                if (now - lastGgaSentMs >= ggaIntervalMs) {
                    lastGga?.let { gga ->
                        sendGgaToServer(out, gga)
                    }
                    lastGgaSentMs = now
                }
            }
        } finally {
            closeSocket()
        }
    }

    private fun sendGgaToServer(out: OutputStream, gga: String) {
        try {
            val line = if (gga.endsWith("\r\n")) gga else "$gga\r\n"
            out.write(line.toByteArray(Charsets.US_ASCII))
            out.flush()
        } catch (e: Exception) {
            Log.w(TAG, "Error sending GGA to caster: ${e.message}")
        }
    }

    /**
     * Read the HTTP-like response header (up to the blank line).
     * The NTRIP v1 caster typically responds with "ICY 200 OK\r\n...\r\n\r\n".
     */
    private fun readHttpHeader(inp: BufferedInputStream): String {
        val sb = StringBuilder()
        var prev = 0
        val maxHeaderBytes = 4096

        while (sb.length < maxHeaderBytes) {
            val b = inp.read()
            if (b < 0) break
            val c = b.toChar()
            sb.append(c)

            // Detect \r\n\r\n (end of headers)
            if (c == '\n' && sb.length >= 4) {
                val tail = sb.substring(sb.length - 4)
                if (tail == "\r\n\r\n") break
            }
            // Also accept \n\n
            if (c == '\n' && prev == '\n'.code) break
            prev = b
        }

        return sb.toString()
    }

    private fun closeSocket() {
        try {
            currentSocket?.close()
        } catch (_: Exception) {
        }
        currentSocket = null
    }
}
