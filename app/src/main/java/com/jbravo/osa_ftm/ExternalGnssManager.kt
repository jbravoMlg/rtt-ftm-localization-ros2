package com.jbravo.osa_ftm

import android.content.Context
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbDeviceConnection
import android.hardware.usb.UsbManager
import android.util.Log
import com.hoho.android.usbserial.driver.UsbSerialDriver
import com.hoho.android.usbserial.driver.UsbSerialPort
import com.hoho.android.usbserial.driver.UsbSerialProber
import com.hoho.android.usbserial.util.SerialInputOutputManager
import java.util.concurrent.Executors
import java.util.concurrent.Future
import java.util.concurrent.atomic.AtomicBoolean

/**
 * Gestiona el GNSS externo por USB (u-blox F9P, etc.).
 *
 * - Abre el puerto serie a 115200.
 * - Envía UBX-CFG-RATE para ponerlo a 10 Hz.
 * - Lee NMEA, valida checksum y parsea GGA.
 * - Para cada fix, llama a [onFix].
 * - Informa de estado vía [onStatus] y [onActiveChanged].
 *
 * NTRIP: el parámetro [ntripConfig] se deja preparado pero no se usa todavía.
 */
class ExternalGnssManager(
    private val context: Context,
    private val onFix: (lat: Double, lon: Double, alt: Double, accM: Double) -> Unit,
    private val onStatus: (String) -> Unit,
    private val onActiveChanged: (Boolean) -> Unit,
    private var ntripConfig: Config? = null
) {

    private val usbManager = context.getSystemService(Context.USB_SERVICE) as UsbManager

    private var deviceConnection: UsbDeviceConnection? = null
    private var serialPort: UsbSerialPort? = null
    private var ioManager: SerialInputOutputManager? = null
    private var ioFuture: Future<*>? = null

    private val executor = Executors.newSingleThreadExecutor()
    private val started = AtomicBoolean(false)

    // NTRIP client (created after serial port is open, if config is present)
    private var ntripClient: NtripClient? = null
    // Lock for writing RTCM to serial (shared with NMEA reader)
    private val serialWriteLock = Any()

    val isRunning: Boolean
        get() = started.get()

    val isNtripConnected: Boolean
        get() = ntripClient?.isConnected == true

    // Observable GNSS/RTK status (updated from the NMEA listener, read from UI)
    @Volatile var lastFixQuality: Int = -1
        private set
    @Volatile var lastSatelliteCount: Int = -1
        private set
    @Volatile var lastHdop: Double = -1.0
        private set

    /** Cumulative RTCM bytes received from the NTRIP caster. */
    val rtcmBytesReceived: Long
        get() = ntripClient?.totalBytesReceived?.get() ?: 0L

    /**
     * Intenta arrancar el GNSS externo.
     * Si no hay dispositivo o no hay permisos, sólo loguea y devuelve.
     */
    fun start(preferredDevice: UsbDevice? = null) {
        if (!started.compareAndSet(false, true)) {
            Log.w(TAG, "External GNSS ya estaba iniciado")
            return
        }

        try {
            val driver = findPreferredDriver(preferredDevice)
            if (driver == null) {
                Log.w(TAG, "No se ha encontrado ningún dispositivo USB-serial compatible")
                onStatus("No USB-serial device found")
                started.set(false)
                onActiveChanged(false)
                return
            }

            val device = driver.device

            // 1) Comprobar permiso antes de abrir
            if (!usbManager.hasPermission(device)) {
                Log.w(TAG, "No permission for USB GNSS device")
                onStatus("No permission for USB GNSS device; call requestUsbPermissionForGnss first")
                started.set(false)
                onActiveChanged(false)
                return
            }

            // 2) Abrir conexión
            val connection = usbManager.openDevice(device)
            if (connection == null) {
                Log.e(TAG, "No se pudo abrir la conexión USB (¿falta permiso?)")
                onStatus("Cannot open USB device (missing permission?)")
                started.set(false)
                onActiveChanged(false)
                return
            }

            deviceConnection = connection

            // 3) Obtener puerto serie
            val port = driver.ports.firstOrNull()
            if (port == null) {
                Log.e(TAG, "El driver USB-serial no tiene puertos disponibles")
                onStatus("USB-serial driver has no ports")
                safeCloseConnection()
                started.set(false)
                onActiveChanged(false)
                return
            }

            serialPort = port

            // 4) Configurar puerto
            port.open(connection)
            port.setParameters(
                115200,
                8,
                UsbSerialPort.STOPBITS_1,
                UsbSerialPort.PARITY_NONE
            )
            port.dtr = true
            port.rts = true

            // 5) Enviar UBX para poner el F9P a 10 Hz
            try {
                Log.i(TAG, "Enviando configuración F9P 10 Hz (UBX-CFG-RATE + CFG-MSG GGA)...")
                UbxConfigSender.configureF9PFor10Hz(port)
                onStatus("UBX-CFG-RATE 10 Hz enviado al F9P")
            } catch (e: Exception) {
                Log.w(TAG, "Error enviando UBX-CFG-RATE 10 Hz", e)
                onStatus("Error enviando UBX-CFG-RATE 10 Hz: ${e.message}")
            }


            // 6) Listener NMEA
            val listener = object : SerialInputOutputManager.Listener {
                private val lineBuffer = StringBuilder()

                // --- Stats RX (para depurar 10 Hz aunque no haya fix) ---
                private var bytesAcc = 0L
                private var nmeaLinesAcc = 0
                private var ggaLinesAcc = 0
                private var lastReportMs = System.currentTimeMillis()
                private var lastFixQ: String? = null
                private var lastNumSv: String? = null
                // ^ used for log formatting; canonical values go to class-level fields

                private fun maybeReport() {
                    val now = System.currentTimeMillis()
                    if (now - lastReportMs >= 1_000L) {
                        val dt = (now - lastReportMs).coerceAtLeast(1L).toDouble()

                        val bytesPerSec = bytesAcc * 1000.0 / dt
                        val nmeaHz = nmeaLinesAcc * 1000.0 / dt
                        val ggaHz  = ggaLinesAcc * 1000.0 / dt

                        onStatus(
                            "USB RX: ${"%.0f".format(bytesPerSec)} B/s | " +
                                    "NMEA ${"%.1f".format(nmeaHz)} Hz | " +
                                    "GGA ${"%.1f".format(ggaHz)} Hz | " +
                                    "fixQ=${lastFixQ ?: "?"} nSV=${lastNumSv ?: "?"}"
                        )

                        bytesAcc = 0
                        nmeaLinesAcc = 0
                        ggaLinesAcc = 0
                        lastReportMs = now
                    }
                }

                override fun onNewData(data: ByteArray) {
                    bytesAcc += data.size.toLong()

                    val text = String(data, Charsets.US_ASCII)
                    synchronized(lineBuffer) {
                        lineBuffer.append(text)
                        var newLineIndex = lineBuffer.indexOf("\n")
                        while (newLineIndex >= 0) {
                            val rawLine = lineBuffer.substring(0, newLineIndex)
                            val line = rawLine.trim('\r', '\n', ' ')
                            lineBuffer.delete(0, newLineIndex + 1)
                            newLineIndex = lineBuffer.indexOf("\n")

                            if (line.isEmpty()) continue
                            nmeaLinesAcc++

                            // Cuenta GGA válidos (checksum OK) aunque no tengan fix
                            try {
                                if (line.startsWith('$') && NmeaChecksum.ok(line)) {
                                    val starIdx = line.indexOf('*')
                                    val noChecksum = if (starIdx > 0) line.substring(0, starIdx) else line
                                    val parts = noChecksum.split(',')
                                    if (parts.isNotEmpty() && parts[0].endsWith("GGA", ignoreCase = true)) {
                                        ggaLinesAcc++
                                        lastFixQ = parts.getOrNull(6)
                                        lastNumSv = parts.getOrNull(7)
                                        lastFixQuality = lastFixQ?.toIntOrNull() ?: -1
                                        lastSatelliteCount = lastNumSv?.toIntOrNull() ?: -1
                                    }
                                }
                            } catch (_: Throwable) {}

                            // Tu procesamiento normal
                            try {
                                handleNmeaLine(line)
                            } catch (e: Exception) {
                                Log.e(TAG, "Error procesando sentencia NMEA: $line", e)
                            } finally {
                                maybeReport()
                            }
                        }
                    }
                }

                override fun onRunError(e: Exception) {
                    Log.e(TAG, "Error en SerialInputOutputManager", e)
                    onStatus("Serial IO error: ${e.message}")
                    stop()
                }
            }

            ioManager = SerialInputOutputManager(port, listener)
            ioFuture = executor.submit(ioManager)

            Log.i(TAG, "External GNSS iniciado y leyendo datos")
            onStatus("External GNSS started")
            onActiveChanged(true)

            // Start NTRIP if configured — forward RTCM corrections to F9P
            startNtripIfConfigured(port)

        } catch (se: SecurityException) {
            Log.e(TAG, "Permiso denegado para acceder al dispositivo USB GNSS", se)
            onStatus("USB permission denied for GNSS: ${se.message}")
            started.set(false)
            safeCloseAll()
            onActiveChanged(false)
        } catch (t: Throwable) {
            Log.e(TAG, "Error inesperado iniciando GNSS externo", t)
            onStatus("Unexpected error starting GNSS: ${t.message}")
            started.set(false)
            safeCloseAll()
            onActiveChanged(false)
        }
    }


    /**
     * Detiene la lectura, NTRIP y cierra puerto y conexión USB.
     */
    fun stop() {
        if (!started.compareAndSet(true, false)) {
            return
        }

        onActiveChanged(false)
        onStatus("External GNSS stopped")

        // Stop NTRIP first
        stopNtrip()

        // Paramos IO manager
        try {
            ioManager?.stop()
        } catch (e: Exception) {
            Log.w(TAG, "Error al parar SerialInputOutputManager", e)
        }
        ioManager = null

        // Cancelamos el futuro/hilo
        try {
            ioFuture?.cancel(true)
        } catch (e: Exception) {
            Log.w(TAG, "Error al cancelar el hilo de IO", e)
        }
        ioFuture = null

        // Cerramos puerto y conexión
        safeCloseAll()

        Log.i(TAG, "External GNSS detenido")
    }

    // -------------------------------------------------------------------------
    //  NMEA
    // -------------------------------------------------------------------------

    private fun handleNmeaLine(line: String) {
        if (!line.startsWith('$')) return
        if (!NmeaChecksum.ok(line)) return

        // Quitamos checksum para parsear campos
        val starIdx = line.indexOf('*')
        val noChecksum = if (starIdx > 0) line.substring(0, starIdx) else line
        val parts = noChecksum.split(',')

        if (parts.isEmpty()) return

        when {
            parts[0].endsWith("GGA", ignoreCase = true) -> {
                Log.d("GGA_RAW", line)
                parseGga(parts)
                // Forward raw GGA to NTRIP caster for VRS
                sendGgaToCaster(line)
            }
            // Si quieres velocidad/heading, puedes añadir parseVTG(parts)
        }
    }



    /**
     * GGA: lat, lon, alt (WGS84 ellipsoidal), hdop -> onFix(lat, lon, alt, accM)
     *
     * GGA fields:
     *   [9]  = altitude above MSL (m)
     *   [11] = geoidal separation N (m): WGS84 ellipsoid height = MSL alt + N
     *
     * The rest of the pipeline (ENU projection, AnchorEstimator) expects WGS84
     * ellipsoidal altitude, so we add the geoidal separation when available.
     */
    private fun parseGga(fields: List<String>) {
        // $GxGGA,time,lat,N/S,lon,E/W,fix,numSv,hdop,altMSL,M,geoidSep,M,...
        if (fields.size < 10) return

        val latStr = fields[2]
        val ns = fields[3]
        val lonStr = fields[4]
        val ew = fields[5]
        val fixQuality = fields[6]
        val hdopStr = fields[8]
        val altMslStr = fields[9]
        // field[10] = "M" (unit); field[11] = geoidal separation
        val geoidSepStr = fields.getOrNull(11)

        // 0 = sin fix
        if (fixQuality.isEmpty() || fixQuality == "0") return
        if (latStr.isBlank() || lonStr.isBlank()) return

        val lat = nmeaCoordToDeg(latStr, ns)
        val lon = nmeaCoordToDeg(lonStr, ew)
        if (!lat.isFinite() || !lon.isFinite()) return

        val altMsl = altMslStr.toDoubleOrNull() ?: 0.0
        val geoidSep = geoidSepStr?.toDoubleOrNull() ?: 0.0
        // WGS84 ellipsoidal height = MSL altitude + geoidal separation
        val alt = altMsl + geoidSep

        val hdop = hdopStr.toDoubleOrNull() ?: -1.0
        val accM = if (hdop > 0.0) hdop * 1.5 else -1.0   // heurística simple
        if (hdop > 0.0) lastHdop = hdop

        onFix(lat, lon, alt, accM)
    }

    /**
     * Convierte "ddmm.mmmm" o "dddmm.mmmm" (+ hemisferio) a grados decimales.
     */
    private fun nmeaCoordToDeg(coord: String, hemi: String): Double {
        if (coord.length < 4) return Double.NaN

        // grados = todo menos los dos últimos dígitos de minutos + resto/60
        val dot = coord.indexOf('.')
        val degLen = if (dot > 0) dot - 2 else coord.length - 2
        if (degLen <= 0) return Double.NaN

        val degPart = coord.substring(0, degLen)
        val minPart = coord.substring(degLen)

        val deg = degPart.toDoubleOrNull() ?: return Double.NaN
        val min = minPart.toDoubleOrNull() ?: return Double.NaN

        var valDeg = deg + (min / 60.0)
        when (hemi.uppercase()) {
            "S", "W" -> valDeg = -valDeg
        }
        return valDeg
    }

    // -------------------------------------------------------------------------
    //  USB helpers
    // -------------------------------------------------------------------------

    /**
     * Busca un driver USB-serial, priorizando vendor u-blox si existe.
     */
    private fun findPreferredDriver(preferredDevice: UsbDevice?): UsbSerialDriver? {
        val prober = UsbSerialProber.getDefaultProber()

        // 1) Si nos pasan un device, intentamos ese primero
        if (preferredDevice != null) {
            val drv = prober.probeDevice(preferredDevice)
            if (drv != null) {
                Log.i(TAG, "Using preferred USB device: ${preferredDevice.productName} (${preferredDevice.deviceName})")
                return drv
            } else {
                Log.w(TAG, "Preferred device is not recognized as USB-serial: ${preferredDevice.deviceName}")
            }
        }

        // 2) Si no, fallback a cualquiera que tenga driver
        val drivers = prober.findAllDrivers(usbManager)
        if (drivers.isEmpty()) {
            Log.w(TAG, "UsbSerialProber: no se han encontrado drivers")
            return null
        }

        // 3) Preferencias suaves (si aparece u-blox, bien; si no, primero)
        val ubloxLike = drivers.firstOrNull { d ->
            val p = (d.device.productName ?: "").lowercase()
            val m = runCatching { d.device.manufacturerName ?: "" }.getOrDefault("").lowercase()
            val s = "$p $m"
            (d.device.vendorId == UBLOX_VENDOR_ID) || ("u-blox" in s) || ("ublox" in s) || ("f9p" in s) || ("zed" in s)
        }

        return ubloxLike ?: drivers.first()
    }


    private fun isUbloxDevice(device: UsbDevice): Boolean {
        // Vendor ID típico de u-blox = 0x1546 (5446 decimal)
        return device.vendorId == UBLOX_VENDOR_ID
    }

    private fun safeCloseAll() {
        safeClosePort()
        safeCloseConnection()
    }

    private fun safeClosePort() {
        try {
            serialPort?.close()
        } catch (e: Exception) {
            Log.w(TAG, "Error cerrando puerto serie GNSS", e)
        } finally {
            serialPort = null
        }
    }

    private fun safeCloseConnection() {
        try {
            deviceConnection?.close()
        } catch (e: Exception) {
            Log.w(TAG, "Error cerrando conexión USB GNSS", e)
        } finally {
            deviceConnection = null
        }
    }

    companion object {
        private const val TAG = "ExternalGnss"
        private const val UBLOX_VENDOR_ID = 0x1546
    }

    // -------------------------------------------------------------------------
    //  NTRIP
    // -------------------------------------------------------------------------

    /**
     * Call this to update the NTRIP config at runtime (e.g. from UI or ADB).
     * If GNSS is already running, restarts NTRIP with the new config.
     */
    fun updateNtripConfig(config: Config?) {
        ntripConfig = config
        stopNtrip()
        val port = serialPort
        if (config != null && config.ntripHost.isNotBlank() && port != null && started.get()) {
            startNtripWithConfig(config, port)
        }
    }

    private fun startNtripIfConfigured(port: UsbSerialPort) {
        val cfg = ntripConfig ?: return
        if (cfg.ntripHost.isBlank() || cfg.mountpoint.isBlank()) {
            Log.i(TAG, "NTRIP config incomplete, skipping")
            return
        }
        startNtripWithConfig(cfg, port)
    }

    private fun startNtripWithConfig(cfg: Config, port: UsbSerialPort) {
        val client = NtripClient(
            host = cfg.ntripHost,
            port = cfg.ntripPort,
            mountpoint = cfg.mountpoint,
            user = cfg.ntripUser,
            password = cfg.ntripPass
        )

        client.onRtcmData = { data, len ->
            // Forward RTCM corrections to the F9P via serial
            try {
                val chunk = if (len == data.size) data else data.copyOfRange(0, len)
                synchronized(serialWriteLock) {
                    port.write(chunk, 200)
                }
            } catch (e: Exception) {
                Log.w(TAG, "Error writing RTCM to serial: ${e.message}")
            }
        }

        client.onStatus = { msg ->
            onStatus("NTRIP: $msg")
        }

        ntripClient = client
        client.start()
        onStatus("NTRIP client started for ${cfg.ntripHost}:${cfg.ntripPort}/${cfg.mountpoint}")
    }

    private fun stopNtrip() {
        ntripClient?.stop()
        ntripClient = null
    }

    /**
     * Feed the latest GGA sentence to the NTRIP caster (for VRS).
     * Called automatically from parseGga when we have a valid fix.
     */
    private fun sendGgaToCaster(ggaSentence: String) {
        ntripClient?.sendGga(ggaSentence)
    }
}
