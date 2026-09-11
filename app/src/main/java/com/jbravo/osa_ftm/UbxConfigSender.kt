package com.jbravo.osa_ftm

import android.util.Log
import com.hoho.android.usbserial.driver.UsbSerialPort

object UbxConfigSender {

    private const val TAG = "UbxConfigSender"

    private const val CLASS_CFG = 0x06
    private const val ID_CFG_RATE = 0x08
    private const val ID_CFG_MSG = 0x01

    /**
     * Construye un frame UBX completo (cabecera, longitud, payload, checksum).
     */
    private fun buildUbxFrame(
        classId: Int,
        msgId: Int,
        payload: ByteArray
    ): ByteArray {
        val len = payload.size
        val frame = ByteArray(6 + len + 2)  // header(2) + class/id(2) + len(2) + payload + ck(2)

        frame[0] = 0xB5.toByte()
        frame[1] = 0x62.toByte()
        frame[2] = classId.toByte()
        frame[3] = msgId.toByte()
        frame[4] = (len and 0xFF).toByte()
        frame[5] = ((len shr 8) and 0xFF).toByte()

        System.arraycopy(payload, 0, frame, 6, len)

        var ckA = 0
        var ckB = 0
        for (i in 2 until 6 + len) {          // desde classId hasta último byte de payload
            val b = frame[i].toInt() and 0xFF
            ckA = (ckA + b) and 0xFF
            ckB = (ckB + ckA) and 0xFF
        }
        frame[6 + len] = ckA.toByte()
        frame[7 + len] = ckB.toByte()

        return frame
    }

    /**
     * Configura el F9P para:
     *  - solución de navegación a 10 Hz (measRate = 100 ms, navRate = 1)
     *  - asegurar que el NMEA GGA está habilitado en todos los puertos
     *
     * Requiere que el puerto esté ya abierto y a la misma velocidad que el GNSS.
     */
    fun configureF9PFor10Hz(port: UsbSerialPort) {
        try {

            // Limpia buffers por si hay NMEA en vuelo
            try {
                port.purgeHwBuffers(true, true)
            } catch (_: Exception) {}

            // ---------------- CFG-RATE (0x06 0x08) ----------------
            val measRateMs = 100   // 100 ms → 10 Hz
            val navRate = 1        // cada ciclo de medida produce solución
            val timeRef = 0        // 0=UTC, 1=GPS (cualquiera suele valer) - Antes era 1

            val payloadRate = ByteArray(6)



            payloadRate[0] = (measRateMs and 0xFF).toByte()
            payloadRate[1] = ((measRateMs shr 8) and 0xFF).toByte()
            payloadRate[2] = (navRate and 0xFF).toByte()
            payloadRate[3] = ((navRate shr 8) and 0xFF).toByte()
            payloadRate[4] = (timeRef and 0xFF).toByte()
            payloadRate[5] = ((timeRef shr 8) and 0xFF).toByte()

            val frameRate = buildUbxFrame(CLASS_CFG, ID_CFG_RATE, payloadRate)
            Log.i(TAG, "Sending CFG-RATE (10 Hz): ${frameRate.joinToString { "%02X".format(it) }}")
            port.write(frameRate, 500)
            readAndLog(port, "CFG-RATE")

            // ---------------- CFG-MSG para NMEA GGA (0xF0 0x00) ----------------
            // Variante de 8 bytes: msgClass, msgId, rate[6] (I2C, UART1, UART2, USB, SPI, reserved)
            // Ponemos rate=1 (una vez por solución) en todos los puertos, para ir a la par con 10 Hz.
            val payloadMsg = ByteArray(8)
            payloadMsg[0] = 0xF0.toByte()  // NMEA message class
            payloadMsg[1] = 0x00.toByte()  // GGA
            for (i in 2 until 8) {
                payloadMsg[i] = 1          // 1 por cada solución de navegación
            }

            val frameMsg = buildUbxFrame(CLASS_CFG, ID_CFG_MSG, payloadMsg)
            Log.i(TAG, "Ensuring NMEA GGA enabled: ${frameMsg.joinToString { "%02X".format(it) }}")
            port.write(frameMsg, 500)
            readAndLog(port, "CFG-MSG GGA")

            Log.i(TAG, "F9P configuration for 10 Hz sent.")
        } catch (e: Exception) {
            Log.e(TAG, "Error sending UBX config to F9P", e)
            throw e
        }
    }

    private fun readAndLog(port: UsbSerialPort, label: String) {
        val buf = ByteArray(128)
        try {
            val n = port.read(buf, 200) // 200 ms de timeout
            if (n > 0) {
                val hex = buf.take(n).joinToString(" ") { "%02X".format(it) }
                Log.i(TAG, "After $label read $n bytes: $hex")
            } else {
                Log.i(TAG, "After $label read: no data")
            }
        } catch (e: Exception) {
            Log.w(TAG, "Error reading after $label", e)
        }
    }

}
