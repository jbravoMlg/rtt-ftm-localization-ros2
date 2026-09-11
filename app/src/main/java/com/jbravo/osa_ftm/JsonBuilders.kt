package com.jbravo.osa_ftm

/**
 * Utilidades para construir pequeños JSONs sin depender de librerías externas.
 *
 * IMPORTANTE:
 *  - Todas las funciones devuelven SOLO el objeto JSON, sin envolver en
 *    {"ts":...,"type":...,"data":...}. Ese envoltorio lo hace FileLogger.appendTaggedJson().
 */
object JsonBuilders {

    /**
     * Escapa una cadena como literal JSON (SIN comillas exteriores).
     *
     * Ejemplo: esc("a\"b") -> a\"b
     */
    private fun esc(s: String): String {
        val sb = StringBuilder(s.length + 8)
        for (c in s) {
            when (c) {
                '"'  -> sb.append("\\\"")
                '\\' -> sb.append("\\\\")
                '\b' -> sb.append("\\b")
                '\u000C' -> sb.append("\\f")
                '\n' -> sb.append("\\n")
                '\r' -> sb.append("\\r")
                '\t' -> sb.append("\\t")
                else -> {
                    if (c < ' ') {
                        sb.append(String.format("\\u%04x", c.code))
                    } else {
                        sb.append(c)
                    }
                }
            }
        }
        return sb.toString()
    }

    /**
     * JSON de una medición RTT simple:
     *
     * {
     *   "bssid": "...",
     *   "distance_m": ...,
     *   "std_m": ...,
     *   "rssi_dbm": ...,
     *   "t_ms": ...
     * }
     */
    fun rttJson(
        bssid: String?,
        distM: Double,
        stdM: Double,
        rssi: Int,
        tMs: Long
    ): String {
        val b = (bssid ?: "")
        val sb = StringBuilder(128)
        sb.append('{')
            .append("\"bssid\":\"").append(esc(b)).append('"')
            .append(",\"distance_m\":").append(distM)
            .append(",\"std_m\":").append(stdM)
            .append(",\"rssi_dbm\":").append(rssi)
            .append(",\"t_ms\":").append(tMs)
            .append('}')
        return sb.toString()
    }

    /**
     * JSON de una estimación de multilateración:
     *
     * {
     *   "bssid": "...",
     *   "lat": ...,
     *   "lon": ...,
     *   "alt": ...,
     *   "cov_xx": ...,
     *   "cov_yy": ...,
     *   "cov_xy": ...,
     *   "n_samples": ...,
     *   "bssid_hash": ...,
     *   "t_ms": ...
     * }
     *
     * (Si necesitas más campos de debug, añádelos por encima antes del '}'.)
     */
    fun mlatJson(
        bssid: String?,
        lat: Double,
        lon: Double,
        alt: Double,
        cov_xx: Double,
        cov_yy: Double,
        cov_xy: Double,
        nS: Double,
        bssidHash: Double,
        tMs: Long
    ): String {
        val b = (bssid ?: "")
        val sb = StringBuilder(192)
        sb.append('{')
            .append("\"bssid\":\"").append(esc(b)).append('"')
            .append(",\"lat\":").append(lat)
            .append(",\"lon\":").append(lon)
            .append(",\"alt\":").append(alt)
            .append(",\"cov_xx\":").append(cov_xx)
            .append(",\"cov_yy\":").append(cov_yy)
            .append(",\"cov_xy\":").append(cov_xy)
            .append(",\"n_samples\":").append(nS)
            .append(",\"bssid_hash\":").append(bssidHash)
            .append(",\"t_ms\":").append(tMs)
            .append('}')
        return sb.toString()
    }

    /**
     * (Opcional) JSON sencillo para GNSS, por si quieres centralizarlo aquí.
     *
     * {
     *   "lat": ...,
     *   "lon": ...,
     *   "alt": ...,
     *   "acc_m": ...,
     *   "t_ms": ...
     * }
     */
    fun gpsJson(
        lat: Double,
        lon: Double,
        alt: Double,
        accM: Double,
        tMs: Long
    ): String {
        val sb = StringBuilder(160)
        sb.append('{')
            .append("\"lat\":").append(lat)
            .append(",\"lon\":").append(lon)
            .append(",\"alt\":").append(alt)
            .append(",\"acc_m\":").append(accM)
            .append(",\"t_ms\":").append(tMs)
            .append('}')
        return sb.toString()
    }
}
