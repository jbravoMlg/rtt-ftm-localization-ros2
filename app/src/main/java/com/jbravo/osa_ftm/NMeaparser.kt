package com.jbravo.osa_ftm

object NmeaParser {

    /**
     * Convierte la latitud NMEA a decimal.
     * Ejemplo: "4807.038", "N" → 48.1173
     */
    fun latitude(raw: String?, direction: String?): Double {
        if (raw == null || raw.length < 4) return 0.0
        val deg = raw.substring(0, 2).toDoubleOrNull() ?: return 0.0
        val min = raw.substring(2).toDoubleOrNull() ?: return 0.0
        val value = deg + min / 60.0
        return if (direction == "S") -value else value
    }

    /**
     * Convierte la longitud NMEA a decimal.
     * Ejemplo: "01131.000", "E" → 11.5167
     */
    fun longitude(raw: String?, direction: String?): Double {
        if (raw == null || raw.length < 5) return 0.0
        val deg = raw.substring(0, 3).toDoubleOrNull() ?: return 0.0
        val min = raw.substring(3).toDoubleOrNull() ?: return 0.0
        val value = deg + min / 60.0
        return if (direction == "W") -value else value
    }
}
