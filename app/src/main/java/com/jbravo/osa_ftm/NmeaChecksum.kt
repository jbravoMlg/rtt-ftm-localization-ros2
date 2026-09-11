package com.jbravo.osa_ftm

object NmeaChecksum {

    /**
     * Verifica si la frase NMEA tiene un checksum válido.
     * @param sentence La línea NMEA completa, como "$GPGGA,...*5C"
     * @return true si el checksum es correcto
     */
    fun ok(sentence: String): Boolean {
        val asterisk = sentence.indexOf('*')
        if (asterisk < 0 || asterisk + 3 > sentence.length) return false

        val data = sentence.substring(1, asterisk) // Excluye el '$'
        var checksum = 0
        for (c in data) {
            checksum = checksum xor c.code
        }

        val expected = sentence.substring(asterisk + 1, asterisk + 3)
        return try {
            val expectedValue = expected.toInt(16)
            checksum == expectedValue
        } catch (_: NumberFormatException) {
            false
        }
    }
}
