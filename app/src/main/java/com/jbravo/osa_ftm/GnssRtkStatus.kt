package com.jbravo.osa_ftm

/**
 * Snapshot of GNSS + NTRIP RTK status for UI display.
 *
 * @param fixQuality GGA field [6]: 0=No fix, 1=GPS, 2=DGNSS, 4=RTK Fixed, 5=RTK Float
 * @param numSatellites Number of satellites used in solution.
 * @param hdop Horizontal Dilution of Precision (lower = better, typically 0.5-2.0).
 * @param rtcmBytesReceived Cumulative RTCM correction bytes received from the caster.
 * @param ntripConnected Whether the NTRIP TCP connection is alive.
 * @param externalGnssActive Whether the USB GNSS receiver is active.
 */
data class GnssRtkStatus(
    val fixQuality: Int = -1,
    val numSatellites: Int = -1,
    val hdop: Double = -1.0,
    val rtcmBytesReceived: Long = 0L,
    val ntripConnected: Boolean = false,
    val externalGnssActive: Boolean = false
) {
    val fixLabel: String get() = when (fixQuality) {
        0 -> "No fix"
        1 -> "GPS"
        2 -> "DGNSS"
        4 -> "RTK Fixed"
        5 -> "RTK Float"
        else -> "—"
    }

    val isRtkFixed: Boolean get() = fixQuality == 4
    val isRtkFloat: Boolean get() = fixQuality == 5
    val hasRtk: Boolean get() = fixQuality == 4 || fixQuality == 5
}
