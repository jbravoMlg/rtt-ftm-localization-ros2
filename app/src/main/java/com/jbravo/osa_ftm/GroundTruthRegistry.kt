package com.jbravo.osa_ftm

import android.content.Context
import android.util.Log
import java.io.File

/**
 * Registry of known AP positions (ground truth) for research benchmarking.
 *
 * Each AP is identified by its BSSID (MAC) and has a known WGS84 position
 * (lat, lon, alt) that was surveyed before the experiment.
 *
 * Ground-truth sources (checked in order):
 *  1. ADB broadcast:
 *       adb shell am broadcast -a com.jbravo.osa_ftm.GROUND_TRUTH \
 *           --es bssid "aa:bb:cc:dd:ee:ff" \
 *           --es lat "36.7201" --es lon "-4.4203" --es alt "42.5" \
 *           --es description "AP on surface"
 *
 *  2. File:  <app-files>/ground_truth.csv
 *       #bssid,lat,lon,alt,description
 *       aa:bb:cc:dd:ee:ff,36.7201,-4.4203,42.5,AP on surface
 *       11:22:33:44:55:66,36.7199,-4.4205,41.0,AP buried 1m
 *
 *  3. UI (manual entry per AP)
 *
 * All positions are compared against live EKF estimates to compute
 * real-time accuracy metrics (CEP, RMSE, etc.).
 */
class GroundTruthRegistry {

    companion object {
        private const val TAG = "GroundTruth"
        private const val PREFS = "ground_truth"
    }

    data class ApTruth(
        val bssid: String,      // canonical lowercase MAC
        val lat: Double,
        val lon: Double,
        val alt: Double,
        val description: String = ""
    )

    private val registry = LinkedHashMap<String, ApTruth>()

    /** Number of registered ground-truth APs. */
    val size: Int get() = registry.size

    /** Register or update an AP's known position. */
    fun put(truth: ApTruth) {
        val key = truth.bssid.trim().lowercase()
        registry[key] = truth.copy(bssid = key)
    }

    /** Remove an AP from the registry. */
    fun remove(bssid: String) {
        registry.remove(bssid.trim().lowercase())
    }

    /** Get the ground truth for a specific AP, or null. */
    fun get(bssid: String): ApTruth? =
        registry[bssid.trim().lowercase()]

    /** All registered APs (snapshot). */
    fun all(): List<ApTruth> = registry.values.toList()

    /** Clear all entries. */
    fun clear() = registry.clear()

    // -----------------------------------------------------------------------
    //  CSV file loading
    // -----------------------------------------------------------------------

    /**
     * Load from CSV file in the app's external files directory.
     * Format: bssid,lat,lon,alt[,description]
     * Lines starting with '#' are comments.
     */
    fun loadFromFile(ctx: Context): Int {
        val file = File(ctx.getExternalFilesDir(null) ?: return 0, "ground_truth.csv")
        if (!file.exists()) {
            Log.i(TAG, "ground_truth.csv not found in ${file.absolutePath}")
            return 0
        }

        var count = 0
        try {
            file.forEachLine { rawLine ->
                val line = rawLine.trim()
                if (line.isEmpty() || line.startsWith("#")) return@forEachLine
                val parts = line.split(",")
                if (parts.size < 4) return@forEachLine

                val bssid = parts[0].trim().lowercase()
                val lat = parts[1].trim().toDoubleOrNull() ?: return@forEachLine
                val lon = parts[2].trim().toDoubleOrNull() ?: return@forEachLine
                val alt = parts[3].trim().toDoubleOrNull() ?: return@forEachLine
                val desc = parts.getOrNull(4)?.trim() ?: ""

                put(ApTruth(bssid, lat, lon, alt, desc))
                count++
            }
            Log.i(TAG, "Loaded $count ground-truth APs from ${file.absolutePath}")
        } catch (e: Exception) {
            Log.e(TAG, "Error loading ground_truth.csv", e)
        }
        return count
    }

    // -----------------------------------------------------------------------
    //  SharedPreferences persistence
    // -----------------------------------------------------------------------

    fun saveToPrefs(ctx: Context) {
        val entries = registry.values.joinToString(";") { ap ->
            "${ap.bssid},${ap.lat},${ap.lon},${ap.alt},${ap.description}"
        }
        ctx.getSharedPreferences(PREFS, Context.MODE_PRIVATE)
            .edit()
            .putString("entries", entries)
            .apply()
    }

    fun loadFromPrefs(ctx: Context): Int {
        val raw = ctx.getSharedPreferences(PREFS, Context.MODE_PRIVATE)
            .getString("entries", null) ?: return 0
        if (raw.isBlank()) return 0

        var count = 0
        for (entry in raw.split(";")) {
            val parts = entry.split(",")
            if (parts.size < 4) continue
            val bssid = parts[0].trim().lowercase()
            val lat = parts[1].toDoubleOrNull() ?: continue
            val lon = parts[2].toDoubleOrNull() ?: continue
            val alt = parts[3].toDoubleOrNull() ?: continue
            val desc = parts.getOrNull(4) ?: ""
            put(ApTruth(bssid, lat, lon, alt, desc))
            count++
        }
        return count
    }
}
