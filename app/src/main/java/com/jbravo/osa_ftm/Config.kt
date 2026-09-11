package com.jbravo.osa_ftm

import android.content.Context

class Config(
    val ntripUser: String = "",
    val ntripPass: String = "",
    val ntripHost: String = "",
    val ntripPort: Int = 2101,
    val mountpoint: String = "",
    val serialBaud: Int = 115200
) {
    /** True if the minimum fields for an NTRIP connection are filled. */
    val isNtripReady: Boolean
        get() = ntripHost.isNotBlank() && mountpoint.isNotBlank()

    companion object {
        private const val PREFS_NAME = "ntrip_config"

        fun from(map: Map<String, String>): Config {
            return Config(
                ntripUser = map["user"] ?: "",
                ntripPass = map["pass"] ?: "",
                ntripHost = map["host"] ?: "",
                ntripPort = (map["port"] ?: "2101").toIntOrNull() ?: 2101,
                mountpoint = map["mountpoint"] ?: "",
                serialBaud = (map["baud"] ?: "115200").toIntOrNull() ?: 115200
            )
        }

        /** Load from SharedPreferences. Returns null if nothing was saved. */
        fun loadFromPrefs(ctx: Context): Config? {
            val p = ctx.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
            val host = p.getString("host", null) ?: return null
            return Config(
                ntripHost = host,
                ntripPort = p.getInt("port", 2101),
                mountpoint = p.getString("mountpoint", "") ?: "",
                ntripUser = p.getString("user", "") ?: "",
                ntripPass = p.getString("pass", "") ?: "",
                serialBaud = p.getInt("baud", 115200)
            )
        }

        /** Persist to SharedPreferences. */
        fun saveToPrefs(ctx: Context, cfg: Config) {
            ctx.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
                .edit()
                .putString("host", cfg.ntripHost)
                .putInt("port", cfg.ntripPort)
                .putString("mountpoint", cfg.mountpoint)
                .putString("user", cfg.ntripUser)
                .putString("pass", cfg.ntripPass)
                .putInt("baud", cfg.serialBaud)
                .apply()
        }
    }
}
