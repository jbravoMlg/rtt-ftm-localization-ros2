package com.jbravo.osa_ftm

import android.annotation.SuppressLint
import android.app.Service
import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.content.Context
import android.content.Intent
import android.os.Binder
import android.os.Build
import android.os.Handler
import android.os.IBinder
import android.os.Looper
import android.os.PowerManager
import android.util.Log
import androidx.core.app.NotificationCompat

/**
 * Foreground service that keeps the acquisition pipeline alive when the
 * Activity moves to the background (e.g. screen off while mounted on drone).
 *
 * Currently the heavy work (RTT, GNSS, ROS 2, estimation) still runs in
 * MainActivity / computeExec — this service provides:
 *  - A persistent foreground notification (required by Android for background
 *    location and Wi-Fi RTT access).
 *  - Periodic status updates in the notification so the pilot can glance
 *    at the drone screen and see if things are running.
 *  - A Binder interface so the Activity can push status strings.
 *
 * Future: migrate the acquisition engine into this service so it truly
 * survives Activity destruction.
 */
class MyBackgroundService : Service() {

    companion object {
        private const val TAG = "MyBackgroundService"
        private const val NOTIFICATION_ID = 1
        private const val CHANNEL_ID = "ros2_channel"
        private const val STATUS_UPDATE_MS = 2_000L
    }

    /** Simple binder for in-process communication with MainActivity. */
    inner class LocalBinder : Binder() {
        fun getService(): MyBackgroundService = this@MyBackgroundService
    }

    private val binder = LocalBinder()

    @Volatile var statusLine: String = "Initializing…"
    @Volatile var rttCount: Long = 0L
    @Volatile var ntripConnected: Boolean = false
    @Volatile var gnssSource: String = "—"

    private val handler = Handler(Looper.getMainLooper())
    private var notificationManager: NotificationManager? = null
    private var wakeLock: PowerManager.WakeLock? = null

    override fun onCreate() {
        super.onCreate()
        createNotificationChannel()
        notificationManager = getSystemService(NotificationManager::class.java)

        acquireWakeLock()

        try {
            startForeground(NOTIFICATION_ID, buildNotification())
        } catch (t: Throwable) {
            Log.e(TAG, "Cannot start foreground service", t)
            releaseWakeLock()
            stopSelf()
            return
        }

        // Periodic notification update
        handler.post(object : Runnable {
            override fun run() {
                if (notificationManager != null) {
                    notificationManager?.notify(NOTIFICATION_ID, buildNotification())
                }
                handler.postDelayed(this, STATUS_UPDATE_MS)
            }
        })
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        return START_STICKY
    }

    override fun onDestroy() {
        handler.removeCallbacksAndMessages(null)
        releaseWakeLock()
        super.onDestroy()
    }

    override fun onBind(intent: Intent?): IBinder = binder

    private fun buildNotification(): Notification {
        val ntripStatus = if (ntripConnected) "NTRIP: RTK" else "NTRIP: off"
        val text = "RTT: $rttCount | GNSS: $gnssSource | $ntripStatus\n$statusLine"

        return NotificationCompat.Builder(this, CHANNEL_ID)
            .setContentTitle("WiFi RTT Acquisition")
            .setContentText(text)
            .setStyle(NotificationCompat.BigTextStyle().bigText(text))
            .setSmallIcon(R.drawable.ic_launcher_foreground)
            .setPriority(NotificationCompat.PRIORITY_LOW)
            .setOngoing(true)
            .build()
    }

    private fun createNotificationChannel() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            val name = "RTT Acquisition"
            val descriptionText = "Foreground service for WiFi RTT / GNSS acquisition"
            val importance = NotificationManager.IMPORTANCE_LOW
            val channel = NotificationChannel(CHANNEL_ID, name, importance).apply {
                description = descriptionText
            }
            getSystemService(NotificationManager::class.java)
                ?.createNotificationChannel(channel)
        }
    }

    @SuppressLint("WakelockTimeout")
    private fun acquireWakeLock() {
        if (wakeLock?.isHeld == true) return
        val powerManager = getSystemService(Context.POWER_SERVICE) as? PowerManager ?: return
        wakeLock = powerManager.newWakeLock(
            PowerManager.PARTIAL_WAKE_LOCK,
            "$packageName:acquisition"
        ).apply {
            setReferenceCounted(false)
            acquire()
        }
    }

    private fun releaseWakeLock() {
        try {
            if (wakeLock?.isHeld == true) wakeLock?.release()
        } catch (t: Throwable) {
            Log.w(TAG, "Error releasing wake lock", t)
        } finally {
            wakeLock = null
        }
    }
}
