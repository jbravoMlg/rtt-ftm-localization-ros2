package com.jbravo.osa_ftm

import android.util.Log

import android.os.SystemClock
import java.util.UUID

import android.provider.Settings

import android.Manifest
import android.annotation.SuppressLint
import android.content.ComponentCallbacks2
import android.content.Context
import androidx.core.content.ContextCompat
import com.hoho.android.usbserial.driver.UsbSerialProber
import com.hoho.android.usbserial.driver.UsbSerialDriver
import java.util.concurrent.ConcurrentHashMap

import android.content.pm.PackageManager
import android.location.Location
import android.net.wifi.ScanResult
import android.net.wifi.WifiManager
import android.net.wifi.rtt.RangingRequest
import android.net.wifi.rtt.WifiRttManager
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.animation.AnimatedVisibility
import androidx.compose.animation.core.animateFloatAsState
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Add
import androidx.compose.material.icons.filled.Delete
import androidx.compose.material.icons.filled.ExpandMore
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.rotate
import androidx.compose.ui.text.input.TextFieldValue
import androidx.compose.ui.unit.dp
import androidx.core.app.ActivityCompat
import com.google.android.gms.location.*
import kotlin.math.abs
import kotlin.math.ln
import java.util.ArrayDeque
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean
import java.lang.ThreadLocal as JThreadLocal
import android.content.BroadcastReceiver
import android.content.Intent
import android.content.IntentFilter
import android.os.IBinder
import android.app.PendingIntent
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbManager
import androidx.compose.runtime.MutableState
import java.util.concurrent.atomic.AtomicLong

class MainActivity : ComponentActivity() {

    /* -------------------------------------------------------------------------
     *  Estado general / hilos
     * ---------------------------------------------------------------------- */
    @Volatile
    private var weightingState: MutableState<Weighting>? = null

    @Volatile private var expId: String? = null
    @Volatile private var expActive: Boolean = false
    @Volatile private var expStartWallMs: Long = 0L
    @Volatile private var expStartMonoMs: Long = 0L

    private val rttSeq = AtomicLong(0L)  // Contador de sequencia rtt
    private val rttCount = AtomicLong(0L) // para comparar con el contador de paquetes ROS

    // Wi-Fi RTT
    private lateinit var wifiManager: WifiManager
    private var ddsMulticastLock: WifiManager.MulticastLock? = null
    private var rttManager: WifiRttManager? = null

    private var rttStateReceiverRegistered = false
    @Volatile private var lastRttNotAvailLogMs: Long = 0L

    private fun isRttAvailable(): Boolean = (rttManager?.isAvailable == true)

    private data class AnchorTrack(
        val est: AnchorEstimator,
        val rttGate: RobustRttGate = RobustRttGate(),
        val outMedXY: Median2D = Median2D(7),

        // Mediana del "suelo local" estimado: z_ground ≈ uU - AGL
        val zGroundMed: RollingMedian = RollingMedian(25),

        // Suavizado de z del anchor estimado (no suelo)
        val zOutMed: RollingMedian = RollingMedian(9),

        var ekf3d: AnchorIEKF3D? = null,
        var ukf3d: AnchorUKF3D? = null,            // Used when FilterMode.UKF
        var stableStreak: Int = 0,
        var lastPubXYZ: Triple<Double, Double, Double>? = null,
        var lastStateLogMs: Long = 0L,

        // --- Adaptive threshold tracking ---
        var prevSigmaH: Double = Double.MAX_VALUE,
        var prevSigmaZ: Double = Double.MAX_VALUE,
        /** How many consecutive updates sigma_h decreased. */
        var convergingStreak: Int = 0,

        // --- Filter mode components (created once per track) ---
        val nlosDetector: NlosDetector = NlosDetector(),
        val empiricalRModel: EmpiricalRModel = EmpiricalRModel(),
        val emaXY: EmaSmooth2D = EmaSmooth2D(0.3),
        val adaptiveR: AdaptiveR = AdaptiveR()
    )

    private val tracks = ConcurrentHashMap<String, AnchorTrack>()


    // ROS 2
    private var ros2: Ros2RttNode? = null
    private var activeRosNamespaceCore: String = ""

    @Volatile private var pendingGnssDevice: UsbDevice? = null


    // GNSS interno (FusedLocationProviderClient)
    private lateinit var fused: FusedLocationProviderClient
    private lateinit var locReq: LocationRequest
    private var locCb: LocationCallback? = null
    @Volatile private var gnssActive = false
    private val allowInternalGnssFallback = false

    // GNSS externo por USB (ZED-F9P, etc.)
    // Campos de estado
    private var externalGnss: ExternalGnssManager? = null
    @Volatile private var externalGnssActive = false
    // @Volatile private var lastExternalGnssFixMs: Long = 0L   // Cambio ultimo fix externo a monotonico.

    @Volatile private var lastExternalGnssFixMonoMs: Long = 0L

    // Background service binding
    private var bgService: MyBackgroundService? = null
    private val serviceConnection = object : android.content.ServiceConnection {
        override fun onServiceConnected(name: android.content.ComponentName?, binder: IBinder?) {
            bgService = (binder as? MyBackgroundService.LocalBinder)?.getService()
        }
        override fun onServiceDisconnected(name: android.content.ComponentName?) {
            bgService = null
        }
    }
    private var serviceBound = false

    // ---------- Experiment / ground-truth / accuracy ----------
    private var experimentConfig by mutableStateOf(ExperimentConfig())
    @Volatile private var activeFilterMode: FilterMode = FilterMode.BASELINE
    private val groundTruth = GroundTruthRegistry()
    private val accuracyMetrics = AccuracyMetrics()

    private var ntripConfig: Config? = null   // cuando cargues el txt, la rellenas
    @Volatile private var lastExternalUiLogMs: Long = 0L

    // NTRIP connection state (for UI)
    private var ntripConnected by mutableStateOf(false)
    private var gnssRtkStatus by mutableStateOf(GnssRtkStatus())

    /**
     * Apply an NTRIP config: persist to prefs, update ntripConfig field,
     * and push to ExternalGnssManager if running.
     */
    private fun applyNtripConfig(cfg: Config) {
        ntripConfig = cfg
        Config.saveToPrefs(applicationContext, cfg)
        externalGnss?.updateNtripConfig(cfg)
        appendLog("NTRIP config applied: ${cfg.ntripHost}:${cfg.ntripPort}/${cfg.mountpoint}")
    }

    private fun disconnectNtrip() {
        externalGnss?.updateNtripConfig(null)
        ntripConnected = false
        appendLog("NTRIP disconnected")
    }

    // BroadcastReceiver for ADB config:
    //   adb shell am broadcast -a com.jbravo.osa_ftm.NTRIP_CONFIG \
    //     --es host "caster.example.com" --es port "2101" \
    //     --es mountpoint "MOUNT" --es user "myuser" --es pass "mypass"
    private val configBroadcastReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            if (intent.action != ACTION_NTRIP_CONFIG) return

            val host = intent.getStringExtra("host") ?: ""
            val port = intent.getStringExtra("port")?.toIntOrNull() ?: 2101
            val mountpoint = intent.getStringExtra("mountpoint") ?: ""
            val user = intent.getStringExtra("user") ?: ""
            val pass = intent.getStringExtra("pass") ?: ""

            if (host.isBlank()) {
                appendLog("ADB NTRIP_CONFIG: missing 'host' extra, ignoring")
                return
            }

            val cfg = Config(
                ntripHost = host,
                ntripPort = port,
                mountpoint = mountpoint,
                ntripUser = user,
                ntripPass = pass
            )

            appendLog("NTRIP config received via ADB: $host:$port/$mountpoint")
            applyNtripConfig(cfg)
        }
    }
    private var configReceiverRegistered = false

    // ADB receiver: experiment config
    //   adb shell am broadcast -a com.jbravo.osa_ftm.EXPERIMENT_CONFIG \
    //       --es label "flight_30m_run1" --es agl "30.0" --es notes "clear sky" \
    //       --es range_bias "1.5"
    private val experimentConfigReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            if (intent.action != ACTION_EXPERIMENT_CONFIG) return
            val label = intent.getStringExtra("label") ?: ""
            val agl = intent.getStringExtra("agl")?.toDoubleOrNull() ?: experimentConfig.aglMeters
            val notes = intent.getStringExtra("notes") ?: ""
            val filterStr = intent.getStringExtra("filter") ?: experimentConfig.filterMode.name
            val filterMode = FilterMode.fromString(filterStr)
            val bias = intent.getStringExtra("range_bias")?.toDoubleOrNull() ?: experimentConfig.rangeBiasM

            // Multi-agent (cooperative OSA) extras
            val agentIdIn = intent.getStringExtra("agent_id")
            val peerIn = intent.getStringExtra("peer_agent_id")
            val coopIn = intent.getStringExtra("cooperative_mode")
            val perIn = intent.getStringExtra("ranging_period_ms")?.toLongOrNull()
            val phaseIn = intent.getStringExtra("ranging_phase_ms")?.toLongOrNull()
            val altModeIn = intent.getStringExtra("altitude_mode")
            val altitudeMode = if (altModeIn != null)
                ExperimentConfig.sanitizeAltitudeMode(altModeIn) else experimentConfig.altitudeMode

            val agentId = if (agentIdIn != null)
                ExperimentConfig.sanitizeAgentId(agentIdIn) else experimentConfig.agentId
            val peerId = if (peerIn != null)
                ExperimentConfig.sanitizeAgentId(peerIn) else experimentConfig.peerAgentId
            val coopMode = if (coopIn != null)
                ExperimentConfig.sanitizeCoopMode(coopIn) else experimentConfig.cooperativeMode
            val rangingPeriod = perIn ?: experimentConfig.rangingPeriodMs
            val rangingPhase = phaseIn ?: experimentConfig.rangingPhaseMs

            experimentConfig = ExperimentConfig(
                label = label, aglMeters = agl, notes = notes,
                filterMode = filterMode, rangeBiasM = bias,
                agentId = agentId, peerAgentId = peerId,
                cooperativeMode = coopMode,
                rangingPeriodMs = rangingPeriod,
                rangingPhaseMs = rangingPhase,
                altitudeMode = altitudeMode
            )
            ExperimentConfig.saveToPrefs(applicationContext, experimentConfig)
            updateAglMeters(agl)
            updateRangeBias(bias)
            applyFilterMode(filterMode)
            updateRangingParams(rangingPeriod, rangingPhase)

            // Start peer subscriptions lazily if cooperative mode is on and
            // a peer is set.  ROS topic names are bound to node namespace at
            // creation, so agent_id changes take effect only after restart.
            if (coopMode != ExperimentConfig.COOP_INDEPENDENT && peerId.isNotBlank()) {
                ros2?.startPeerSubscriptions(peerId)
            }

            val agentWarn = if (agentIdIn != null && ros2 != null)
                " (agent_id change requires app restart to affect ROS topics)" else ""
            appendLog(
                "Experiment config via ADB: label=$label agl=${agl}m bias=${bias}m " +
                "filter=${filterMode.name} agent=$agentId peer=$peerId coop=$coopMode " +
                "period=${rangingPeriod}ms phase=${rangingPhase}ms altmode=$altitudeMode$agentWarn"
            )
        }
    }

    // ADB receiver: ground truth AP
    //   adb shell am broadcast -a com.jbravo.osa_ftm.GROUND_TRUTH \
    //       --es bssid "aa:bb:cc:dd:ee:ff" --es lat "36.72" --es lon "-4.42" \
    //       --es alt "42.5" --es description "AP on surface"
    private val groundTruthReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            if (intent.action != ACTION_GROUND_TRUTH) return
            val bssid = intent.getStringExtra("bssid")?.trim()?.lowercase() ?: return
            val lat = intent.getStringExtra("lat")?.toDoubleOrNull() ?: return
            val lon = intent.getStringExtra("lon")?.toDoubleOrNull() ?: return
            val alt = intent.getStringExtra("alt")?.toDoubleOrNull() ?: return
            val desc = intent.getStringExtra("description") ?: ""

            groundTruth.put(GroundTruthRegistry.ApTruth(bssid, lat, lon, alt, desc))
            groundTruth.saveToPrefs(applicationContext)
            appendLog("Ground truth via ADB: $bssid → ($lat, $lon, $alt) $desc")
        }
    }

    // ADB receiver: remote commands (start / stop / set_weighting)
    //   adb shell am broadcast -a com.jbravo.osa_ftm.REMOTE_CMD --es cmd "start"
    //   adb shell am broadcast -a com.jbravo.osa_ftm.REMOTE_CMD --es cmd "stop"
    //   adb shell am broadcast -a com.jbravo.osa_ftm.REMOTE_CMD --es cmd "set_weighting:WLS"
    private val commandReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            if (intent.action != ACTION_REMOTE_CMD) return
            val cmd = intent.getStringExtra("cmd")?.trim()?.lowercase() ?: return
            appendLog("REMOTE_CMD received: $cmd")
            try {
                handleRemoteCommand(cmd)
            } catch (t: Throwable) {
                Log.e("MainActivity", "REMOTE_CMD failed: $cmd", t)
                appendLog("Remote CMD failed ($cmd): ${t.javaClass.simpleName}: ${t.message}")
            }
        }
    }

    private fun handleRemoteCommand(cmd: String) {
        when {
            cmd == "start" -> runOnUiThreadSafe("REMOTE_CMD start") {
                startExperiment("adb_remote_cmd", "")
                stopContinuousRanging()
                startContinuousRanging("")
                uiContinuous = true
                appendLog("Remote START: experiment + continuous ranging started")
            }
            cmd == "stop" -> runOnUiThreadSafe("REMOTE_CMD stop") {
                stopExperiment("adb_remote_cmd")
                stopContinuousRanging()
                uiContinuous = false
                appendLog("Remote STOP: experiment + continuous ranging stopped")
            }
            cmd.startsWith("set_weighting:") -> {
                val method = cmd.removePrefix("set_weighting:").trim().uppercase()
                val w: Weighting? = when (method) {
                    "WLS" -> Weighting.WLS
                    "HUBER" -> Weighting.Huber(delta = 1.345)
                    "TRIM" -> Weighting.Trim(fraction = 0.20)
                    "SIGMACLIP" -> Weighting.SigmaClip(k = 3.0)
                    else -> null
                }
                if (w != null) {
                    runOnUiThreadSafe("REMOTE_CMD set_weighting") {
                        onWeightingChanged(w)
                        appendLog("Remote SET_WEIGHTING: $w")
                    }
                } else {
                    appendLog("Remote SET_WEIGHTING: unknown method '$method'")
                }
            }
            cmd.startsWith("set_sync_window:") -> {
                val ms = cmd.removePrefix("set_sync_window:").trim().toLongOrNull()
                if (ms != null && ms in 10..30_000) {
                    syncWindowMs = ms
                    appendLog("Remote SET_SYNC_WINDOW: ${ms}ms")
                } else {
                    appendLog("Remote SET_SYNC_WINDOW: invalid value '$cmd'")
                }
            }
            cmd == "get_status" -> writeStatusJsonForAdb()
            cmd == "ntrip_connect" -> runOnUiThreadSafe("REMOTE_CMD ntrip_connect") {
                val cfg = ntripConfig ?: Config.loadFromPrefs(applicationContext)
                if (cfg != null && cfg.isNtripReady) {
                    applyNtripConfig(cfg)
                    ntripConnected = true
                    appendLog("Remote NTRIP_CONNECT: connecting to ${cfg.ntripHost}:${cfg.ntripPort}/${cfg.mountpoint}")
                } else {
                    appendLog("Remote NTRIP_CONNECT: no valid config available")
                }
            }
            cmd == "ntrip_disconnect" -> runOnUiThreadSafe("REMOTE_CMD ntrip_disconnect") {
                disconnectNtrip()
                appendLog("Remote NTRIP_DISCONNECT: disconnected")
            }
            else -> appendLog("Remote CMD unknown: '$cmd'")
        }
    }

    // Flag para saber si hemos registrado el receiver
    private var usbPermissionReceiverRegistered = false



    // Receiver para detectar que se conecta el GNSS externo
    private val usbAttachReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            appendLog("usbAttachReceiver: action=${intent.action}")
            if (intent.action == UsbManager.ACTION_USB_DEVICE_ATTACHED) {
                val dev = intent.getParcelableExtra<UsbDevice>(UsbManager.EXTRA_DEVICE)
                appendLog("USB device attached: ${dev?.deviceName} / ${dev?.productName}")
                appendLog("USB GNSS connected. Trying to start / request permission…")
                maybeStartExternalGnss(preferred = dev)
            }
        }
    }




    private val usbPermissionReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            appendLog("usbPermissionReceiver: action=${intent.action}")

            if (ACTION_USB_PERMISSION == intent.action) {
                val device = intent.getParcelableExtra<UsbDevice>(UsbManager.EXTRA_DEVICE)
                val granted = intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)

                appendLog("usbPermissionReceiver: dev=${device?.deviceName}, granted=$granted")

                if (granted) {
                    appendLog("USB permission granted for GNSS: ${device?.productName ?: device?.deviceName}")

                    val devToStart = device ?: pendingGnssDevice
                    pendingGnssDevice = null

                    if (devToStart != null) {
                        externalGnss?.start(devToStart)
                    } else {
                        externalGnss?.start()
                    }
                } else {
                    appendLog("USB permission denied for GNSS.")
                    pendingGnssDevice = null
                }

            }
        }
    }


    private fun maybeStartExternalGnss(preferred: UsbDevice? = null) {
        val usbManager = getSystemService(Context.USB_SERVICE) as UsbManager

        val best = pickBestUsbSerialDevice(usbManager, preferred)
        if (best == null) {
            appendLog("maybeStartExternalGnss: no USB-Serial device found (no drivers).")
            return
        }

        appendLog(
            "maybeStartExternalGnss: selected device=" +
                    "${best.deviceName}, vid=0x${best.vendorId.toString(16)}, pid=0x${best.productId.toString(16)}, " +
                    "product=${best.productName}"
        )

        if (usbManager.hasPermission(best)) {
            appendLog("maybeStartExternalGnss: already have permission → starting External GNSS")
            externalGnss?.start(best)   // <- arrancar con ESTE device
        } else {
            appendLog("maybeStartExternalGnss: no permission yet → requesting…")
            requestUsbPermissionForGnss(best)
        }
    }

    private fun pickBestUsbSerialDevice(
        usbManager: UsbManager,
        preferred: UsbDevice?
    ): UsbDevice? {
        val prober = UsbSerialProber.getDefaultProber()

        // 1) Si viene un device “attached”, úsalo si el prober lo reconoce como serial.
        if (preferred != null) {
            val d = prober.probeDevice(preferred)
            if (d != null) return preferred
        }

        // 2) Si no, lista todos los USB-Serial disponibles
        val drivers: List<UsbSerialDriver> = prober.findAllDrivers(usbManager)
        if (drivers.isEmpty()) return null

        // 3) Heurística suave: prioriza strings que suelen aparecer en GNSS/placas RTK
        fun score(dev: UsbDevice): Int {
            val p = (dev.productName ?: "").lowercase()
            val m = runCatching { dev.manufacturerName ?: "" }.getOrDefault("").lowercase()

            val s = "$p $m"
            var sc = 0

            // u-blox explícito (cuando exista)
            if ("u-blox" in s || "ublox" in s) sc += 1000
            if ("zed" in s || "f9p" in s || "rtk" in s || "gnss" in s) sc += 500

            // placas comunes
            if ("ardusimple" in s || "sparkfun" in s || "simple" in s) sc += 200

            // si es VID típico u-blox (cuando aplique)
            if (dev.vendorId == 0x1546) sc += 300

            return sc
        }

        val best = drivers
            .map { it.device }
            .maxByOrNull { score(it) }

        return best ?: drivers.first().device
    }




    private fun requestUsbPermissionForGnss(device: UsbDevice) {
        val usbManager = getSystemService(Context.USB_SERVICE) as UsbManager

        val flags = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            PendingIntent.FLAG_IMMUTABLE
        } else {
            0
        }

        pendingGnssDevice = device

        val permissionIntent = PendingIntent.getBroadcast(
            this,
            0,
            Intent(ACTION_USB_PERMISSION).setPackage(packageName),
            flags
        )

        appendLog("Requesting USB permission for device: ${device.deviceName} (${device.productName})")
        usbManager.requestPermission(device, permissionIntent)
    }

    private fun handleUsbAttachIntent(intent: Intent?): Boolean {
        if (intent?.action != UsbManager.ACTION_USB_DEVICE_ATTACHED) return false

        val device = intent.getParcelableExtra<UsbDevice>(UsbManager.EXTRA_DEVICE)
        appendLog(
            "USB attach intent delivered to activity: " +
                "${device?.deviceName} / ${device?.productName}"
        )
        maybeStartExternalGnss(preferred = device)
        return true
    }




    private var usbReceiverRegistered = false




    // Estimadores por anchor (BSSID) — evita mezclar muestras de anchors distintos
    private val originSet = AtomicBoolean(false)
    @Volatile private var enuOrigin: GpsPose? = null
    private val MAX_RANGING_APS = 10   // Android FTM supports up to ~10 concurrent APs

    @Volatile private var currentWeighting: Weighting = Weighting.WLS

    // Ventana de sincronización GPS↔RTT (ms). Configurable en runtime vía:
    //   ADB broadcast:  --es cmd "set_sync_window:1200"
    //   ROS 2 topic:    ftm/command  "set_sync_window:1200"
    // Valores típicos: 120 (precisión, requiere GPS ≥5 Hz), 1200 (tolerante, GPS 1 Hz)
    @Volatile private var syncWindowMs = 1200L

    // Buffer GNSS (thread-safe)
    private val gpsBuffer = ArrayDeque<GpsPose>()
    private val gpsLock = Any()
    private val maxGpsBuffer = 1_200   // A 10 Hz, X muestras


    // Parámetros de sincronización GNSS ↔ RTT
    private val MIN_SAMPLES_COUNT = 8
    private val MAX_GNSS_ACC_M = 15.0   // Reject GNSS fixes worse than this (metres)
    private val Z_PRIOR_SIGMA = 5.0     // σ for z-prior regularisation (metres)

    // Si el "gap" entre fixes (prev/next) es mayor, no interpolamos.
    // Se ajusta automáticamente a 2× syncWindowMs.
    private val maxInterpBracketMs: Long get() = syncWindowMs * 2

    private val MAX_SAMPLES_WINDOW_MS = 120_000L


    // Medición continua RTT
    private val rttHandler = Handler(Looper.getMainLooper())
    private val continuous = AtomicBoolean(false)
    @Volatile private var inFlight = false
    private var lastRequest: RangingRequest? = null
    @Suppress("UnusedPrivateMember")
    private var lastResponders: List<ScanResult> = emptyList()
    private var lastBuildMs: Long = 0L
    private var backoffMs: Long = 0L
    private val MIN_PERIOD_MS = 0L

    // Minimum gap between ranging bursts (ms).  Can be overridden live via
    // [updateRangingParams] to implement cooperative jitter between two OSAs.
    @Volatile private var rangingGapMs: Long = 120L
    @Volatile private var rangingPhaseMs: Long = 0L
    @Volatile private var rangingFirstBurstDone: Boolean = false
    private val RESCAN_EVERY_MS = 3_000L    // Descubrir nuevos anchors: estaba en 30_000L
    // Lo cambio a 3_000: Eso hará que needRescan(now) fuerce más a menudo
    private var lastRttUiLogMs = 0L

    // Executor único para cómputo pesado (RTT + estimador + ROS + logs)
    private val computeExec = Executors.newSingleThreadExecutor { r ->
        Thread(r, "rtt-compute").apply { isDaemon = true }
    }

    // File logger asíncrono
    private lateinit var fileLogger: FileLogger

    // Estado del publicador (para la UI)
    @Volatile private var lastPublishMs: Long = 0L
    private fun isPublishingActive(now: Long = System.currentTimeMillis()): Boolean =
        (now - lastPublishMs) <= 3_000L

    // Last time we saw valid RTT measurements (for "Signal detected" UI)
    @Volatile private var lastRttTrafficMs: Long = 0L
    private fun isRttTrafficActive(now: Long = System.currentTimeMillis()): Boolean =
        (now - lastRttTrafficMs) <= 3_000L

    // Estado UI de medición continua (solo para mostrar estado)
    private var uiContinuous by mutableStateOf(false)

    // Altura AGL del dron (m)
    // aglMeters se usa en el cómputo, aglUiMeters sólo para la UI.
    // Drone height above ground (AGL, meters). Default value; can be changed via UI / ROS 2.
    @Volatile private var aglMeters: Double = 1.5
    private var aglUiMeters by mutableDoubleStateOf(aglMeters)

    // Vertical excursion of the drone trajectory (max alt − min alt, in m) over
    // the rolling GNSS buffer. Shown in the status row when altitude_mode=auto
    // so the operator can visually check whether there is enough vertical
    // diversity for the 3D solver to recover the anchor Z.
    private var droneDzUiM by mutableDoubleStateOf(0.0)

    // RTT range bias (m). Subtracted from raw distance before gating/filter.
    @Volatile private var rangeBiasM: Double = 0.0
    private var rangeBiasUiM by mutableDoubleStateOf(rangeBiasM)


    private fun updateAglMeters(newAgl: Double) {
        aglMeters = newAgl
        aglUiMeters = newAgl
    }

    private fun updateRangeBias(newBias: Double) {
        rangeBiasM = newBias
        rangeBiasUiM = newBias
    }

    /** Update ranging period (min gap) and phase offset. Takes effect for the next burst. */
    private fun updateRangingParams(periodMs: Long, phaseMs: Long) {
        rangingGapMs = periodMs.coerceAtLeast(0L)
        rangingPhaseMs = phaseMs.coerceAtLeast(0L)
        rangingFirstBurstDone = false
        appendLog("Ranging params updated: period=${rangingGapMs}ms phase=${rangingPhaseMs}ms")
    }

    // ThreadLocal para reutilizar StringBuilder y evitar GC
    private val sbLocal = object : JThreadLocal<StringBuilder>() {
        override fun initialValue(): StringBuilder = StringBuilder(256)
    }

    private fun acquireSb(): StringBuilder {
        val sb = sbLocal.get() ?: StringBuilder(256).also { sbLocal.set(it) }
        sb.setLength(0)
        return sb
    }

    /* -------------------------------------------------------------------------
     *  Log UI + fichero
     * ---------------------------------------------------------------------- */

    data class LogItem(val id: Long, val text: String)
    private val logLines = mutableStateListOf<LogItem>()
    private val uiLogBuffer = ArrayDeque<LogItem>()
    private var logIdCounter = 0L
    private var logFlushScheduled = false
    private val LOG_FLUSH_MS = 1_000L
    private val MAX_LOG_LINES = 60

    // Activar / desactivar logs en UI (fichero siempre activo)
    private var uiLoggingEnabled = true

    private fun jsonEscape(s: String) =
        "\"" + s.replace("\\", "\\\\").replace("\"", "\\\"") + "\""

    private fun vpnIpv4AddrsJson(): String {
        val ips = mutableListOf<String>()
        try {
            val interfaces = java.net.NetworkInterface.getNetworkInterfaces()
            for (iface in java.util.Collections.list(interfaces)) {
                val isVpnLike = iface.name.contains("tun", ignoreCase = true) ||
                        iface.name.contains("zt", ignoreCase = true) ||
                        iface.displayName.contains("ZeroTier", ignoreCase = true)
                if (!iface.isUp || iface.isLoopback) continue
                for (addr in java.util.Collections.list(iface.inetAddresses)) {
                    val host = addr.hostAddress ?: continue
                    if (host.contains(':')) continue
                    if ((host.startsWith("10.148.") || isVpnLike) && host !in ips) {
                        ips.add(host)
                    }
                }
            }
        } catch (_: Throwable) {
            // Best-effort diagnostic only.
        }
        return ips.joinToString(prefix = "[", postfix = "]") { jsonEscape(it) }
    }

    private fun runOnUiThreadSafe(label: String, block: () -> Unit) {
        runOnUiThread {
            try {
                block()
            } catch (t: Throwable) {
                Log.e("MainActivity", "$label failed", t)
                appendLog("$label failed: ${t.javaClass.simpleName}: ${t.message}")
            }
        }
    }

    private fun acquireDdsMulticastLock() {
        try {
            if (ddsMulticastLock?.isHeld == true) return
            ddsMulticastLock = wifiManager.createMulticastLock("osa-ftm-fastdds-discovery").apply {
                setReferenceCounted(false)
                acquire()
            }
            appendLog("DDS multicast lock acquired for FastDDS discovery")
        } catch (t: Throwable) {
            Log.w("MainActivity", "Cannot acquire DDS multicast lock", t)
            appendLog("Cannot acquire DDS multicast lock: ${t.javaClass.simpleName}: ${t.message}")
        }
    }

    private fun releaseDdsMulticastLock() {
        try {
            if (ddsMulticastLock?.isHeld == true) {
                ddsMulticastLock?.release()
            }
        } catch (t: Throwable) {
            Log.w("MainActivity", "Cannot release DDS multicast lock", t)
        } finally {
            ddsMulticastLock = null
        }
    }

    private fun writeStatusJsonForAdb() {
        try {
            val eg = externalGnss
            val ntripOk = eg?.isNtripConnected == true
            val fix = eg?.lastFixQuality ?: -1
            val sats = eg?.lastSatelliteCount ?: -1
            val hdop = eg?.lastHdop ?: -1.0
            val rtcmBytes = eg?.rtcmBytesReceived ?: 0L
            val extActive = externalGnssActive
            val cfg = ntripConfig
            val rosNs = activeRosNamespaceCore.ifBlank { buildRosNamespaceCore() }
            val vpnIps = vpnIpv4AddrsJson()
            val ros = ros2
            val json = buildString {
                append("{")
                append("\"agent_id\":${jsonEscape(currentAgentId())},")
                append("\"ros_namespace\":${jsonEscape(rosNs)},")
                append("\"ros_node_alive\":${ros != null},")
                append("\"device_model\":${jsonEscape(android.os.Build.MODEL)},")
                append("\"vpn_ips\":$vpnIps,")
                append("\"dds_multicast_lock_held\":${ddsMulticastLock?.isHeld == true},")
                append("\"ros_rtt_pub_count\":${ros?.rttPublishCount ?: 0L},")
                append("\"ros_location_pub_count\":${ros?.locationPublishCount ?: 0L},")
                append("\"ros_anchor_pub_count\":${ros?.anchorPublishCount ?: 0L},")
                append("\"ros_publish_failure_count\":${ros?.publishFailureCount ?: 0L},")
                append("\"ros_last_rtt_pub_ms\":${ros?.lastRttPublishWallMs ?: 0L},")
                append("\"ros_last_location_pub_ms\":${ros?.lastLocationPublishWallMs ?: 0L},")
                append("\"ros_last_anchor_pub_ms\":${ros?.lastAnchorPublishWallMs ?: 0L},")
                append("\"ros_last_publish_error_ms\":${ros?.lastPublishErrorWallMs ?: 0L},")
                append("\"ros_last_publish_error\":${jsonEscape(ros?.lastPublishError ?: "")},")
                append("\"ntrip_connected\":$ntripOk,")
                append("\"fix_quality\":$fix,")
                append("\"fix_label\":${jsonEscape(gnssRtkStatus.fixLabel)},")
                append("\"satellites\":$sats,")
                append("\"hdop\":$hdop,")
                append("\"rtcm_bytes\":$rtcmBytes,")
                append("\"external_gnss_active\":$extActive,")
                append("\"sync_window_ms\":$syncWindowMs,")
                append("\"weighting\":${jsonEscape(currentWeighting.toString())},")
                append("\"ntrip_host\":${jsonEscape(cfg?.ntripHost ?: "")},")
                append("\"ntrip_port\":${cfg?.ntripPort ?: 2101},")
                append("\"ntrip_mountpoint\":${jsonEscape(cfg?.mountpoint ?: "")},")
                append("\"ntrip_user\":${jsonEscape(cfg?.ntripUser ?: "")},")
                append("\"continuous_ranging\":$uiContinuous")
                append("}")
            }
            val dir = getExternalFilesDir(null) ?: filesDir
            val f = java.io.File(dir, "status.json")
            f.writeText(json)
            appendLog("STATUS written to ${f.absolutePath}")
        } catch (t: Throwable) {
            Log.e("MainActivity", "Cannot write status.json", t)
            appendLog("Cannot write status.json: ${t.javaClass.simpleName}: ${t.message}")
        }
    }

    private fun installCrashReporter() {
        val previous = Thread.getDefaultUncaughtExceptionHandler()
        Thread.setDefaultUncaughtExceptionHandler { thread, throwable ->
            runCatching { writeCrashReport(thread.name, throwable) }
            if (previous != null) {
                previous.uncaughtException(thread, throwable)
            } else {
                kotlin.system.exitProcess(2)
            }
        }
    }

    private fun writeCrashReport(threadName: String, throwable: Throwable) {
        val sw = java.io.StringWriter()
        throwable.printStackTrace(java.io.PrintWriter(sw))
        val dir = getExternalFilesDir(null) ?: filesDir
        val report = java.io.File(dir, "crash_last.txt")
        report.writeText(
            buildString {
                append("time_ms=").append(System.currentTimeMillis()).append('\n')
                append("thread=").append(threadName).append('\n')
                append("agent_id=").append(currentAgentId()).append('\n')
                append("ros_namespace=").append(activeRosNamespaceCore).append('\n')
                append("vpn_ips=").append(vpnIpv4AddrsJson()).append('\n')
                append("exception=").append(throwable.javaClass.name).append(": ")
                    .append(throwable.message).append("\n\n")
                append(sw.toString())
            }
        )
    }

    /**
     * Agent identifier used as "agent_id" field in every JSONL record.
     * Prefers the explicit [ExperimentConfig.agentId] when set, otherwise
     * falls back to the device-derived ROS namespace core.
     */
    private fun currentAgentId(): String {
        val explicit = ExperimentConfig.sanitizeAgentId(experimentConfig.agentId)
        return if (explicit.isNotBlank()) explicit else buildRosNamespaceCore()
    }

    private fun appendLog(msg: String, alsoToUi: Boolean = true) {

        Log.d("MainActivity", msg)

        // Push status to background service notification
        bgService?.let { svc ->
            svc.statusLine = msg
            svc.rttCount = rttCount.get()
            svc.ntripConnected = externalGnss?.isNtripConnected == true
            svc.gnssSource = if (externalGnssActive) "External" else if (gnssActive) "Internal" else "—"
        }
        // Update observable GNSS/RTK status for Compose UI
        externalGnss?.let { eg ->
            gnssRtkStatus = GnssRtkStatus(
                fixQuality = eg.lastFixQuality,
                numSatellites = eg.lastSatelliteCount,
                hdop = eg.lastHdop,
                rtcmBytesReceived = eg.rtcmBytesReceived,
                ntripConnected = eg.isNtripConnected,
                externalGnssActive = externalGnssActive
            )
        }
        val safe = if (msg.length > 1_000) msg.take(1_000) + " …" else msg
        val ts = System.currentTimeMillis()
        val line = "[$ts] $safe"
        val item = LogItem(++logIdCounter, line)

        // Siempre a fichero
        if (::fileLogger.isInitialized) {
            fileLogger.appendTaggedJson(
                "app.jsonl",
                "app",
                "{\"msg\":${jsonEscape(msg)},\"t_ms\":$ts}"
            )
        }

        // UI opcional
        if (!uiLoggingEnabled || !alsoToUi) return

        synchronized(uiLogBuffer) {
            uiLogBuffer.addLast(item)
            while (uiLogBuffer.size > MAX_LOG_LINES) uiLogBuffer.removeFirst()
        }
        if (!logFlushScheduled) {
            logFlushScheduled = true
            rttHandler.postDelayed({
                synchronized(uiLogBuffer) {
                    while (uiLogBuffer.isNotEmpty()) {
                        logLines.add(uiLogBuffer.removeFirst())
                    }
                }
                val overflow = logLines.size - MAX_LOG_LINES
                repeat(overflow.coerceAtLeast(0)) { logLines.removeAt(0) }
                logFlushScheduled = false
            }, LOG_FLUSH_MS)
        }
    }

    /* -------------------------------------------------------------------------
     *  Permisos
     * ---------------------------------------------------------------------- */

    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { grants ->
        val fine = grants[Manifest.permission.ACCESS_FINE_LOCATION] ?: false
        val coarse = grants[Manifest.permission.ACCESS_COARSE_LOCATION] ?: false
        val nearby = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU)
            (grants[Manifest.permission.NEARBY_WIFI_DEVICES] ?: false) else true

        if (fine || coarse) {
            appendLog("Location permits granted.")
            startAndBindBackgroundServiceIfPossible("Permissions granted")
            maybeStartInternalGnssFallback("Permissions granted")
        }
        if (fine && nearby) {
            appendLog("Wi-Fi RTT permissions granted.")
        }
    }

    private fun hasAllWifiPerms(): Boolean {
        val fineOk = ActivityCompat.checkSelfPermission(
            this,
            Manifest.permission.ACCESS_FINE_LOCATION
        ) == PackageManager.PERMISSION_GRANTED

        val coarseOk = ActivityCompat.checkSelfPermission(
            this,
            Manifest.permission.ACCESS_COARSE_LOCATION
        ) == PackageManager.PERMISSION_GRANTED

        val nearbyOk = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            ActivityCompat.checkSelfPermission(
                this,
                Manifest.permission.NEARBY_WIFI_DEVICES
            ) == PackageManager.PERMISSION_GRANTED
        } else true

        return fineOk && coarseOk && nearbyOk
    }

    private fun hasRuntimeLocationPermission(): Boolean {
        val fineOk = ActivityCompat.checkSelfPermission(
            this,
            Manifest.permission.ACCESS_FINE_LOCATION
        ) == PackageManager.PERMISSION_GRANTED
        val coarseOk = ActivityCompat.checkSelfPermission(
            this,
            Manifest.permission.ACCESS_COARSE_LOCATION
        ) == PackageManager.PERMISSION_GRANTED
        return fineOk || coarseOk
    }

    private fun startAndBindBackgroundServiceIfPossible(reason: String) {
        if (serviceBound) return
        if (!hasRuntimeLocationPermission()) {
            appendLog("$reason → foreground service deferred until location permission is granted")
            return
        }

        val serviceIntent = Intent(this, MyBackgroundService::class.java)
        try {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                startForegroundService(serviceIntent)
            } else {
                startService(serviceIntent)
            }
            bindService(serviceIntent, serviceConnection, BIND_AUTO_CREATE)
            serviceBound = true
            appendLog("Foreground acquisition service started ($reason)")
        } catch (t: Throwable) {
            appendLog("Cannot start foreground service ($reason): ${t.javaClass.simpleName}: ${t.message}")
            Log.e("MainActivity", "Cannot start foreground service", t)
        }
    }

    private fun executeCompute(taskName: String, block: () -> Unit) {
        try {
            computeExec.execute {
                try {
                    block()
                } catch (t: Throwable) {
                    Log.e("MainActivity", "Compute task failed: $taskName", t)
                    appendLog("Compute task failed ($taskName): ${t.javaClass.simpleName}: ${t.message}", alsoToUi = false)
                    if (taskName == "processRttMeasurement") {
                        inFlight = false
                    }
                }
            }
        } catch (t: Throwable) {
            Log.e("MainActivity", "Cannot schedule compute task: $taskName", t)
            appendLog("Cannot schedule compute task ($taskName): ${t.javaClass.simpleName}: ${t.message}", alsoToUi = false)
            if (taskName == "processRttMeasurement") {
                inFlight = false
            }
        }
    }

    private fun requestNeededPermissions() {
        val perms = mutableListOf(
            Manifest.permission.ACCESS_FINE_LOCATION,
            Manifest.permission.ACCESS_COARSE_LOCATION,
            Manifest.permission.ACCESS_WIFI_STATE,
            Manifest.permission.CHANGE_WIFI_STATE,
            Manifest.permission.INTERNET
        )
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            perms += Manifest.permission.NEARBY_WIFI_DEVICES
            perms += Manifest.permission.POST_NOTIFICATIONS
        }
        permissionLauncher.launch(perms.toTypedArray())
    }

    private fun loadConfigOnStartup() {
        // Puedes usar /sdcard/Download/ntrip_config.txt o el directorio de la app
        val cfgFile = getExternalFilesDir(null)?.resolve("ntrip_config.txt")

        if (cfgFile == null || !cfgFile.exists()) {
            appendLog("ntrip_config.txt not found in ${cfgFile?.absolutePath ?: "null"}")
            return
        }

        try {
            val map = mutableMapOf<String, String>()
            cfgFile.forEachLine { rawLine ->
                val line = rawLine.trim()
                if (line.isEmpty() || line.startsWith("#")) return@forEachLine
                val kv = line.split("=", limit = 2)
                if (kv.size == 2) {
                    map[kv[0].trim()] = kv[1].trim()
                }
            }
            ntripConfig = Config.from(map)
            appendLog("NTRIP config loadad: host=${ntripConfig?.ntripHost}, mp=${ntripConfig?.mountpoint}")
        } catch (e: Exception) {
            appendLog("Error: cannot read NTRIP config: ${e.message}")
        }
    }


    private fun copyConfigFromDownloadIfNeeded() {
        val downloads = android.os.Environment.getExternalStoragePublicDirectory(
            android.os.Environment.DIRECTORY_DOWNLOADS
        )
        val downloadFile = java.io.File(downloads, "ntrip_config.txt")
        val internalFile = getExternalFilesDir(null)?.resolve("ntrip_config.txt") ?: return

        if (!internalFile.exists() && downloadFile.exists()) {
            try {
                downloadFile.copyTo(internalFile, overwrite = true)
                appendLog("ntrip_config.txt copied to  ${internalFile.absolutePath}")
            } catch (e: Exception) {
                appendLog("Error: cannot copy ntrip_config.txt: ${e.message}")
            }
        }
    }


    /* -------------------------------------------------------------------------
     *  Persistencia de MACs
     * ---------------------------------------------------------------------- */

    private val PREFS = "rtt_prefs"
    private val KEY_MACS = "mac_list_v1"

    private fun loadMacs(): MutableList<String> {
        val p = getSharedPreferences(PREFS, Context.MODE_PRIVATE)
        val raw = p.getStringSet(KEY_MACS, null)
        return if (raw != null) {
            raw.map { it.lowercase() }.sorted().toMutableList()
        } else {
            mutableListOf(
                "aa:bb:cc:dd:ee:ff",
                "00:11:22:33:44:55",
                "12:34:56:78:9a:bc",
                "9c:4f:5f:0f:da:ab"
            )
        }
    }

    private fun saveMacs(list: List<String>) {
        val clean = list.map { it.lowercase() }.toSet()
        getSharedPreferences(PREFS, Context.MODE_PRIVATE)
            .edit()
            .putStringSet(KEY_MACS, clean)
            .apply()
    }

    /* -------------------------------------------------------------------------
     *  Ciclo de vida Activity
     * ---------------------------------------------------------------------- */

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        installCrashReporter()


        wifiManager = applicationContext.getSystemService(Context.WIFI_SERVICE) as WifiManager
        rttManager = applicationContext.getSystemService(Context.WIFI_RTT_RANGING_SERVICE) as? WifiRttManager
        acquireDdsMulticastLock()

        ContextCompat.registerReceiver(
            this,
            rttStateReceiver,
            IntentFilter(WifiRttManager.ACTION_WIFI_RTT_STATE_CHANGED),
            ContextCompat.RECEIVER_NOT_EXPORTED
        )
        rttStateReceiverRegistered = true

        @Suppress("DEPRECATION")
        ContextCompat.registerReceiver(
            this,
            scanResultsReceiver,
            IntentFilter(WifiManager.SCAN_RESULTS_AVAILABLE_ACTION),
            ContextCompat.RECEIVER_NOT_EXPORTED
        )
        scanResultsReceiverRegistered = true

        // Logs
        fileLogger = FileLogger(this)
        appendLog("Logs en: ${fileLogger.logsDirPath()}")
        startAndBindBackgroundServiceIfPossible("App startup")

        // Config NTRIP
        copyConfigFromDownloadIfNeeded()
        loadConfigOnStartup()
        // Also try loading from SharedPreferences if file-based config was not found
        if (ntripConfig == null) {
            ntripConfig = Config.loadFromPrefs(applicationContext)
            if (ntripConfig != null) {
                appendLog("NTRIP config loaded from SharedPreferences: host=${ntripConfig?.ntripHost}")
            }
        }

        // Register ADB config broadcast receiver
        ContextCompat.registerReceiver(
            this,
            configBroadcastReceiver,
            IntentFilter(ACTION_NTRIP_CONFIG),
            ContextCompat.RECEIVER_EXPORTED
        )
        configReceiverRegistered = true

        // Load experiment config & ground truth
        experimentConfig = ExperimentConfig.loadFromPrefs(applicationContext)
        activeFilterMode = experimentConfig.filterMode
        if (experimentConfig.label.isNotBlank()) {
            appendLog("Experiment config loaded: label=${experimentConfig.label} agl=${experimentConfig.aglMeters}m bias=${experimentConfig.rangeBiasM}m")
            updateAglMeters(experimentConfig.aglMeters)
            updateRangeBias(experimentConfig.rangeBiasM)
        }
        val nGt = groundTruth.loadFromFile(applicationContext)
        val nGtPrefs = groundTruth.loadFromPrefs(applicationContext)
        if (nGt + nGtPrefs > 0) {
            appendLog("Ground truth loaded: ${groundTruth.size} APs (${nGt} from file, ${nGtPrefs} from prefs)")
        }

        // Register experiment / ground-truth ADB receivers
        ContextCompat.registerReceiver(
            this, experimentConfigReceiver,
            IntentFilter(ACTION_EXPERIMENT_CONFIG),
            ContextCompat.RECEIVER_EXPORTED
        )
        ContextCompat.registerReceiver(
            this, groundTruthReceiver,
            IntentFilter(ACTION_GROUND_TRUTH),
            ContextCompat.RECEIVER_EXPORTED
        )
        ContextCompat.registerReceiver(
            this, commandReceiver,
            IntentFilter(ACTION_REMOTE_CMD),
            ContextCompat.RECEIVER_EXPORTED
        )

        // GNSS externo USB, ahora con NTRIP...
        externalGnss = ExternalGnssManager(
            context = applicationContext,
            onFix = { lat, lon, alt, accM -> onExternalGnssFix(lat, lon, alt, accM) },
            onStatus = { msg -> appendLog("External GNSS: $msg") },
            onActiveChanged = { isActive ->
                externalGnssActive = isActive
                if (!isActive) {
                    markExternalGnssStopped()
                    runOnUiThread {
                        maybeStartInternalGnssFallback("External GNSS stopped")
                    }
                } else {
                    runOnUiThread {
                        if (gnssActive) {
                            appendLog("External GNSS active → stopping internal GNSS")
                            stopLocationUpdates()
                        }
                    }
                }
            },
            ntripConfig = ntripConfig
        )

// 1) Receivers
        registerReceiver(usbAttachReceiver, IntentFilter(UsbManager.ACTION_USB_DEVICE_ATTACHED))
        usbReceiverRegistered = true

        ContextCompat.registerReceiver(
            this,
            usbPermissionReceiver,
            IntentFilter(ACTION_USB_PERMISSION),
            ContextCompat.RECEIVER_NOT_EXPORTED
        )
        usbPermissionReceiverRegistered = true

        // 2) Intento inicial: si ya hay permiso, arrancamos directamente, si no, pedimos permiso
        if (!handleUsbAttachIntent(intent)) {
            maybeStartExternalGnss()
        }


        // GNSS interno (Fused)
        fused = LocationServices.getFusedLocationProviderClient(this)
        locReq = LocationRequest.Builder(
            Priority.PRIORITY_HIGH_ACCURACY,
            1_000L      // 1 Hz aprox para el GNSS interno
        )
            .setWaitForAccurateLocation(true)
            .build()

        // ROS 2: namespace por dispositivo (Bluetooth name o modelo)
        val nsCore = buildRosNamespaceCore()
        activeRosNamespaceCore = nsCore
        val nodeName = "android_rtt_node_${getOrCreateDeviceId()}"

        ros2 = try {
            Ros2RttNode(nodeName, nsCore).apply {

            // AGL updates from ROS 2 topic /<nsCore>/ftm/set_agl
            setOnAglChangeCallback { newAgl ->
                this@MainActivity.runOnUiThread {
                    if (experimentConfig.altitudeMode != ExperimentConfig.ALT_MODE_MANUAL_AGL) {
                        appendLog(
                            "AGL change from ROS 2 (%.1f m) ignored: altitude_mode=auto. " +
                                    "Switch to manual_agl if you want to seed AGL."
                                        .format(newAgl)
                        )
                        return@runOnUiThread
                    }
                    val oldAgl = aglMeters
                    updateAglMeters(newAgl)

                    if (originSet.get()) {
                        appendLog(
                            "AGL changed from %.1f m to %.1f m (from ROS 2). " +
                                    "Resetting estimator to apply new AGL."
                                        .format(oldAgl, newAgl)
                        )
                        // Automatic reset so the new AGL is actually used
                        resetEstimator()
                    } else {
                        appendLog(
                            "AGL changed from %.1f m to %.1f m (from ROS 2). " +
                                    "ENU origin is not fixed yet; new AGL will be used when it is."
                                        .format(oldAgl, newAgl)
                        )
                    }
                }
            }


            setOnStartExperimentCallback {
                runOnUiThread {
                    startExperiment("ros_start_experiment", "")
                    appendLog("Servicio ROS2 /$nsCore/ftm/start_experiment recibido. Iniciando medición continua (todos los anchors)…")
                    stopContinuousRanging()
                    startContinuousRanging("")
                    uiContinuous = true
                }
            }
            setOnStopExperimentCallback {
                runOnUiThread {
                    stopExperiment("ros_stop_experiment")
                    appendLog("Servicio ROS2 /$nsCore/ftm/stop_experiment recibido. Deteniendo medición continua…")
                    stopContinuousRanging()
                    uiContinuous = false
                }
            }

            // NUEVO: cambio de ponderación vía servicios ftm/set_weighting_*
            setOnWeightingChangeCallback { methodName ->
                val w: Weighting = when (methodName.uppercase()) {
                    "WLS" -> Weighting.WLS
                    "HUBER" -> Weighting.Huber(delta = 1.345)      // mismo valor por defecto que en la UI
                    "TRIM" -> Weighting.Trim(fraction = 0.20)
                    "SIGMACLIP" -> Weighting.SigmaClip(k = 3.0)
                    else -> {
                        appendLog("Service set_weighting: unknown method '$methodName'")
                        return@setOnWeightingChangeCallback
                    }
                }

                // Opcional pero recomendable: obligar a cambiar con las medidas paradas
                if (uiContinuous) {
                    appendLog("Warning: set_weighting was received while continuos measurement was active. Better: stop → set_weighting → start.")
                }

                onWeightingChanged(w)
                appendLog("Weighting method changed via ROS2 to: $w")
            }

            setOnSyncWindowChangeCallback { windowMs ->
                syncWindowMs = windowMs
                appendLog("Sync window changed via ROS2 to: ${windowMs}ms")
            }

            // Cooperative OSA: peer GNSS and peer RTT subscribers (ROS 2).
            setOnPeerLocationCallback { lat, lon, alt, accM, tMs ->
                handlePeerLocation(lat, lon, alt, accM, tMs)
            }
            setOnPeerRttCallback { json ->
                handlePeerRttJson(json)
            }
            }
        } catch (t: Throwable) {
            appendLog("ROS2 init failed: ${t.javaClass.simpleName}: ${t.message}")
            Log.e("MainActivity", "ROS2 init failed", t)
            null
        }

        // Apply initial ranging period / phase from config.
        updateRangingParams(experimentConfig.rangingPeriodMs, experimentConfig.rangingPhaseMs)

        // If the operator has already configured a peer agent + cooperative
        // mode, start the peer subscriptions right away.
        if (experimentConfig.cooperativeMode != ExperimentConfig.COOP_INDEPENDENT &&
            experimentConfig.peerAgentId.isNotBlank()
        ) {
            ros2?.startPeerSubscriptions(experimentConfig.peerAgentId)
            appendLog(
                "Cooperative OSA enabled: mode=${experimentConfig.cooperativeMode} " +
                        "peer=${experimentConfig.peerAgentId}"
            )
        }

        appendLog("ROS2 initiated. Namespace: /$nsCore")
        appendLog("Active ROS2 services: /$nsCore/ftm/start_experiment y /$nsCore/ftm/stop_experiment")

        // Pide permisos al arrancar si faltan
        if (!hasAllWifiPerms()) {
            appendLog("Wi-Fi/location permission not allowed. Trying to connect…")
            requestNeededPermissions()
        } else {
            maybeStartInternalGnssFallback("App startup")
        }

        // UI Compose
        setContent {
            AppUi()
        }

        appendLog("App started. RTT/GNSS permission: ${hasAllWifiPerms()}")
    }

    override fun onNewIntent(intent: Intent) {
        super.onNewIntent(intent)
        setIntent(intent)
        handleUsbAttachIntent(intent)
    }


    override fun onResume() {
        super.onResume()
        // Retry external GNSS: the USB device may have been plugged in while the
        // app was in background, or permission may have just been granted. The
        // call is idempotent thanks to started.compareAndSet in ExternalGnssManager.
        if (externalGnss != null && externalGnssActive.not()) {
            maybeStartExternalGnss()
        }
        if (hasAllWifiPerms()) {
            maybeStartInternalGnssFallback("App resume")
        }
    }

    override fun onPause() {
        super.onPause()
        //stopContinuousRanging()
        //stopLocationUpdates()

        // sólo flush de logs:
        fileLogger.flushAll()
    }

    override fun onTrimMemory(level: Int) {
        super.onTrimMemory(level)
        if (level >= ComponentCallbacks2.TRIM_MEMORY_UI_HIDDEN) {
            fileLogger.flushAll()
        }
    }




    override fun onDestroy() {
        stopExperiment("activity_destroy")
        super.onDestroy()
        if (rttStateReceiverRegistered) {
            runCatching { unregisterReceiver(rttStateReceiver) }
            rttStateReceiverRegistered = false
        }

        if (scanResultsReceiverRegistered) {
            runCatching { unregisterReceiver(scanResultsReceiver) }
            scanResultsReceiverRegistered = false
        }

        rttStateReceiverRegistered = false
        stopLocationUpdates()
        stopContinuousRanging()

        externalGnss?.stop()
        if (usbReceiverRegistered) {
            runCatching { unregisterReceiver(usbAttachReceiver) }
            usbReceiverRegistered = false
        }

        if (usbPermissionReceiverRegistered) {
            runCatching { unregisterReceiver(usbPermissionReceiver) }
            usbPermissionReceiverRegistered = false
        }

        if (configReceiverRegistered) {
            runCatching { unregisterReceiver(configBroadcastReceiver) }
            configReceiverRegistered = false
        }

        runCatching { unregisterReceiver(experimentConfigReceiver) }
        runCatching { unregisterReceiver(groundTruthReceiver) }
        runCatching { unregisterReceiver(commandReceiver) }

        computeExec.shutdown()
        fileLogger.close()
        ros2?.dispose()
        releaseDdsMulticastLock()

        if (serviceBound) {
            unbindService(serviceConnection)
            serviceBound = false
        }
    }



    /* -------------------------------------------------------------------------
     *  UI Compose
     * ---------------------------------------------------------------------- */

    @Composable
    private fun AppUi() {
        MaterialTheme {
            val macList = remember { mutableStateListOf<String>().also { it.addAll(loadMacs()) } }
            var bssidText by remember { mutableStateOf(TextFieldValue("")) }
            var dropdownExpanded by remember { mutableStateOf(false) }

            // Intro desplegable
            var introExpanded by remember { mutableStateOf(false) }
            val caretRotation by animateFloatAsState(
                targetValue = if (introExpanded) 180f else 0f,
                label = "intro-caret"
            )

            val pubActive by remember { derivedStateOf { isPublishingActive() } }
            val rttActive by remember { derivedStateOf { isRttTrafficActive() } }

            val pubColor = when {
                pubActive -> MaterialTheme.colorScheme.primary
                rttActive -> MaterialTheme.colorScheme.tertiary
                uiContinuous -> MaterialTheme.colorScheme.secondary
                else -> MaterialTheme.colorScheme.error
            }


            // Método de ponderación seleccionado
            var selectedWeighting by remember { mutableStateOf<Weighting>(currentWeighting) }

            // Expone el BSSID actual para targetBssid()
            LaunchedEffect(bssidText.text) {
                currentBssidGetter = { bssidText.text }
            }

            Scaffold(
                bottomBar = {
                    BottomAppBar(
                        modifier = Modifier.navigationBarsPadding()
                    ) {
                        Row(
                            Modifier
                                .fillMaxWidth()
                                .padding(horizontal = 12.dp),
                            horizontalArrangement = Arrangement.spacedBy(8.dp)
                        ) {
                            Button(
                                onClick = { onScanAndRange(bssidText.text) },
                                modifier = Modifier.weight(1f),
                                enabled = !uiContinuous && hasAllWifiPerms()
                            ) { Text("Single ranging") }

                            Button(
                                onClick = {
                                    startExperiment("ui_start", bssidText.text.trim().lowercase())
                                    startContinuousRanging(bssidText.text)
                                    uiContinuous = true

                                },
                                modifier = Modifier.weight(1f),
                                colors = ButtonDefaults.buttonColors(
                                    containerColor = if (uiContinuous)
                                        MaterialTheme.colorScheme.primary
                                    else
                                        MaterialTheme.colorScheme.secondary
                                ),
                                enabled = hasAllWifiPerms()
                            ) { Text(if (uiContinuous) "Ranging…" else "Continuous ranging"
                            ) }

                            OutlinedButton(
                                onClick = {
                                    stopExperiment("ui_stop")
                                    stopContinuousRanging()
                                    uiContinuous = false

                                },
                                modifier = Modifier.weight(1f)
                            ) { Text("Stop") }
                        }
                    }
                }
            ) { innerPadding ->
                LazyColumn(
                    modifier = Modifier
                        .fillMaxSize()
                        .padding(innerPadding)
                        .systemBarsPadding()
                        .imePadding(),
                    contentPadding = PaddingValues(16.dp),
                    verticalArrangement = Arrangement.spacedBy(12.dp)
                ) {
                    // 1) Ponderación robusta
                    item {
                        ElevatedCard(Modifier.fillMaxWidth()) {
                            Column(Modifier.padding(12.dp)) {
                                Text(
                                    "Robust weighting",
                                    style = MaterialTheme.typography.titleMedium
                                )
                                Spacer(Modifier.height(8.dp))
                                WeightingSelector(
                                    onWeightingChange = { w ->
                                        selectedWeighting = w
                                        onWeightingChanged(w)
                                    }
                                )

// GNSS / AGL status row (always visible)
                                Spacer(Modifier.height(6.dp))
                                Row(
                                    Modifier.fillMaxWidth(),
                                    horizontalArrangement = Arrangement.SpaceBetween
                                ) {
                                    val gnssText = if (!originSet.get()) {
                                        "Waiting for GNSS to set ENU origin…"
                                    } else {
                                        "ENU origin fixed"
                                    }

                                    AssistChip(
                                        onClick = {},
                                        label = {
                                            Text(
                                                gnssText,
                                                style = MaterialTheme.typography.bodySmall
                                            )
                                        }
                                    )

                                    Text(
                                        text = if (experimentConfig.altitudeMode == ExperimentConfig.ALT_MODE_MANUAL_AGL)
                                            "AGL = ${"%.1f".format(aglUiMeters)} m"
                                        else
                                            "ΔZ drone = ${"%.1f".format(droneDzUiM)} m",
                                        style = MaterialTheme.typography.bodyMedium
                                    )
                                }
                            }
                        }
                    }

                    // 2) Intro
                    item {
                        ElevatedCard(
                            modifier = Modifier.fillMaxWidth(),
                            onClick = { introExpanded = !introExpanded }
                        ) {
                            Row(
                                Modifier
                                    .fillMaxWidth()
                                    .padding(12.dp),
                                horizontalArrangement = Arrangement.SpaceBetween
                            ) {
                                Column(Modifier.weight(1f)) {
                                    Text(
                                        "Wi-Fi RTT + GNSS → Anchor estimation",
                                        style = MaterialTheme.typography.titleMedium
                                    )
                                    Text(
                                        "Measures FTM distances and publishes them via ROS 2; refines the AP position using GNSS.",
                                        style = MaterialTheme.typography.bodyMedium
                                    )
                                }
                                Icon(
                                    imageVector = Icons.Default.ExpandMore,
                                    contentDescription = null,
                                    modifier = Modifier.rotate(caretRotation)
                                )
                            }
                            AnimatedVisibility(visible = introExpanded) {
                                Column(
                                    Modifier.padding(
                                        horizontal = 12.dp,
                                        vertical = 8.dp
                                    )
                                ) {
                                    Text("• Use “Continuous ranging” to sample at the maximum allowed rate.")
                                    Text("• Select a BSSID from the list or add your own to filter.")
                                    Text("• The status indicator shows whether there were recent publishes (3 s).")

                                }
                            }
                        }
                    }

                    // 3) Logs
                    item {
                        Box(
                            modifier = Modifier
                                .fillMaxWidth()
                                .heightIn(min = 120.dp, max = 220.dp)
                        ) {
                            LazyColumn(Modifier.fillMaxSize()) {
                                items(
                                    items = logLines,
                                    key = { it.id },
                                    contentType = { "log" }
                                ) { item ->
                                    Text(item.text)
                                    Spacer(Modifier.height(2.dp))
                                }
                            }
                        }
                    }

                    // 4) Soporte y estado publisher
                    item {
                        Row(
                            Modifier.fillMaxWidth(),
                            horizontalArrangement = Arrangement.SpaceBetween
                        ) {
                            Text("RTT support: ${isRttSupported()}")
                            Surface(
                                color = pubColor,
                                contentColor = MaterialTheme.colorScheme.onPrimary,
                                tonalElevation = 2.dp,
                                shadowElevation = 2.dp,
                                shape = MaterialTheme.shapes.small
                            ) {
                                Text(
                                    text = when {
                                        pubActive -> "Publishing"
                                        rttActive -> "Signal detected"
                                        uiContinuous -> "No traffic"
                                        else -> "Stopped"
                                    },
                                    modifier = Modifier.padding(
                                        horizontal = 10.dp,
                                        vertical = 6.dp
                                    )
                                )

                            }
                        }
                    }

                    // 5) Selector de BSSID
                    item {
                        MacSelector(
                            macList = macList,
                            text = bssidText,
                            onTextChange = { bssidText = it },
                            expanded = dropdownExpanded,
                            onExpandedChange = { dropdownExpanded = it },
                            onPick = { picked ->
                                bssidText = TextFieldValue(picked)
                                dropdownExpanded = false
                            },
                            onAdd = {
                                val mac = bssidText.text.trim().lowercase()
                                if (mac.isNotBlank() && macRegex.matches(mac) && !macList.contains(
                                        mac
                                    )
                                ) {
                                    macList.add(mac)
                                    saveMacs(macList)
                                    appendLog("MAC added: $mac")
                                } else {
                                    appendLog("Invalid or duplicated MAC: $mac")
                                }
                            },
                            onDelete = { mac ->
                                macList.remove(mac)
                                saveMacs(macList)
                                appendLog("MAC removed: $mac")
                            }
                        )
                    }

                    // 6) Reset estimación
                    item {
                        OutlinedButton(
                            onClick = { resetEstimator() },
                            modifier = Modifier.fillMaxWidth()
                        ) {
                            Text("Reset estimator")
                        }
                    }

                    // 7) Pre-flight experiment config
                    item {
                        PreFlightConfigCard(
                            config = experimentConfig,
                            groundTruthCount = groundTruth.size,
                            onConfigChanged = { cfg ->
                                experimentConfig = cfg
                                ExperimentConfig.saveToPrefs(applicationContext, cfg)
                                updateAglMeters(cfg.aglMeters)
                                updateRangeBias(cfg.rangeBiasM)
                                applyFilterMode(cfg.filterMode)
                                appendLog("Experiment config updated: label=${cfg.label} agl=${cfg.aglMeters}m bias=${cfg.rangeBiasM}m filter=${cfg.filterMode.name}")
                            }
                        )
                    }

                    // 8) NTRIP RTK configuration
                    item {
                        NtripConfigCard(
                            initialConfig = ntripConfig,
                            isConnected = ntripConnected,
                            rtkStatus = gnssRtkStatus,
                            onConnect = { cfg ->
                                applyNtripConfig(cfg)
                                ntripConnected = true
                            },
                            onDisconnect = { disconnectNtrip() }
                        )
                    }

                    item { Spacer(Modifier.height(8.dp)) }
                }
            }
        }
    }

    /* -------------------------------------------------------------------------
     *  Wi-Fi RTT
     * ---------------------------------------------------------------------- */
    private fun isRttSupported(): Boolean {
        return packageManager.hasSystemFeature(PackageManager.FEATURE_WIFI_RTT) &&
                (rttManager != null)
    }

    /**
     * Construye el namespace ROS2 a partir del nombre Bluetooth o modelo.
     * Resultado típico: "pixel7_pro", "poco_x7_pro", etc.
     * Se usa como prefix en Ros2RttNode (no lleva barra inicial).
     */


    private fun getOrCreateDeviceId(): String {
        val p = getSharedPreferences("rtt_prefs", MODE_PRIVATE)
        val key = "device_id_v1"
        val existing = p.getString(key, null)
        if (!existing.isNullOrBlank()) return existing
        val fresh = java.util.UUID.randomUUID().toString().substring(0, 8)
        p.edit().putString(key, fresh).apply()
        return fresh
    }

    private fun buildRosNamespaceCore(): String {
        // If the operator has set an explicit agent_id via ADB
        // (EXPERIMENT_CONFIG --es agent_id ...), use it as the ROS 2
        // namespace core.  This keeps cooperative experiments predictable:
        //   /osa1/ftm_rtt, /osa2/phone/location, ...
        val explicit = ExperimentConfig.sanitizeAgentId(experimentConfig.agentId)
        if (explicit.isNotBlank()) return explicit

        val model = android.os.Build.MODEL
            .lowercase()
            .replace("[^a-z0-9_]".toRegex(), "_")
            .trim('_')
            .ifBlank { "phone" }

        return "${model}_${getOrCreateDeviceId()}"
    }



    @SuppressLint("MissingPermission")
    private fun onScanAndRange(bssidInput: String) {
        if (!hasAllWifiPerms()) {
            appendLog("Wi-Fi/location are not allowed. Requesting…")
            requestNeededPermissions()
            return
        }
        if (!isRttSupported()) {
            appendLog("Wi-Fi RTT (802.11mc) is not supported.")
            return
        }

        val scanResults = wifiManager.scanResults.orEmpty()
        if (scanResults.isEmpty()) {
            appendLog("No scanning results (start Wi-Fi + location).")
            return
        }
        val responders = scanResults
            .filter { it.is80211mcResponder }
            .filter { sr ->
                if (bssidInput.isBlank()) true
                else sr.BSSID.equals(bssidInput, ignoreCase = true)
            }

        if (responders.isEmpty()) {
            appendLog(
                "No anchors within range" +
                        if (bssidInput.isNotBlank()) " for $bssidInput" else ""
            )
            return
        }

        if (!originSet.get()) {
            val last = lastGpsOrNull()
            if (last != null) {
                appendLog(
                    "No ENU origin has been set yet. It will be established during the first RTT+GNSS synchronisation."
                )
            } else {
                appendLog("No GNSS is yet available to fix ENU origin.")
            }
        }

        val builder = RangingRequest.Builder()
        val selected: List<ScanResult> =
            if (bssidInput.isNotBlank()) listOf(responders.first()) else responders.take(MAX_RANGING_APS)

        selected.forEach { builder.addAccessPoint(it) }
        val request = builder.build()
        appendLog("Ranging against  ${selected.size} AP(s) FTM…")
        if (!isRttAvailable()) {
            appendLog("startRanging aborted: RTT not available (would be code=2).", alsoToUi = true)
            inFlight = false
            appendLog("RTT not available right now. Turn on Wi-Fi / location and try again.", alsoToUi = true)
            return

        }

        startRanging(request)
    }

    private fun startContinuousRanging(bssidInput: String) {
        if (!hasAllWifiPerms()) {
            appendLog("Wi-Fi/location permissions not granted.")
            requestNeededPermissions()
            return
        }
        if (!isRttSupported()) {
            appendLog("This device does not support Wi-Fi RTT (802.11mc).")
            return
        }

        // Pide un escaneo activo (el sistema lo throttleará, pero ayuda)
        try {
            wifiManager.startScan()
        } catch (e: Exception) {
            appendLog("startScan() failed: ${e.message}")
        }

        continuous.set(true)
        backoffMs = 0L
        lastRequest = buildRangingRequestOrNull(bssidInput)
        scheduleNextRtt(0L, bssidInput)
        appendLog(
            "Continuous measurement initiated" +
                    if (bssidInput.isNotBlank()) " on $bssidInput" else " (all anchors visible)"
        )
    }

    private fun stopContinuousRanging() {
        continuous.set(false)
        rangingFirstBurstDone = false

        // Flush pending UI log entries BEFORE clearing the handler,
        // otherwise the log flush runnable is killed with logFlushScheduled=true
        // and the UI console freezes.
        synchronized(uiLogBuffer) {
            while (uiLogBuffer.isNotEmpty()) {
                logLines.add(uiLogBuffer.removeFirst())
            }
        }
        val overflow = logLines.size - MAX_LOG_LINES
        repeat(overflow.coerceAtLeast(0)) { logLines.removeAt(0) }
        logFlushScheduled = false

        rttHandler.removeCallbacksAndMessages(null)
        inFlight = false
        if (::fileLogger.isInitialized) {
            fileLogger.flushAll()
        }
        appendLog("Continuous measurement stopped.")
    }

    private fun scheduleNextRtt(delayMs: Long, bssidInput: String) {
        if (!continuous.get()) return

        rttHandler.postDelayed({
            if (!continuous.get() || inFlight) return@postDelayed

            val now = System.currentTimeMillis()
            if (lastRequest == null || needRescan(now)) {
                // Trigger a real Wi-Fi scan so scanResults get refreshed;
                // the OS will throttle this (~4 scans per 2 min) but it keeps
                // the anchor list current on a moving drone.
                try {
                    @Suppress("DEPRECATION")
                    wifiManager.startScan()
                } catch (e: Exception) {
                    Log.w("MainActivity", "startScan() on rescan: ${e.message}")
                }
                lastRequest = buildRangingRequestOrNull(bssidInput)
                if (lastRequest == null) {
                    // No hay anchors; reintenta más tarde
                    scheduleNextRtt(1_000L, bssidInput)
                    return@postDelayed
                }
            }
            inFlight = true

            if (!isRttAvailable()) {
                if (System.currentTimeMillis() - lastRttNotAvailLogMs > 2_000L) {
                    appendLog("RTT not available (Wi-Fi off / RTT disabled). Waiting…", alsoToUi = true)
                    lastRttNotAvailLogMs = System.currentTimeMillis()
                }
                inFlight = false
                // Reintento suave (y además el receiver te despertará cuando vuelva)
                scheduleNextRtt(1_000L, bssidInput)
                return@postDelayed
            }


            startRanging(lastRequest!!)
        }, computeEffectiveDelay(delayMs))
    }

    /**
     * Combine the caller-requested delay with the configured per-agent
     * phase/period so two OSAs can interleave their FTM bursts.
     *
     * First burst after start or after updateRangingParams: apply [rangingPhaseMs].
     * Subsequent bursts: use max([delayMs], [rangingGapMs]).
     */
    private fun computeEffectiveDelay(delayMs: Long): Long {
        val gap = rangingGapMs
        if (!rangingFirstBurstDone) {
            rangingFirstBurstDone = true
            return (delayMs + rangingPhaseMs).coerceAtLeast(gap)
        }
        return delayMs.coerceAtLeast(gap)
    }



    private fun needRescan(now: Long): Boolean =
        (now - lastBuildMs) > RESCAN_EVERY_MS

    private fun buildRangingRequestOrNull(bssidInput: String): RangingRequest? {
        if (!hasAllWifiPerms()) return null
        val scanResults = try {
            wifiManager.scanResults.orEmpty()
        } catch (_: SecurityException) {
            emptyList()
        }

        val responders = scanResults
            .filter { it.is80211mcResponder }
            .filter { sr ->
                if (bssidInput.isBlank()) true
                else sr.BSSID.equals(bssidInput, ignoreCase = true)
            }

        if (responders.isEmpty()) {
            appendLog(
                "There are no FTM anchors within range." +
                        if (bssidInput.isNotBlank()) " for $bssidInput" else ""
            )
            return null
        }

        val selected =
            if (bssidInput.isNotBlank()) listOf(responders.first()) else responders.take(MAX_RANGING_APS)

        val builder = RangingRequest.Builder()
        selected.forEach { builder.addAccessPoint(it) }
        lastResponders = selected
        lastBuildMs = System.currentTimeMillis()
        return builder.build()
    }

    @SuppressLint("MissingPermission")
    private fun startRanging(request: RangingRequest) {
        val mgr = rttManager ?: run {
            appendLog("WifiRttManager is not available.")
            inFlight = false
            return
        }

        val fineOk = ActivityCompat.checkSelfPermission(
            this,
            Manifest.permission.ACCESS_FINE_LOCATION
        ) == PackageManager.PERMISSION_GRANTED
        val coarseOk = ActivityCompat.checkSelfPermission(
            this,
            Manifest.permission.ACCESS_COARSE_LOCATION
        ) == PackageManager.PERMISSION_GRANTED
        val nearbyOk = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            ActivityCompat.checkSelfPermission(
                this,
                Manifest.permission.NEARBY_WIFI_DEVICES
            ) == PackageManager.PERMISSION_GRANTED
        } else true

        if (!fineOk || !coarseOk || !nearbyOk) {
            appendLog("Permissions missing: location/nearby. Requesting...")
            requestNeededPermissions()
            inFlight = false
            return
        }

        val callback = object : android.net.wifi.rtt.RangingResultCallback() {
            override fun onRangingFailure(code: Int) {
                appendLog("RTT failed: code=$code")
                inFlight = false
                backoffMs =
                    if (backoffMs == 0L) 200L
                    else (backoffMs * 2L).coerceAtMost(5_000L)
                scheduleNextRtt(backoffMs, targetBssid())
            }

            override fun onRangingResults(results: MutableList<android.net.wifi.rtt.RangingResult>) {
                val wanted = targetBssid()
                val batchWallMs = System.currentTimeMillis()
                val batchMonoMs = SystemClock.elapsedRealtime()
                var nOk = 0
                var nFail = 0
                val distances = mutableListOf<Double>()

                for (res in results) {
                    val bssid = res.macAddress?.toString()
                    if (wanted.isNotBlank() && !bssid.equals(wanted, ignoreCase = true)) continue

                    if (res.status != android.net.wifi.rtt.RangingResult.STATUS_SUCCESS) {
                        nFail++
                        continue
                    }
                    nOk++

                    val distM = res.distanceMm / 1000.0
                    val stdM = res.distanceStdDevMm / 1000.0
                    val rssi = res.rssi
                    distances.add(distM)

                    executeCompute("processRttMeasurement") {
                        processRttMeasurement(
                            bssid = bssid,
                            distM = distM,
                            stdM = stdM,
                            rssi = rssi,
                            tWallMs = batchWallMs,
                            tMonoMs = batchMonoMs
                        )
                    }
                }

                logRttBatchSummary(nOk, nFail, distances)
                inFlight = false
                backoffMs = 0L
                scheduleNextRtt(MIN_PERIOD_MS, targetBssid())
            }

        }

        try {
            mgr.startRanging(request, mainExecutor, callback)
        } catch (t: Throwable) {
            Log.e("MainActivity", "startRanging failed", t)
            appendLog("startRanging failed: ${t.javaClass.simpleName}: ${t.message}")
            inFlight = false
            backoffMs = if (backoffMs == 0L) 500L else (backoffMs * 2L).coerceAtMost(5_000L)
            if (continuous.get()) {
                scheduleNextRtt(backoffMs, targetBssid())
            }
        }
    }


    private val rttStateReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            if (intent.action != WifiRttManager.ACTION_WIFI_RTT_STATE_CHANGED) return

            val avail = isRttAvailable()
            appendLog("RTT state changed → available=$avail", alsoToUi = true)

            // Si estabas en continuo, reanuda el ciclo cuando vuelva RTT
            if (avail && continuous.get() && !inFlight) {
                scheduleNextRtt(0L, targetBssid())
            }
        }
    }

    /**
     * React to fresh Wi-Fi scan results immediately instead of polling stale
     * scanResults on a timer.  This eliminates the long blind window when an AP
     * disappears and reappears (Android throttles startScan() to ~4/2min, so
     * the 1-second poll loop was reading stale data for 30–120+ seconds).
     */
    private val scanResultsReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            if (intent.action != WifiManager.SCAN_RESULTS_AVAILABLE_ACTION) return
            if (!continuous.get()) return

            // Rebuild the ranging request with the fresh scan results
            val bssid = targetBssid()
            val fresh = buildRangingRequestOrNull(bssid)
            if (fresh != null) {
                lastRequest = fresh
                // If we were stuck without anchors AND not already in-flight,
                // kick the ranging loop immediately.
                if (!inFlight) {
                    scheduleNextRtt(0L, bssid)
                }
            }
        }
    }
    private var scanResultsReceiverRegistered = false


    private fun startExperiment(reason: String, targetBssid: String) {
        // Si ya había uno activo, ciérralo para no mezclar vuelos.
        if (expActive) stopExperiment("auto_restart")

        // Reset completo: estimadores, EKF, origen ENU —
        // cada experimento debe ser independiente.
        resetEstimator()
        accuracyMetrics.reset()

        val wall = System.currentTimeMillis()
        val mono = SystemClock.elapsedRealtime()
        val id = "exp_${wall}_" + UUID.randomUUID().toString().substring(0, 8)

        expId = id
        expActive = true
        expStartWallMs = wall
        expStartMonoMs = mono

        // Apply experiment config AGL and range bias
        if (experimentConfig.aglMeters > 0) {
            updateAglMeters(experimentConfig.aglMeters)
        }
        updateRangeBias(experimentConfig.rangeBiasM)

        appendLog("EXPERIMENT START id=$id reason=$reason target=$targetBssid label=${experimentConfig.label} agl=${experimentConfig.aglMeters}m bias=${experimentConfig.rangeBiasM}m gt_aps=${groundTruth.size}")

        val json = """{"type":"experiment","event":"start","id":"$id","reason":${jsonEscape(reason)},"target_bssid":${jsonEscape(targetBssid)},"label":${jsonEscape(experimentConfig.label)},"filter_mode":"${experimentConfig.filterMode.name}","agl_m":${experimentConfig.aglMeters},"range_bias_m":${experimentConfig.rangeBiasM},"notes":${jsonEscape(experimentConfig.notes)},"gt_aps":${groundTruth.size},"agent_id":"${currentAgentId()}","peer_agent_id":"${experimentConfig.peerAgentId}","cooperative_mode":"${experimentConfig.cooperativeMode}","ranging_period_ms":${experimentConfig.rangingPeriodMs},"ranging_phase_ms":${experimentConfig.rangingPhaseMs},"t_ms":$wall,"t_mono_ms":$mono}"""

        fileLogger.appendTaggedJson("experiment.jsonl", "exp", json, flushNow = true)

        // Clock-sync sanity check: difference between wall time and the most
        // recent GPS fix (UTC).  On dual-OSA experiments both phones must
        // share a common time base (NTP or PPS from the F9P) for the
        // cross-agent fusion to work.
        val lastGps = lastGpsOrNull()
        val gpsAgeMs = if (lastGps != null) (wall - lastGps.tMs) else -1L
        val clockCheckJson = """{"type":"clock_check","exp_id":"$id","agent_id":"${currentAgentId()}","t_ms":$wall,"gps_age_ms":$gpsAgeMs,"gps_source":"${lastGps?.source ?: "none"}","ntrip_connected":${externalGnss?.isNtripConnected == true}}"""
        fileLogger.appendTaggedJson("experiment.jsonl", "clock_check", clockCheckJson, flushNow = true)
        if (gpsAgeMs < 0) {
            appendLog("WARNING clock_check: no GPS fix available at experiment start")
        } else if (gpsAgeMs > 5_000L) {
            appendLog("WARNING clock_check: last GPS fix is ${gpsAgeMs} ms old")
        }

        // Log ground truth positions for reproducibility
        for (ap in groundTruth.all()) {
            val gtJson = """{"type":"ground_truth","exp_id":"$id","bssid":${jsonEscape(ap.bssid)},"lat":${ap.lat},"lon":${ap.lon},"alt":${ap.alt},"t_ms":$wall}"""
            fileLogger.appendTaggedJson("experiment.jsonl", "gt", gtJson)
        }

        // Separadores en los logs principales (opcional pero muy útil)
        fileLogger.appendTaggedJson("gps.jsonl", "exp", json)
        fileLogger.appendTaggedJson("rtt.jsonl", "exp", json)
        fileLogger.appendTaggedJson("mlat.jsonl", "exp", json)
    }

    private fun stopExperiment(reason: String) {
        if (!expActive) return

        val wall = System.currentTimeMillis()
        val mono = SystemClock.elapsedRealtime()
        val id = expId ?: "unknown"
        val dur = mono - expStartMonoMs

        expActive = false

        appendLog("EXPERIMENT STOP id=$id reason=$reason duration=${dur}ms")

        val json = """{"type":"experiment","event":"stop","id":"$id","reason":${jsonEscape(reason)},"label":${jsonEscape(experimentConfig.label)},"filter_mode":"${activeFilterMode.name}","agl_m":${experimentConfig.aglMeters},"agent_id":"${currentAgentId()}","peer_agent_id":"${experimentConfig.peerAgentId}","cooperative_mode":"${experimentConfig.cooperativeMode}","t_ms":$wall,"t_mono_ms":$mono,"duration_ms":$dur}"""

        fileLogger.appendTaggedJson("experiment.jsonl", "exp", json, flushNow = true)
        fileLogger.appendTaggedJson("gps.jsonl", "exp", json)
        fileLogger.appendTaggedJson("rtt.jsonl", "exp", json)
        fileLogger.appendTaggedJson("mlat.jsonl", "exp", json, flushNow = true)

        // Compute joint geometric spread + azimuth coverage across all anchors.
        // We take the max over tracks because the worst-observed anchor is
        // typically the bottleneck for bootstrap.
        var geom2d = Double.NaN
        var geom3d = Double.NaN
        var azCov = Double.NaN
        for ((_, tr) in tracks) {
            val s2 = tr.est.geometricSpread2D()
            val s3 = tr.est.geometricSpread3D()
            val az = tr.est.azimuthCoverageDeg(0.0, 0.0)
            if (s2.isFinite() && (geom2d.isNaN() || s2 > geom2d)) geom2d = s2
            if (s3.isFinite() && (geom3d.isNaN() || s3 > geom3d)) geom3d = s3
            if (az.isFinite() && (azCov.isNaN() || az > azCov)) azCov = az
        }

        // Per-agent stats — for now we only log own-agent counters locally;
        // the offboard fusion node aggregates across agents.
        val ownAgentStats = AccuracyMetrics.AgentStats(
            agentId = currentAgentId(),
            nRtt = rttCount.get(),
            meanSigmaUsed = Double.NaN,
            gnssAccMean = Double.NaN,
            zSpreadM = geom3d
        )
        val perAgent = mapOf(ownAgentStats.agentId to ownAgentStats)

        // Log accuracy summary if ground truth was available
        val summaryJson = accuracyMetrics.summaryToJson(
            id, experimentConfig.label, experimentConfig.aglMeters,
            agentId = currentAgentId(),
            peerAgentId = experimentConfig.peerAgentId,
            cooperativeMode = experimentConfig.cooperativeMode,
            geomSpread2DM = geom2d,
            geomSpread3DM = geom3d,
            azimuthCoverageDeg = azCov,
            perAgent = perAgent
        )
        if (summaryJson != null) {
            fileLogger.appendTaggedJson("experiment.jsonl", "accuracy", summaryJson, flushNow = true)
            val s = accuracyMetrics.computeSummary()
            if (s != null) {
                appendLog("ACCURACY SUMMARY: ${s.nSamples} samples, ${s.nAps} APs, " +
                    "CEP50=${"%.2f".format(s.cep50)}m, RMSE2D=${"%.2f".format(s.rmse2D)}m, " +
                    "RMSE3D=${"%.2f".format(s.rmse3D)}m, spread3D=${"%.1f".format(geom3d)}m, azCov=${"%.0f".format(azCov)}°")
            }
        } else {
            // Even without ground truth, log geometry stats for analysis.
            val geomJson = """{"type":"geometry_summary","exp_id":"$id","agent_id":"${currentAgentId()}","geom_spread_2d_m":${if (geom2d.isFinite()) geom2d else "null"},"geom_spread_3d_m":${if (geom3d.isFinite()) geom3d else "null"},"azimuth_coverage_deg":${if (azCov.isFinite()) azCov else "null"},"t_ms":$wall}"""
            fileLogger.appendTaggedJson("experiment.jsonl", "geom", geomJson, flushNow = true)
            appendLog("GEOMETRY SUMMARY: spread2D=${"%.1f".format(geom2d)}m, spread3D=${"%.1f".format(geom3d)}m, azCov=${"%.0f".format(azCov)}°")
        }
    }


    private fun logRttBatchSummary(nOk: Int, nFail: Int, distances: List<Double> = emptyList()) {
        if (nOk <= 0 && nFail <= 0) return
        val now = System.currentTimeMillis()

        if (nOk > 0) {
            lastRttTrafficMs = now
        }

        val distStr = if (distances.isNotEmpty()) {
            val avg = distances.average()
            ", dist=${"%,.2f".format(avg)}m"
        } else ""

        if (now - lastRttUiLogMs >= 1_000L) {
            appendLog("RTT summary: ok=$nOk, fail=$nFail$distStr", alsoToUi = true)
            lastRttUiLogMs = now
        } else {
            appendLog("RTT ok=$nOk fail=$nFail$distStr", alsoToUi = false)
        }
    }


    private fun cleanNum(x: Double): Double =
        if (x.isFinite()) x else -1.0

    private fun bssidHash(bssid: String): Double {
        val h = bssid.hashCode().toLong() - Int.MIN_VALUE.toLong()
        return ln(h.toDouble() + 1.0)
    }

    /* -------------------------------------------------------------------------
     *  Procesado de RTT + GNSS + estimador (en computeExec)
     * ---------------------------------------------------------------------- */

    private fun processRttMeasurement(
        bssid: String?,
        distM: Double,
        stdM: Double,
        rssi: Int,
        tWallMs: Long,
        tMonoMs: Long
    ) {
        val seq = rttSeq.incrementAndGet()
        val n = rttCount.incrementAndGet()

        // 1) Sync GNSS ↔ RTT
        val sync = syncGpsToRtt(tMonoMs, tWallMs)
        val gps = sync?.pose
        val synced = sync != null && (gps!!.accM <= 0.0 || gps.accM <= MAX_GNSS_ACC_M)

        // Publish raw RTT always (even unsynchronised) so the topic
        // matches the on-screen ok=1 rate. When GPS is missing the
        // gps_* fields will be -1 and "synced" will be false.
        val bssidStr = canonicalBssid(bssid ?: return)

        // 2b) Build and publish raw RTT JSON (always, even without GPS)
        val rawSigma = if (stdM > 0.0) stdM else 1.0
        val id = expId ?: ""
        val sb = acquireSb()
        val rttJson = sb.append('{')
            .append("\"seq\":").append(seq)
            .append(",\"bssid\":\"").append(bssidStr).append('"')
            .append(",\"distance_m\":").append(distM)
            .append(",\"std_m\":").append(rawSigma)
            .append(",\"rssi_dbm\":").append(rssi)
            .append(",\"synced\":").append(synced)
            .append(",\"t_ms\":").append(tWallMs)
            .append(",\"t_mono_ms\":").append(tMonoMs)
            .append(",\"gps_t_ms\":").append(if (synced) gps!!.tMs else -1L)
            .append(",\"gps_mono_ms\":").append(if (synced) gps!!.monoMs else -1L)
            .append(",\"gps_acc_m\":").append(if (synced) gps!!.accM else -1.0)
            .append(",\"gps_source\":\"").append(if (synced) gps!!.source else "none").append('"')
            .append(",\"sync_method\":\"").append(sync?.method ?: "none").append('"')
            .append(",\"sync_dt_prev_ms\":").append(sync?.dtPrevMs ?: -1L)
            .append(",\"sync_dt_next_ms\":").append(sync?.dtNextMs ?: -1L)
            .append(",\"sync_bracket_gap_ms\":").append(sync?.bracketGapMs ?: -1L)
            .append(",\"sync_window_ms\":").append(syncWindowMs)
            .append(",\"filter_mode\":\"").append(activeFilterMode.name).append('"')
            .append(",\"exp_id\":\"").append(id).append('"')
            .append(",\"agent_id\":\"").append(currentAgentId()).append('"')
            .append('}')
            .toString()

        fileLogger.appendTaggedJson("rtt.jsonl", "rtt", rttJson)
        ros2?.publishRttJson(rttJson)

        // If GPS sync failed or accuracy too bad, skip estimation
        if (!synced) return

        // 2c) Configured range bias: subtract the known systematic offset before
        //     gating/filter.  The IEKF still estimates residual bias online.
        val correctedDist = (distM - rangeBiasM).coerceAtLeast(0.0)

        // 3) Track per anchor (needs valid GPS)
        val track = getTrackForBssid(bssidStr, gps!!)
        val est = track.est

        // 3b) GATING robusto sobre RTT (mediana + MAD) + sigma robusta
        val (accepted, gatedObs) = track.rttGate.pushAndGate(
            RttObs(
                tMonoMs = tMonoMs,
                d = correctedDist,
                sigma = rawSigma,
                rssi = rssi
            )
        )

        if (!accepted || gatedObs == null) return

        val mode = activeFilterMode

        // 5) ENU del dron (teléfono)
        val (uE, uN, uU) = est.latLonToENU(gps.lat, gps.lon, gps.alt)

        // "Suelo local" aproximado bajo el dron: z_ground ≈ uU - AGL (solo como prior).
        // En altitude_mode=auto NO se alimenta — el solver 3D estima Z por sí solo
        // a partir de la diversidad vertical de las trayectorias.
        if (experimentConfig.altitudeMode == ExperimentConfig.ALT_MODE_MANUAL_AGL) {
            track.zGroundMed.push(uU - aglMeters)
        }

        pruneOldGps(tMonoMs)

        // 5b) Compute effective sigma based on filter mode
        var effectiveSigma = gatedObs.sigma.coerceAtLeast(0.12)
        when (mode) {
            FilterMode.NLOS -> {
                val (_, adjusted) = track.nlosDetector.evaluate(gatedObs.d, effectiveSigma, rssi)
                effectiveSigma = adjusted
            }
            FilterMode.EMPIRICAL_R -> {
                val empR = track.empiricalRModel.computeR(gatedObs.d, rssi, effectiveSigma)
                effectiveSigma = kotlin.math.sqrt(empR)
            }
            else -> {} // BASELINE, UKF, EMA, ADAPTIVE_R use gate sigma directly
        }

        // 5c) Inflate R with GPS position uncertainty: R_eff = σ_RTT² + σ_GPS²
        //     Drone position error propagates 1:1 into range residual.
        if (gps.accM > 0.0) {
            effectiveSigma = kotlin.math.sqrt(effectiveSigma * effectiveSigma + gps.accM * gps.accM)
        }

        // 6) Bootstrap 3D (batch) con Gauss-Newton
        val needsBootstrap = when (mode) {
            FilterMode.UKF -> track.ukf3d == null
            else -> track.ekf3d == null
        }
        if (needsBootstrap) {
            est.addSampleFromGpsAndRtt(
                gps,
                RttSample(tWallMs, gatedObs.d, gatedObs.sigma, gatedObs.rssi)
            )

            if (est.sampleCount() < MIN_SAMPLES_COUNT) return

            // In altitude_mode=auto, require enough vertical diversity before
            // committing to a 3D bootstrap; otherwise the anchor Z is
            // observationally unidentified and GN will converge to a
            // geometry-dependent arbitrary value.
            val minVz = if (experimentConfig.altitudeMode == ExperimentConfig.ALT_MODE_AUTO) 3.0 else 0.0
            val init3d = est.refineGaussNewton3D(minVerticalSpread = minVz) ?: return

            val zPrior = track.zGroundMed.medianOrNull()
            val zInit = if (zPrior != null && kotlin.math.abs(init3d.z - zPrior) > 20.0) zPrior else init3d.z

            when (mode) {
                FilterMode.UKF -> {
                    val p0 = arrayOf(
                        doubleArrayOf(25.0, 0.0, 0.0),
                        doubleArrayOf(0.0, 25.0, 0.0),
                        doubleArrayOf(0.0, 0.0, 100.0)
                    )
                    val x0 = doubleArrayOf(init3d.x, init3d.y, zInit)
                    track.ukf3d = AnchorUKF3D(x0 = x0, p0 = p0)
                }
                else -> {
                    // 4-state IEKF: [x, y, z, bias]
                    val p0 = arrayOf(
                        doubleArrayOf(25.0, 0.0, 0.0, 0.0),
                        doubleArrayOf(0.0, 25.0, 0.0, 0.0),
                        doubleArrayOf(0.0, 0.0, 100.0, 0.0),
                        doubleArrayOf(0.0, 0.0, 0.0, 4.0)   // bias prior ±2 m
                    )
                    val x0 = doubleArrayOf(init3d.x, init3d.y, zInit, 0.0)
                    track.ekf3d = AnchorIEKF3D(x0 = x0, p0 = p0)
                }
            }
            return
        }

        // 7) Filter update (IEKF3D or UKF3D depending on mode)
        val xE: Double
        val yN2: Double
        val zU2: Double
        val sigmaH: Double
        val sigmaZ: Double
        val nAcc: Long
        val covP: Array<DoubleArray>

        when (mode) {
            FilterMode.UKF -> {
                val ukf = track.ukf3d!!
                ukf.predict()
                val updOk = ukf.update(
                    z = gatedObs.d, sigma = effectiveSigma,
                    uE = uE, uN = uN, uU = uU, gateGamma = 9.0
                )
                if (!updOk) return

                track.zGroundMed.medianOrNull()?.let { z0 ->
                    ukf.updateZPrior(zPrior = z0, sigmaZ = Z_PRIOR_SIGMA)
                }
                xE = ukf.x[0]; yN2 = ukf.x[1]; zU2 = ukf.x[2]
                sigmaH = kotlin.math.sqrt(ukf.P[0][0] + ukf.P[1][1])
                sigmaZ = kotlin.math.sqrt(ukf.P[2][2])
                nAcc = ukf.nAccepted
                covP = ukf.P
            }
            FilterMode.ADAPTIVE_R -> {
                val ekf = track.ekf3d!!
                ekf.predict()
                // Compute innovation & HPH^T for adaptive R
                val dx0 = ekf.x[0] - uE; val dy0 = ekf.x[1] - uN; val dz0 = ekf.x[2] - uU
                val hPred = kotlin.math.sqrt(dx0*dx0 + dy0*dy0 + dz0*dz0).coerceAtLeast(1e-6)
                val innovation = gatedObs.d - (hPred + ekf.bias)
                // H = [dx/d, dy/d, dz/d, 1]  (4-state Jacobian)
                val hArr = doubleArrayOf(dx0/hPred, dy0/hPred, dz0/hPred, 1.0)
                var hpht = 0.0
                for (i in 0 until 4) for (j in 0 until 4) hpht += hArr[i] * ekf.P[i][j] * hArr[j]
                val adaptR = track.adaptiveR.update(innovation, hpht)
                val adaptSigma = kotlin.math.sqrt(adaptR).coerceAtLeast(0.12)

                val updOk = ekf.updateIEKF(
                    z = gatedObs.d, sigma = adaptSigma,
                    uE = uE, uN = uN, uU = uU, iters = 3, gateGamma = 9.0
                )
                if (!updOk) return

                track.zGroundMed.medianOrNull()?.let { z0 ->
                    ekf.updateZPrior(zPrior = z0, sigmaZ = Z_PRIOR_SIGMA)
                }
                xE = ekf.x[0]; yN2 = ekf.x[1]; zU2 = ekf.x[2]
                sigmaH = kotlin.math.sqrt(ekf.P[0][0] + ekf.P[1][1])
                sigmaZ = kotlin.math.sqrt(ekf.P[2][2])
                nAcc = ekf.nAccepted
                covP = ekf.P
            }
            else -> {
                // BASELINE, NLOS, EMPIRICAL_R, EMA — all use IEKF3D
                val ekf = track.ekf3d!!
                ekf.predict()
                val updOk = ekf.updateIEKF(
                    z = gatedObs.d, sigma = effectiveSigma,
                    uE = uE, uN = uN, uU = uU, iters = 3, gateGamma = 9.0
                )
                if (!updOk) return

                track.zGroundMed.medianOrNull()?.let { z0 ->
                    ekf.updateZPrior(zPrior = z0, sigmaZ = Z_PRIOR_SIGMA)
                }
                xE = ekf.x[0]; yN2 = ekf.x[1]; zU2 = ekf.x[2]
                sigmaH = kotlin.math.sqrt(ekf.P[0][0] + ekf.P[1][1])
                sigmaZ = kotlin.math.sqrt(ekf.P[2][2])
                nAcc = ekf.nAccepted
                covP = ekf.P
            }
        }

        // 8) Log de estado (para que "veas" multilateración aunque aún no sea estable)
        val nowMs = System.currentTimeMillis()
        if (nowMs - track.lastStateLogMs >= 1_000L) {
            val (latS, lonS, altS) = est.enuToLatLon(xE, yN2, zU2)
            val sbS = acquireSb()
            val stateJson = sbS.append('{')
                .append("\"bssid\":\"").append(bssidStr).append('"')
                .append(",\"lat\":").append(latS)
                .append(",\"lon\":").append(lonS)
                .append(",\"alt\":").append(altS)
                .append(",\"enu_x\":").append(xE)
                .append(",\"enu_y\":").append(yN2)
                .append(",\"enu_z\":").append(zU2)
                .append(",\"sigma_h\":").append(sigmaH)
                .append(",\"sigma_z\":").append(sigmaZ)
                .append(",\"bias_m\":").append(
                    when (mode) {
                        FilterMode.UKF -> 0.0
                        else -> track.ekf3d?.bias ?: 0.0
                    })
                .append(",\"n_accepted\":").append(nAcc)
                .append(",\"t_ms\":").append(nowMs)
                .append(",\"last_rtt_t_ms\":").append(tWallMs)
                .append(",\"exp_id\":\"").append(id).append('"')
                .append(",\"agent_id\":\"").append(currentAgentId()).append('"')
                .append(",\"filter_mode\":\"").append(mode.name).append('"')
                .append(",\"mode\":\"state\"")
                .append('}')
                .toString()

            fileLogger.appendTaggedJson("mlat_state.jsonl", "mlat_state", stateJson)

            // Publish intermediate state to ROS 2 so the GUI can show
            // in-progress estimates before the stability check passes.
            val isStableForState = sigmaH.isFinite() && sigmaH < 2.5 && nAcc >= 10L
            ros2?.publishAnchorStateFor(
                bssidStr,
                latS, lonS, altS,
                sigmaH, sigmaZ,
                nAcc,
                isStableForState,
                bssidHash(bssidStr)
            )

            track.lastStateLogMs = nowMs
        }

        // 9) Adaptive stability check — publish when the filter has converged
        val isConverging = sigmaH < track.prevSigmaH * 1.02
        track.convergingStreak = if (isConverging) track.convergingStreak + 1 else 0
        track.prevSigmaH = sigmaH
        track.prevSigmaZ = sigmaZ

        val minAccepted = if (track.convergingStreak >= 5) 10L else 15L
        val maxSigmaH = 2.5
        val maxSigmaZ2 = 8.0
        val maxJumpXY = kotlin.math.max(1.0, sigmaH * 2.0)
        val maxJumpZ = kotlin.math.max(2.0, sigmaZ * 2.0)
        val needStreak = if (track.convergingStreak >= 8) 2 else 3

        val lastPub = track.lastPubXYZ
        val jumpXY = if (lastPub != null) {
            kotlin.math.sqrt(
                (xE - lastPub.first) * (xE - lastPub.first) +
                        (yN2 - lastPub.second) * (yN2 - lastPub.second)
            )
        } else 0.0

        val jumpZ = if (lastPub != null) kotlin.math.abs(zU2 - lastPub.third) else 0.0

        val stableNow =
            (nAcc >= minAccepted) &&
                    (sigmaH.isFinite() && sigmaH < maxSigmaH) &&
                    (sigmaZ.isFinite() && sigmaZ < maxSigmaZ2) &&
                    (jumpXY.isFinite() && jumpXY < maxJumpXY) &&
                    (jumpZ.isFinite() && jumpZ < maxJumpZ)

        track.stableStreak = if (stableNow) track.stableStreak + 1 else 0
        if (track.stableStreak < needStreak) return

        // 10) Suavizado de salida y publicación final
        val xPub: Double
        val yPub: Double
        when (mode) {
            FilterMode.EMA -> {
                val (ex, ey) = track.emaXY.push(xE, yN2)
                xPub = ex; yPub = ey
            }
            else -> {
                val (mx, my) = track.outMedXY.push(xE, yN2)
                xPub = mx; yPub = my
            }
        }
        track.zOutMed.push(zU2)
        val zPub = track.zOutMed.medianOrNull() ?: zU2

        track.lastPubXYZ = Triple(xPub, yPub, zPub)

        val (latE, lonE, altE) = est.enuToLatLon(xPub, yPub, zPub)

        val cov_xx = covP[0][0]
        val cov_xy = covP[0][1]
        val cov_yy = covP[1][1]
        val nSS = nAcc.toDouble()

        ros2?.publishAnchorEstimateFor(
            bssidStr,
            latE, lonE, altE,
            cov_xx, cov_yy, cov_xy,
            nSS,
            bssidHash(bssidStr)
        )

        // 11) Log mlat estable (dataset "bueno")
        val dtWallMs = tWallMs - gps.tMs

        val sb2 = acquireSb()
        val mlatJson = sb2.append('{')
            .append("\"bssid\":\"").append(bssidStr).append('"')
            .append(",\"lat\":").append(latE)
            .append(",\"lon\":").append(lonE)
            .append(",\"alt\":").append(altE)
            .append(",\"cov_xx\":").append(cov_xx)
            .append(",\"cov_yy\":").append(cov_yy)
            .append(",\"cov_xy\":").append(cov_xy)
            .append(",\"n_samples\":").append(nSS)
            .append(",\"bias_m\":").append(
                when (mode) {
                    FilterMode.UKF -> 0.0
                    else -> track.ekf3d?.bias ?: 0.0
                })
            .append(",\"bssid_hash\":").append(bssidHash(bssidStr))
            .append(",\"t_ms\":").append(nowMs)
            .append(",\"last_rtt_t_ms\":").append(tWallMs)
            .append(",\"last_rtt_t_mono_ms\":").append(tMonoMs)
            .append(",\"last_gps_t_ms\":").append(gps.tMs)
            .append(",\"last_gps_t_mono_ms\":").append(gps.monoMs)
            .append(",\"last_dt_ms\":").append(dtWallMs)
            .append(",\"exp_id\":\"").append(id).append('"')
            .append(",\"agent_id\":\"").append(currentAgentId()).append('"')
            .append(",\"filter_mode\":\"").append(mode.name).append('"')
            .append(",\"mode\":\"stable\"")
            .append('}')
            .toString()

        fileLogger.appendTaggedJson("mlat.jsonl", "mlat", mlatJson, flushNow = true)
        lastPublishMs = nowMs

        // 12) Ground-truth accuracy computation (if available)
        val gt = groundTruth.get(bssidStr)
        if (gt != null) {
            accuracyMetrics.addSample(
                bssid = bssidStr,
                estLat = latE, estLon = lonE, estAlt = altE,
                gtLat = gt.lat, gtLon = gt.lon, gtAlt = gt.alt,
                tMs = nowMs
            )
        }

        if (n % 100L == 0L) {
            appendLog("RTT processed so far: $n", alsoToUi = true)
        }
    }



    private fun canonicalBssid(bssid: String): String =
        bssid.trim().lowercase()

    private fun getTrackForBssid(bssid: String, originGps: GpsPose): AnchorTrack {
        if (enuOrigin == null && originSet.compareAndSet(false, true)) {
            enuOrigin = originGps
            appendLog("ENU origin fixed at (lat0, lon0, alt0)=(${originGps.lat}, ${originGps.lon}, ${originGps.alt}), AGL≈$aglMeters m")
        }
        val o = enuOrigin ?: originGps
        val key = canonicalBssid(bssid)

        return tracks.computeIfAbsent(key) {
            // In AUTO altitude mode the AGL seed is disabled; the 3D solver
            // recovers anchor Z from the natural vertical diversity of the
            // drone trajectories. Pass 0.0 so adjustedRadiusSquared() and
            // initialZGuess() degrade gracefully (see AnchorEstimator).
            val aglForEstimator =
                if (experimentConfig.altitudeMode == ExperimentConfig.ALT_MODE_MANUAL_AGL) aglMeters
                else 0.0
            val ae = AnchorEstimator(o.lat, o.lon, o.alt, aglForEstimator).also { it.setWeighting(currentWeighting) }
            AnchorTrack(est = ae)
        }
    }


    private fun onWeightingChanged(w: Weighting) {
        currentWeighting = w
        executeCompute("setWeighting") {
            for (t in tracks.values) {
                t.est.setWeighting(w)
            }
        }
        appendLog("Weighting method changed to: $w", alsoToUi = false)
    }

    private fun resetEstimator() {
        originSet.set(false)
        enuOrigin = null

        executeCompute("resetEstimator") {
            tracks.clear()
        }

        appendLog("Estimators reset (all anchors). ENU origin will be fixed on the next RTT+GNSS sync.")
    }

    /** Apply a new filter mode: resets all tracks so the new pipeline starts fresh. */
    private fun applyFilterMode(mode: FilterMode) {
        if (mode == activeFilterMode) return
        activeFilterMode = mode
        resetEstimator()
        appendLog("Filter mode changed to ${mode.name} -- estimators reset")
    }


    /* -------------------------------------------------------------------------
     *  GNSS
     * ---------------------------------------------------------------------- */

    private fun startLocationUpdates() {
        if (gnssActive) return

        val fineOk = ActivityCompat.checkSelfPermission(
            this,
            Manifest.permission.ACCESS_FINE_LOCATION
        ) == PackageManager.PERMISSION_GRANTED
        val coarseOk = ActivityCompat.checkSelfPermission(
            this,
            Manifest.permission.ACCESS_COARSE_LOCATION
        ) == PackageManager.PERMISSION_GRANTED

        if (!fineOk || !coarseOk) {
            appendLog("Missing location permissions.")
            return
        }

        val cb = object : LocationCallback() {
            override fun onLocationResult(result: LocationResult) {
                result.lastLocation?.let { handleLocation(it) }
            }
        }
        locCb = cb
        try {
            fused.requestLocationUpdates(locReq, cb, mainLooper)
            gnssActive = true
        } catch (t: Throwable) {
            appendLog("Cannot start internal GNSS updates: ${t.javaClass.simpleName}: ${t.message}")
            Log.e("MainActivity", "Cannot start internal GNSS updates", t)
            locCb = null
            gnssActive = false
        }
    }

    private fun stopLocationUpdates() {
        locCb?.let { cb ->
            runCatching { fused.removeLocationUpdates(cb) }
                .onFailure { Log.w("MainActivity", "Cannot stop location updates", it) }
        }
        locCb = null
        gnssActive = false
    }

    private fun maybeStartInternalGnssFallback(reason: String) {
        if (!allowInternalGnssFallback) {
            if (gnssActive) stopLocationUpdates()
            appendLog("$reason → internal GNSS fallback disabled; waiting for external GNSS")
            return
        }
        if (!gnssActive) {
            appendLog("$reason → starting internal GNSS fallback")
            startLocationUpdates()
        }
    }

    private fun handleLocation(loc: Location) {
        if (!allowInternalGnssFallback) {
            return
        }

        val wallMs = System.currentTimeMillis()
        val monoMs = loc.elapsedRealtimeNanos / 1_000_000L

        // Si hay GNSS externo con fix reciente (< 5 s), ignoramos el interno
        if (externalGnssActive && (monoMs - lastExternalGnssFixMonoMs) < 5_000L) {
            return
        }

        val lat = loc.latitude
        val lon = loc.longitude
        val alt = if (loc.hasAltitude()) loc.altitude else 0.0
        val acc = if (loc.hasAccuracy()) loc.accuracy.toDouble() else -1.0

        pushGps(GpsPose(lat, lon, alt, wallMs, monoMs, acc, "internal"))

        ros2?.publishLocation(lat, lon, alt, acc, wallMs)

        val id = expId ?: ""
        val json = """{"lat":$lat,"lon":$lon,"alt":$alt,"acc_m":$acc,"t_ms":$wallMs,"t_mono_ms":$monoMs,"source":"internal","exp_id":"$id","agent_id":"${currentAgentId()}"}"""
        fileLogger.appendTaggedJson("gps.jsonl", "gps", json)
    }




    fun markExternalGnssStopped() {
        externalGnssActive = false
        lastExternalGnssFixMonoMs = 0L
    }



    /**
     * Llamar desde el lector NMEA del GNSS externo cuando haya un fix nuevo.
     *
     * @param lat  latitud WGS84 en grados
     * @param lon  longitud WGS84 en grados
     * @param alt  altitud en m (MSL o el que te dé el receptor)
     * @param accM precisión estimada en metros (usa HDOP o -1.0 si no tienes)
     */

    private var lastExtLocMonoMs: Long? = null


    fun onExternalGnssFix(
        lat: Double,
        lon: Double,
        alt: Double,
        accM: Double = -1.0
    ) {
        val wallMs = System.currentTimeMillis()
        val monoMs = SystemClock.elapsedRealtime()

        // debug Hz con monotónico
        lastExtLocMonoMs?.let { prev ->
            val dt = monoMs - prev
            if (dt > 0) {
                val hz = 1000.0 / dt.toDouble()
                Log.i("EXT_LOC", "onExternalGnssFix cada ${dt} ms (~${"%.1f".format(hz)} Hz)")
            }
        }
        lastExtLocMonoMs = monoMs

        externalGnssActive = true
        lastExternalGnssFixMonoMs = monoMs

        val pose = GpsPose(lat, lon, alt, wallMs, monoMs, accM, "external")
        pushGps(pose)

        ros2?.publishLocation(lat, lon, alt, accM, wallMs)

        val id = expId ?: ""
        val json = """{"lat":$lat,"lon":$lon,"alt":$alt,"acc_m":$accM,"t_ms":$wallMs,"t_mono_ms":$monoMs,"source":"external","exp_id":"$id","agent_id":"${currentAgentId()}"}"""

        fileLogger.appendTaggedJson("gps.jsonl", "gps", json)

        if (wallMs - lastExternalUiLogMs >= 2_000L) {
            appendLog(
                "External GNSS: lat=${"%.7f".format(lat)}, lon=${"%.7f".format(lon)}, alt=${"%.1f".format(alt)} m, acc≈${"%.1f".format(accM)} m"
            )
            lastExternalUiLogMs = wallMs
        }
    }


    /* -------------------------------------------------------------------------
     *  Cooperative OSA: peer drone GNSS + RTT ingest
     * ---------------------------------------------------------------------- */

    /** Running counters for peer-data logging throttling and UI warnings. */
    @Volatile private var lastPeerLocMs: Long = 0L
    @Volatile private var lastPeerRttMs: Long = 0L
    @Volatile private var lastPeerDistanceWarnMs: Long = 0L
    // Last peer drone position (for distance-between-drones check)
    @Volatile private var lastPeerLat: Double = Double.NaN
    @Volatile private var lastPeerLon: Double = Double.NaN
    @Volatile private var lastPeerAlt: Double = Double.NaN

    /**
     * Called when /<peerAgentId>/phone/location publishes the peer drone's
     * GNSS fix. Used for (a) adding peer pose samples to each anchor's
     * geometric-spread buffer and (b) anti-collision warnings.
     */
    private fun handlePeerLocation(lat: Double, lon: Double, alt: Double, accM: Double, tMs: Long) {
        val coop = experimentConfig.cooperativeMode
        if (coop == ExperimentConfig.COOP_INDEPENDENT) return
        lastPeerLat = lat; lastPeerLon = lon; lastPeerAlt = alt
        lastPeerLocMs = System.currentTimeMillis()

        // Log to JSONL for post-processing (offloaded to computeExec to avoid
        // blocking the ROS 2 spin thread).
        val id = expId ?: ""
        val peerId = ros2?.peerAgentId ?: experimentConfig.peerAgentId
        val ownAgent = experimentConfig.agentId
        val json = """{"type":"peer_location","peer_agent_id":"$peerId","agent_id":"$ownAgent","lat":$lat,"lon":$lon,"alt":$alt,"acc_m":$accM,"t_ms":$tMs,"exp_id":"$id"}"""
        fileLogger.appendTaggedJson("gps.jsonl", "peer_gps", json)

        // Feed peer pose into every anchor estimator for joint bootstrap spread.
        executeCompute("peerLocation") {
            for ((_, track) in tracks) {
                try {
                    val (x, y, z) = track.est.latLonToENU(lat, lon, alt)
                    track.est.addPeerPose(x, y, z)
                } catch (_: Throwable) { /* swallow */ }
            }
        }

        // Anti-collision warning — print at most once per 2 s.
        val ownGps = lastGpsOrNull() ?: return
        val dE = (lon - ownGps.lon) * 111_319.49 * kotlin.math.cos(Math.toRadians(lat))
        val dN = (lat - ownGps.lat) * 111_319.49
        val dU = alt - ownGps.alt
        val dist3D = kotlin.math.sqrt(dE * dE + dN * dN + dU * dU)
        val now = System.currentTimeMillis()
        if (dist3D < 10.0 && now - lastPeerDistanceWarnMs > 2_000L) {
            appendLog("WARNING: peer drone $peerId is ${"%.1f".format(dist3D)} m from this OSA — anti-collision threshold!")
            lastPeerDistanceWarnMs = now
        }
    }

    /**
     * Called when /<peerAgentId>/ftm_rtt publishes a JSON RTT measurement.
     *
     * Always logged for post-processing (appended to rtt.jsonl with
     * type="peer_rtt").  In cooperative_mode == "fused_onboard", the RTT is
     * also paired with the peer's GNSS fix and injected into the local
     * anchor estimator for joint filtering — experimental.
     */
    private fun handlePeerRttJson(json: String) {
        if (experimentConfig.cooperativeMode == ExperimentConfig.COOP_INDEPENDENT) return
        lastPeerRttMs = System.currentTimeMillis()
        fileLogger.appendTaggedJson("rtt.jsonl", "peer_rtt", json)
        // Note: on-board fusion of peer RTT is intentionally deferred; the
        // recommended path is fused_offboard via the external fusion node,
        // which avoids relying on the possibly unstable Wi-Fi mesh between
        // two UAVs during flight.
    }





    /* Buffer GNSS (thread-safe) */

    private fun pushGps(p: GpsPose) {
        synchronized(gpsLock) {
            gpsBuffer.addLast(p)
            while (gpsBuffer.size > maxGpsBuffer) gpsBuffer.removeFirst()
        }
        updateDroneVerticalExcursion()
    }

    /**
     * Recompute drone vertical excursion (max alt − min alt) over the rolling
     * GNSS buffer and publish it to the UI state [droneDzUiM]. Used as a
     * diagnostic chip in altitude_mode=auto: if this stays below ~3 m the
     * anchor Z is unlikely to converge.
     */
    private fun updateDroneVerticalExcursion() {
        var minZ = Double.POSITIVE_INFINITY
        var maxZ = Double.NEGATIVE_INFINITY
        synchronized(gpsLock) {
            if (gpsBuffer.isEmpty()) return
            for (g in gpsBuffer) {
                if (g.alt < minZ) minZ = g.alt
                if (g.alt > maxZ) maxZ = g.alt
            }
        }
        val dz = (maxZ - minZ).coerceAtLeast(0.0)
        runOnUiThread { droneDzUiM = dz }
    }

    private fun pruneOldGps(nowMonoMs: Long) {
        synchronized(gpsLock) {
            while (gpsBuffer.isNotEmpty() &&
                nowMonoMs - gpsBuffer.first().monoMs > MAX_SAMPLES_WINDOW_MS
            ) {
                gpsBuffer.removeFirst()
            }
        }
    }


    private fun lerp(a: Double, b: Double, f: Double): Double = a + (b - a) * f

    private data class GpsSync(
        val pose: GpsPose,
        val method: String,          // "interp" o "closest"
        val dtPrevMs: Long,
        val dtNextMs: Long,
        val bracketGapMs: Long
    )

    /**
     * Devuelve una pose GNSS alineada al instante del RTT.
     * - Si hay fixes prev/next cerca: interpola (mejor para dron en movimiento).
     * - Si no: fallback a "closest" dentro de syncWindowMs.
     */
    private fun syncGpsToRtt(rttMonoMs: Long, rttWallMs: Long): GpsSync? {
        var prev: GpsPose? = null
        var next: GpsPose? = null

        synchronized(gpsLock) {
            for (p in gpsBuffer) {
                val m = p.monoMs
                if (m <= rttMonoMs && (prev == null || m > prev!!.monoMs)) prev = p
                if (m >= rttMonoMs && (next == null || m < next!!.monoMs)) next = p
            }
        }

        // 1) Interpolación si hay bracket válido y gap razonable
        if (prev != null && next != null) {
            val gap = next!!.monoMs - prev!!.monoMs
            if (gap in 1..maxInterpBracketMs) {
                val f = (rttMonoMs - prev!!.monoMs).toDouble() / gap.toDouble()

                val lat = lerp(prev!!.lat, next!!.lat, f)
                val lon = lerp(prev!!.lon, next!!.lon, f)
                val alt = lerp(prev!!.alt, next!!.alt, f)

                // conservador: la peor precisión de los dos
                val acc = kotlin.math.max(prev!!.accM, next!!.accM)
                val src = if (prev!!.source == next!!.source) prev!!.source else "mixed"

                val poseAtRtt = GpsPose(
                    lat = lat,
                    lon = lon,
                    alt = alt,
                    tMs = rttWallMs,
                    monoMs = rttMonoMs,
                    accM = acc,
                    source = src
                )

                return GpsSync(
                    pose = poseAtRtt,
                    method = "interp",
                    dtPrevMs = rttMonoMs - prev!!.monoMs,
                    dtNextMs = next!!.monoMs - rttMonoMs,
                    bracketGapMs = gap
                )
            }
        }

        // 2) Fallback: closest dentro de ventana estricta
        val closest = findClosestGpsByMono(rttMonoMs, syncWindowMs) ?: return null
        val dt = rttMonoMs - closest.monoMs
        return GpsSync(
            pose = closest,            // aquí usamos la pose del fix más cercano (no interpolada)
            method = "closest",
            dtPrevMs = dt,
            dtNextMs = dt,
            bracketGapMs = 0L
        )
    }

    private fun findClosestGpsByMono(rttMonoMs: Long, windowMs: Long): GpsPose? {
        var best: GpsPose? = null
        var bestAbs = Long.MAX_VALUE
        synchronized(gpsLock) {
            for (p in gpsBuffer) {
                val d = abs(p.monoMs - rttMonoMs)
                if (d < bestAbs) {
                    bestAbs = d
                    best = p
                }
            }
        }
        return if (best != null && bestAbs <= windowMs) best else null
    }


    private fun lastGpsOrNull(): GpsPose? =
        synchronized(gpsLock) { gpsBuffer.lastOrNull() }

    /* -------------------------------------------------------------------------
     *  BSSID actual (para filtros)
     * ---------------------------------------------------------------------- */

    private var currentBssidGetter: (() -> String)? = null
    private fun targetBssid(): String =
        currentBssidGetter?.invoke()?.trim()?.lowercase() ?: ""

    companion object {
        private val macRegex =
            Regex("^[0-9a-f]{2}(:[0-9a-f]{2}){5}\$", RegexOption.IGNORE_CASE)

        private const val ACTION_USB_PERMISSION = "com.jbravo.osa_ftm.USB_PERMISSION"
        private const val ACTION_NTRIP_CONFIG = "com.jbravo.osa_ftm.NTRIP_CONFIG"
        private const val ACTION_EXPERIMENT_CONFIG = "com.jbravo.osa_ftm.EXPERIMENT_CONFIG"
        private const val ACTION_GROUND_TRUTH = "com.jbravo.osa_ftm.GROUND_TRUTH"
        private const val ACTION_REMOTE_CMD = "com.jbravo.osa_ftm.REMOTE_CMD"
    }
}

/* -------------------- COMPOSABLES AUXILIARES -------------------- */

@Composable
private fun NtripConfigCard(
    initialConfig: Config?,
    isConnected: Boolean,
    rtkStatus: GnssRtkStatus = GnssRtkStatus(),
    onConnect: (Config) -> Unit,
    onDisconnect: () -> Unit
) {
    var expanded by remember { mutableStateOf(false) }
    var host by remember { mutableStateOf(initialConfig?.ntripHost ?: "") }
    var port by remember { mutableStateOf((initialConfig?.ntripPort ?: 2101).toString()) }
    var mountpoint by remember { mutableStateOf(initialConfig?.mountpoint ?: "") }
    var user by remember { mutableStateOf(initialConfig?.ntripUser ?: "") }
    var pass by remember { mutableStateOf(initialConfig?.ntripPass ?: "") }

    // Update fields when config changes externally (e.g. via ADB)
    LaunchedEffect(initialConfig) {
        initialConfig?.let {
            host = it.ntripHost
            port = it.ntripPort.toString()
            mountpoint = it.mountpoint
            user = it.ntripUser
            pass = it.ntripPass
        }
    }

    ElevatedCard(
        modifier = Modifier.fillMaxWidth(),
        onClick = { expanded = !expanded }
    ) {
        Column(Modifier.padding(12.dp)) {
            Row(
                Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Column(Modifier.weight(1f)) {
                    Text(
                        "NTRIP RTK Corrections",
                        style = MaterialTheme.typography.titleMedium
                    )
                    Text(
                        if (isConnected) "Connected to $host/$mountpoint"
                        else if (host.isNotBlank()) "Configured: $host/$mountpoint"
                        else "Not configured",
                        style = MaterialTheme.typography.bodySmall,
                        color = if (isConnected) MaterialTheme.colorScheme.primary
                        else MaterialTheme.colorScheme.onSurfaceVariant
                    )
                }
                Surface(
                    color = when {
                        rtkStatus.isRtkFixed -> MaterialTheme.colorScheme.primary
                        rtkStatus.isRtkFloat -> MaterialTheme.colorScheme.tertiary
                        isConnected -> MaterialTheme.colorScheme.secondary
                        else -> MaterialTheme.colorScheme.surfaceVariant
                    },
                    shape = MaterialTheme.shapes.small
                ) {
                    Text(
                        when {
                            rtkStatus.isRtkFixed -> "RTK FIX"
                            rtkStatus.isRtkFloat -> "RTK FLT"
                            isConnected -> "NTRIP"
                            else -> "OFF"
                        },
                        modifier = Modifier.padding(horizontal = 8.dp, vertical = 4.dp),
                        color = when {
                            rtkStatus.isRtkFixed -> MaterialTheme.colorScheme.onPrimary
                            rtkStatus.isRtkFloat -> MaterialTheme.colorScheme.onTertiary
                            isConnected -> MaterialTheme.colorScheme.onSecondary
                            else -> MaterialTheme.colorScheme.onSurfaceVariant
                        },
                        style = MaterialTheme.typography.labelSmall
                    )
                }
            }

            // GNSS / RTK status strip (visible when external GNSS is active)
            if (rtkStatus.externalGnssActive) {
                Spacer(Modifier.height(6.dp))
                Row(
                    Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween
                ) {
                    // Fix quality label with color coding
                    val fixColor = when (rtkStatus.fixQuality) {
                        4 -> MaterialTheme.colorScheme.primary            // RTK Fixed — green
                        5 -> MaterialTheme.colorScheme.tertiary           // RTK Float — amber
                        2 -> MaterialTheme.colorScheme.secondary          // DGNSS
                        1 -> MaterialTheme.colorScheme.onSurfaceVariant   // GPS only
                        else -> MaterialTheme.colorScheme.error           // No fix
                    }
                    Text(
                        rtkStatus.fixLabel,
                        style = MaterialTheme.typography.labelMedium,
                        color = fixColor
                    )

                    Text(
                        "${rtkStatus.numSatellites} sats",
                        style = MaterialTheme.typography.labelMedium,
                        color = MaterialTheme.colorScheme.onSurfaceVariant
                    )

                    Text(
                        if (rtkStatus.hdop > 0) "HDOP ${"%.1f".format(rtkStatus.hdop)}" else "HDOP —",
                        style = MaterialTheme.typography.labelMedium,
                        color = MaterialTheme.colorScheme.onSurfaceVariant
                    )

                    val rtcmKb = rtkStatus.rtcmBytesReceived / 1024.0
                    Text(
                        if (rtkStatus.ntripConnected) "RTCM ${"%.0f".format(rtcmKb)} KB" else "RTCM —",
                        style = MaterialTheme.typography.labelMedium,
                        color = if (rtkStatus.ntripConnected) MaterialTheme.colorScheme.primary
                        else MaterialTheme.colorScheme.onSurfaceVariant
                    )
                }
            }

            AnimatedVisibility(visible = expanded) {
                Column(Modifier.padding(top = 8.dp)) {
                    OutlinedTextField(
                        value = host,
                        onValueChange = { host = it },
                        label = { Text("Caster host") },
                        placeholder = { Text("caster.example.com") },
                        singleLine = true,
                        modifier = Modifier.fillMaxWidth()
                    )
                    Spacer(Modifier.height(4.dp))
                    Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                        OutlinedTextField(
                            value = port,
                            onValueChange = { port = it.filter { c -> c.isDigit() } },
                            label = { Text("Port") },
                            singleLine = true,
                            modifier = Modifier.weight(1f)
                        )
                        OutlinedTextField(
                            value = mountpoint,
                            onValueChange = { mountpoint = it },
                            label = { Text("Mountpoint") },
                            singleLine = true,
                            modifier = Modifier.weight(2f)
                        )
                    }
                    Spacer(Modifier.height(4.dp))
                    Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                        OutlinedTextField(
                            value = user,
                            onValueChange = { user = it },
                            label = { Text("User") },
                            singleLine = true,
                            modifier = Modifier.weight(1f)
                        )
                        OutlinedTextField(
                            value = pass,
                            onValueChange = { pass = it },
                            label = { Text("Password") },
                            singleLine = true,
                            modifier = Modifier.weight(1f)
                        )
                    }
                    Spacer(Modifier.height(8.dp))
                    Row(
                        Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.End
                    ) {
                        if (isConnected) {
                            OutlinedButton(onClick = onDisconnect) {
                                Text("Disconnect")
                            }
                        } else {
                            Button(
                                onClick = {
                                    val cfg = Config(
                                        ntripHost = host.trim(),
                                        ntripPort = port.toIntOrNull() ?: 2101,
                                        mountpoint = mountpoint.trim(),
                                        ntripUser = user.trim(),
                                        ntripPass = pass.trim()
                                    )
                                    onConnect(cfg)
                                },
                                enabled = host.isNotBlank() && mountpoint.isNotBlank()
                            ) {
                                Text("Connect NTRIP")
                            }
                        }
                    }
                }
            }
        }
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
private fun MacSelector(
    macList: MutableList<String>,
    text: TextFieldValue,
    onTextChange: (TextFieldValue) -> Unit,
    expanded: Boolean,
    onExpandedChange: (Boolean) -> Unit,
    onPick: (String) -> Unit,
    onAdd: () -> Unit,
    onDelete: (String) -> Unit
) {
    Column {
        ExposedDropdownMenuBox(
            expanded = expanded,
            onExpandedChange = { onExpandedChange(!expanded) }
        ) {
            OutlinedTextField(
                value = text,
                onValueChange = onTextChange,
                label = { Text("(Optional) target BSSID") },
                placeholder = { Text("aa:bb:cc:dd:ee:ff") },
                trailingIcon = {
                    ExposedDropdownMenuDefaults.TrailingIcon(expanded = expanded)
                },
                modifier = Modifier
                    .menuAnchor()
                    .fillMaxWidth()
            )
            ExposedDropdownMenu(
                expanded = expanded,
                onDismissRequest = { onExpandedChange(false) }
            ) {

                // Opcion ALL:
                DropdownMenuItem(
                    text = { Text("All anchors") },
                    onClick = {
                        onPick("")              // BSSID vacío => sin filtro
                        onExpandedChange(false)
                    }
                )

                if (macList.isEmpty()) {
                    DropdownMenuItem(
                        text = { Text("No saved MACs") },
                        onClick = {}
                    )
                } else {
                    macList.forEach { mac ->
                        DropdownMenuItem(
                            text = {
                                Row(
                                    Modifier.fillMaxWidth(),
                                    horizontalArrangement = Arrangement.SpaceBetween
                                ) {
                                    Text(mac)
                                    Icon(
                                        Icons.Default.Delete,
                                        contentDescription = "Delete",
                                        modifier = Modifier
                                            .clickable { onDelete(mac) }
                                            .padding(start = 8.dp)
                                    )
                                }
                            },
                            onClick = { onPick(mac) }
                        )
                    }
                }
            }
        }

        Spacer(Modifier.height(8.dp))

        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.End) {
            FilledTonalButton(onClick = onAdd) {
                Icon(Icons.Default.Add, contentDescription = null)
                Spacer(Modifier.width(6.dp))
                Text("Add MAC")
            }
        }
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun WeightingSelector(
    modifier: Modifier = Modifier,
    onWeightingChange: (Weighting) -> Unit
) {
    val options = listOf("WLS", "Huber", "Trim", "SigmaClip")
    var expanded by remember { mutableStateOf(false) }
    var selected by remember { mutableStateOf(options[0]) }

    var huberDelta by remember { mutableStateOf(1.345f) }
    var trimFrac by remember { mutableStateOf(0.20f) }
    var sigmaK by remember { mutableStateOf(3.0f) }

    Column(modifier) {
        ExposedDropdownMenuBox(
            expanded = expanded,
            onExpandedChange = { expanded = !expanded }
        ) {
            OutlinedTextField(
                value = selected,
                onValueChange = {},
                readOnly = true,
                label = { Text("Weighting method") },
                trailingIcon = { ExposedDropdownMenuDefaults.TrailingIcon(expanded) },
                modifier = Modifier
                    .menuAnchor()
                    .fillMaxWidth()
            )
            ExposedDropdownMenu(
                expanded = expanded,
                onDismissRequest = { expanded = false }
            ) {
                options.forEach { opt ->
                    DropdownMenuItem(
                        text = { Text(opt) },
                        onClick = {
                            selected = opt
                            expanded = false
                            when (opt) {
                                "WLS" -> onWeightingChange(Weighting.WLS)
                                "Huber" -> onWeightingChange(
                                    Weighting.Huber(delta = huberDelta.toDouble())
                                )

                                "Trim" -> onWeightingChange(
                                    Weighting.Trim(fraction = trimFrac.toDouble())
                                )

                                "SigmaClip" -> onWeightingChange(
                                    Weighting.SigmaClip(k = sigmaK.toDouble())
                                )
                            }
                        }
                    )
                }
            }
        }

        when (selected) {
            "Huber" -> {
                Spacer(Modifier.height(8.dp))
                Text("Huber δ (delta): ${"%.3f".format(huberDelta)}")
                Slider(
                    value = huberDelta,
                    onValueChange = {
                        huberDelta = it.coerceIn(0.5f, 5f)
                        onWeightingChange(Weighting.Huber(delta = huberDelta.toDouble()))
                    },
                    valueRange = 0.5f..5f
                )
            }

            "Trim" -> {
                Spacer(Modifier.height(8.dp))
                Text("Trim fraction: ${(trimFrac * 100).toInt()}%")

                Slider(
                    value = trimFrac,
                    onValueChange = {
                        trimFrac = it.coerceIn(0f, 0.9f)
                        onWeightingChange(Weighting.Trim(fraction = trimFrac.toDouble()))
                    },
                    valueRange = 0f..0.9f
                )
            }

            "SigmaClip" -> {
                Spacer(Modifier.height(8.dp))
                Text("k (samples with |r|/σ > k are discarded): ${"%.2f".format(sigmaK)}")
                Slider(
                    value = sigmaK,
                    onValueChange = {
                        sigmaK = it.coerceAtLeast(0.5f)
                        onWeightingChange(Weighting.SigmaClip(k = sigmaK.toDouble()))
                    },
                    valueRange = 0.5f..8f
                )
            }
        }
    }
}


@Composable
private fun PreFlightConfigCard(
    config: ExperimentConfig,
    groundTruthCount: Int,
    onConfigChanged: (ExperimentConfig) -> Unit
) {
    var expanded by remember { mutableStateOf(false) }
    var label by remember { mutableStateOf(config.label) }
    var aglText by remember { mutableStateOf(config.aglMeters.toString()) }
    var biasText by remember { mutableStateOf(config.rangeBiasM.toString()) }
    var notes by remember { mutableStateOf(config.notes) }
    var selectedFilter by remember { mutableStateOf(config.filterMode) }
    var filterDropdownExpanded by remember { mutableStateOf(false) }
    var altitudeMode by remember { mutableStateOf(config.altitudeMode) }

    // Sync if changed externally (e.g. via ADB)
    LaunchedEffect(config) {
        label = config.label
        aglText = config.aglMeters.toString()
        biasText = config.rangeBiasM.toString()
        notes = config.notes
        selectedFilter = config.filterMode
        altitudeMode = config.altitudeMode
    }

    ElevatedCard(
        modifier = Modifier.fillMaxWidth(),
        onClick = { expanded = !expanded }
    ) {
        Column(Modifier.padding(12.dp)) {
            Row(
                Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Column(Modifier.weight(1f)) {
                    Text(
                        "Pre-flight config",
                        style = MaterialTheme.typography.titleMedium
                    )
                    val altSummary = if (config.altitudeMode == ExperimentConfig.ALT_MODE_MANUAL_AGL)
                        "${config.aglMeters}m AGL (manual)" else "alt=auto"
                    Text(
                        if (label.isNotBlank()) "$label — $altSummary — bias ${config.rangeBiasM}m — ${config.filterMode.name}"
                        else "$altSummary — bias ${config.rangeBiasM}m — ${config.filterMode.name}",
                        style = MaterialTheme.typography.bodySmall,
                        color = MaterialTheme.colorScheme.onSurfaceVariant
                    )
                }
                if (groundTruthCount > 0) {
                    Surface(
                        color = MaterialTheme.colorScheme.primaryContainer,
                        shape = MaterialTheme.shapes.small
                    ) {
                        Text(
                            "$groundTruthCount GT APs",
                            modifier = Modifier.padding(horizontal = 8.dp, vertical = 4.dp),
                            color = MaterialTheme.colorScheme.onPrimaryContainer,
                            style = MaterialTheme.typography.labelSmall
                        )
                    }
                }
                val rotation by animateFloatAsState(if (expanded) 180f else 0f, label = "arrow")
                Icon(
                    Icons.Default.ExpandMore,
                    contentDescription = null,
                    modifier = Modifier.rotate(rotation)
                )
            }

            AnimatedVisibility(visible = expanded) {
                Column {
                    Spacer(Modifier.height(8.dp))
                    OutlinedTextField(
                        value = label,
                        onValueChange = { label = it },
                        label = { Text("Experiment label") },
                        placeholder = { Text("e.g. flight_30m_run1") },
                        singleLine = true,
                        modifier = Modifier.fillMaxWidth()
                    )
                    Spacer(Modifier.height(4.dp))
                    // Altitude handling mode: AUTO (recommended — no AGL seed
                    // needed; 3D solver recovers anchor Z from vertical
                    // diversity of drone trajectories) or MANUAL_AGL (legacy).
                    Row(
                        Modifier.fillMaxWidth(),
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        Text("Altitude:", style = MaterialTheme.typography.bodyMedium)
                        Spacer(Modifier.width(8.dp))
                        FilterChip(
                            selected = altitudeMode == ExperimentConfig.ALT_MODE_AUTO,
                            onClick = { altitudeMode = ExperimentConfig.ALT_MODE_AUTO },
                            label = { Text("Auto (3D)") }
                        )
                        Spacer(Modifier.width(6.dp))
                        FilterChip(
                            selected = altitudeMode == ExperimentConfig.ALT_MODE_MANUAL_AGL,
                            onClick = { altitudeMode = ExperimentConfig.ALT_MODE_MANUAL_AGL },
                            label = { Text("Manual AGL") }
                        )
                    }
                    if (altitudeMode == ExperimentConfig.ALT_MODE_MANUAL_AGL) {
                        Spacer(Modifier.height(4.dp))
                        OutlinedTextField(
                            value = aglText,
                            onValueChange = { aglText = it.filter { c -> c.isDigit() || c == '.' } },
                            label = { Text("Flight altitude AGL (m)") },
                            singleLine = true,
                            modifier = Modifier.fillMaxWidth()
                        )
                    } else {
                        Spacer(Modifier.height(4.dp))
                        Text(
                            "No AGL seed required. Vary drone altitude during the run " +
                                "(≥3 m vertical excursion recommended) so the solver can " +
                                "recover the anchor Z.",
                            style = MaterialTheme.typography.bodySmall,
                            color = MaterialTheme.colorScheme.onSurfaceVariant
                        )
                    }
                    Spacer(Modifier.height(4.dp))
                    OutlinedTextField(
                        value = biasText,
                        onValueChange = { biasText = it.filter { c -> c.isDigit() || c == '.' || c == '-' } },
                        label = { Text("Range bias correction (m)") },
                        placeholder = { Text("e.g. 1.5 (subtracted from RTT)") },
                        singleLine = true,
                        modifier = Modifier.fillMaxWidth()
                    )
                    Spacer(Modifier.height(4.dp))
                    OutlinedTextField(
                        value = notes,
                        onValueChange = { notes = it },
                        label = { Text("Notes (optional)") },
                        placeholder = { Text("Weather, AP deployment, etc.") },
                        modifier = Modifier.fillMaxWidth(),
                        maxLines = 3
                    )
                    Spacer(Modifier.height(4.dp))
                    // Filter mode dropdown
                    Box {
                        OutlinedTextField(
                            value = selectedFilter.name,
                            onValueChange = {},
                            readOnly = true,
                            label = { Text("Filter mode") },
                            trailingIcon = {
                                Icon(
                                    Icons.Default.ExpandMore,
                                    contentDescription = null,
                                    modifier = Modifier.clickable { filterDropdownExpanded = true }
                                )
                            },
                            modifier = Modifier
                                .fillMaxWidth()
                                .clickable { filterDropdownExpanded = true },
                            singleLine = true
                        )
                        DropdownMenu(
                            expanded = filterDropdownExpanded,
                            onDismissRequest = { filterDropdownExpanded = false }
                        ) {
                            FilterMode.entries.forEach { fm ->
                                DropdownMenuItem(
                                    text = { Text(fm.name) },
                                    onClick = {
                                        selectedFilter = fm
                                        filterDropdownExpanded = false
                                    }
                                )
                            }
                        }
                    }
                    Spacer(Modifier.height(8.dp))
                    Row(
                        Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.End
                    ) {
                        Button(
                            onClick = {
                                val agl = aglText.toDoubleOrNull() ?: config.aglMeters
                                val bias = biasText.toDoubleOrNull() ?: config.rangeBiasM
                                onConfigChanged(
                                    config.copy(
                                        label = label.trim(),
                                        aglMeters = agl,
                                        notes = notes.trim(),
                                        filterMode = selectedFilter,
                                        rangeBiasM = bias,
                                        altitudeMode = altitudeMode
                                    )
                                )
                            }
                        ) {
                            Text("Apply")
                        }
                    }
                }
            }
        }
    }
}
