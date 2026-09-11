package com.jbravo.osa_ftm;

import android.util.Log;

import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.interfaces.MessageDefinition;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.publisher.Publisher;
import org.ros2.rcljava.service.RMWRequestId;
import org.ros2.rcljava.service.Service;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

import std_msgs.msg.Float64MultiArray;
import std_srvs.srv.Trigger;
import std_srvs.srv.Trigger_Request;
import std_srvs.srv.Trigger_Response;

import std_msgs.msg.Float64MultiArray;
import org.ros2.rcljava.subscription.Subscription;

import org.ros2.rcljava.subscription.Subscription;
import std_msgs.msg.Float64;

/**
 * Nodo ROS 2 para la app WiFi RTT.
 *
 * Publica (con nombre resuelto a través de namespacePrefix):
 *  - ftm_rtt :
 *        std_msgs/String con JSON de medición RTT cruda.
 *
 *  - phone/location :
 *        std_msgs/Float64MultiArray con [lat, lon, alt, acc_m, t_ms].
 *
 *  - ftm/anchor/<ap_xxxxxxxxxxxx>/estimate :
 *        std_msgs/Float64MultiArray con
 *        [lat, lon, alt, cov_xx, cov_yy, cov_xy, nSamples, bssidHash].
 *
 * Servicios:
 *  - ftm/start_experiment : std_srvs/Trigger
 *  - ftm/stop_experiment  : std_srvs/Trigger
 *
 * Suscripciones de entrada:
 *  - ftm/command : std_msgs/String
 *        Alternativa fiable a los servicios Trigger (que pueden fallar
 *        con rcljava). Mensajes: "start", "stop", "wls", "huber",
 *        "trim", "sigmaclip".
 *  - ftm/set_agl : Float64MultiArray con [agl_m]
 */
public class Ros2RttNode {

    private static final String TAG = "Ros2RttNode";

    // Nodo ROS 2 subyacente
    private final Node node;

    // Namespace lógico para tópicos/servicios (sin barra inicial)
    private final String namespacePrefix;

    // Publishers base
    private final Publisher<std_msgs.msg.String> rttPub;
    private final Publisher<Float64MultiArray>   locPub;

    // Servicios ROS2 para controlar el experimento
    private Service<Trigger> startSrv;
    private Service<Trigger> stopSrv;

    // Nuevos servicios para cambiar el método de ponderación
    private Service<Trigger> setWlsSrv;
    private Service<Trigger> setHuberSrv;
    private Service<Trigger> setTrimSrv;
    private Service<Trigger> setSigmaClipSrv;

    private Subscription<Float64MultiArray> setAglSub;
    // Suscripción de comando por tópico (fallback fiable cuando servicios no funcionan)
    private Subscription<std_msgs.msg.String> commandSub;

    // Peer subscribers (cooperative OSA experiments) — created lazily via
    // startPeerSubscriptions() so they can be wired after the Activity has
    // registered the callbacks.
    private Subscription<Float64MultiArray> peerLocSub;
    private Subscription<std_msgs.msg.String> peerRttSub;
    private volatile String peerAgentId = "";
    // Callback hacia la Activity para aplicar el cambio de AGL
    public interface AglChangeCallback {
        void onAglChangeRequested(double aglMeters);
    }
    private volatile AglChangeCallback onAglChangeCallback;

    public void setOnAglChangeCallback(AglChangeCallback cb) {
        this.onAglChangeCallback = cb;
    }

    // Suscriptor para cambiar la AGL del dron vía tópico ftm/set_agl

    // Callback hacia la Activity para aplicar el cambio de ponderación
    public interface WeightingChangeCallback {
        void onWeightingChangeRequested(String methodName); // "WLS", "Huber", "Trim", "SigmaClip"
    }

    private volatile WeightingChangeCallback onWeightingChangeCallback;

    public void setOnWeightingChangeCallback(WeightingChangeCallback cb) {
        this.onWeightingChangeCallback = cb;
    }

    // Callback para cambio de ventana de sincronización GPS↔RTT
    public interface SyncWindowChangeCallback {
        void onSyncWindowChangeRequested(long windowMs);
    }
    private volatile SyncWindowChangeCallback onSyncWindowChangeCallback;
    public void setOnSyncWindowChangeCallback(SyncWindowChangeCallback cb) {
        this.onSyncWindowChangeCallback = cb;
    }

    /** Callback invoked whenever a peer drone publishes its GNSS location. */
    public interface PeerLocationCallback {
        void onPeerLocation(double lat, double lon, double alt, double accM, long tMs);
    }
    private volatile PeerLocationCallback onPeerLocation;
    public void setOnPeerLocationCallback(PeerLocationCallback cb) { this.onPeerLocation = cb; }

    /** Callback invoked whenever a peer drone publishes a raw RTT JSON measurement. */
    public interface PeerRttCallback {
        void onPeerRttJson(String json);
    }
    private volatile PeerRttCallback onPeerRtt;
    public void setOnPeerRttCallback(PeerRttCallback cb) { this.onPeerRtt = cb; }


    // Cache de publishers por anchor: topic -> publisher
    private final ConcurrentHashMap<String, Publisher<Float64MultiArray>> anchorPubs =
            new ConcurrentHashMap<>();

    // Publish diagnostics exposed through status.json for ADB/GUI debugging.
    private final AtomicLong rttPublishCount = new AtomicLong(0);
    private final AtomicLong locationPublishCount = new AtomicLong(0);
    private final AtomicLong anchorPublishCount = new AtomicLong(0);
    private final AtomicLong publishFailureCount = new AtomicLong(0);
    private volatile long lastRttPublishWallMs = 0L;
    private volatile long lastLocationPublishWallMs = 0L;
    private volatile long lastAnchorPublishWallMs = 0L;
    private volatile long lastPublishErrorWallMs = 0L;
    private volatile long lastPublishErrorLogWallMs = 0L;
    private volatile String lastPublishError = "";

    // Estado de ciclo de vida
    private volatile boolean started = false;
    private final AtomicBoolean disposed = new AtomicBoolean(false);

    // Callbacks hacia la Activity (se setean desde MainActivity)
    private volatile Runnable onStartExperiment = null;
    private volatile Runnable onStopExperiment  = null;

    // Hilo de spin del nodo
    private final Thread spinThread;

    /* --------------------------------------------------------------------- */
    /*  Constructores                                                        */
    /* --------------------------------------------------------------------- */

    public Ros2RttNode(String nodeName) {
        this(nodeName, null, true);
    }

    public Ros2RttNode(String nodeName, String namespacePrefix) {
        this(nodeName, namespacePrefix, true);
    }

    public Ros2RttNode(String nodeName, String namespacePrefix, boolean spinInBackground) {
        this.namespacePrefix = namespacePrefix;

        // Inicializa RCLJava (idempotente)
        RCLJava.rclJavaInit();

        // Crea nodo SIN namespace (lo metemos en los nombres de tópico)
        this.node = RCLJava.createNode(nodeName);

        // Publishers base
        this.rttPub = this.node.createPublisher(
                std_msgs.msg.String.class,
                resolveName("ftm_rtt")
        );
        this.locPub = this.node.createPublisher(
                Float64MultiArray.class,
                resolveName("phone/location")
        );

        // -----------------------------------------------------------------
        // Servicios: createService lanza NoSuchFieldException / IllegalAccessException
        // -----------------------------------------------------------------

        Service<Trigger> tmpStart  = null;
        Service<Trigger> tmpStop   = null;
        Service<Trigger> tmpWls    = null;
        Service<Trigger> tmpHuber  = null;
        Service<Trigger> tmpTrim   = null;
        Service<Trigger> tmpSigma  = null;

        Subscription<Float64MultiArray> tmpAglSub = null;

        try {
            // start_experiment
            tmpStart = this.node.<Trigger>createService(
                    Trigger.class,
                    resolveName("ftm/start_experiment"),
                    (RMWRequestId header, Trigger_Request req, Trigger_Response res) -> {
                        Log.i(TAG, "start_experiment recibido, req=" + req);

                        Runnable cb = onStartExperiment;
                        if (cb != null) {
                            try {
                                cb.run();
                                res.setSuccess(true);
                                res.setMessage("Experiment started");
                            } catch (Throwable t) {
                                Log.e(TAG, "Excepción en start callback", t);
                                res.setSuccess(false);
                                res.setMessage("Exception in start callback: " + t.getMessage());
                            }
                        } else {
                            res.setSuccess(false);
                            res.setMessage("No start callback registered");
                        }
                    }
            );

            // stop_experiment
            tmpStop = this.node.<Trigger>createService(
                    Trigger.class,
                    resolveName("ftm/stop_experiment"),
                    (RMWRequestId header, Trigger_Request req, Trigger_Response res) -> {
                        Log.i(TAG, "stop_experiment recibido, req=" + req);

                        Runnable cb = onStopExperiment;
                        if (cb != null) {
                            try {
                                cb.run();
                                res.setSuccess(true);
                                res.setMessage("Experiment stopped");
                            } catch (Throwable t) {
                                Log.e(TAG, "Excepción en stop callback", t);
                                res.setSuccess(false);
                                res.setMessage("Exception in stop callback: " + t.getMessage());
                            }
                        } else {
                            res.setSuccess(false);
                            res.setMessage("No stop callback registered");
                        }
                    }
            );

            // ---- NUEVOS: set_weighting_* ----

            tmpWls = this.node.<Trigger>createService(
                    Trigger.class,
                    resolveName("ftm/set_weighting_wls"),
                    (RMWRequestId header, Trigger_Request req, Trigger_Response res) -> {
                        Log.i(TAG, "set_weighting_wls recibido");
                        WeightingChangeCallback cb = onWeightingChangeCallback;
                        if (cb != null) {
                            try {
                                cb.onWeightingChangeRequested("WLS");
                                res.setSuccess(true);
                                res.setMessage("Weighting set to WLS");
                            } catch (Throwable t) {
                                Log.e(TAG, "Error aplicando WLS", t);
                                res.setSuccess(false);
                                res.setMessage("Exception setting WLS: " + t.getMessage());
                            }
                        } else {
                            res.setSuccess(false);
                            res.setMessage("No weighting callback registered");
                        }
                    }
            );

            // ---- NUEVO: tópico ftm/set_agl (AGL del dron en metros) ----
            tmpAglSub = this.node.createSubscription(
                    Float64MultiArray.class,
                    resolveName("ftm/set_agl"),
                    msg -> {
                        double agl = Double.NaN;
                        try {
                            // Soporta tanto double[] como List<Double>
                            Object raw = msg.getData();

                            if (raw instanceof double[]) {
                                double[] arr = (double[]) raw;
                                if (arr.length > 0) {
                                    agl = arr[0];
                                }
                            } else if (raw instanceof java.util.List) {
                                java.util.List<?> list = (java.util.List<?>) raw;
                                if (!list.isEmpty() && list.get(0) instanceof Number) {
                                    agl = ((Number) list.get(0)).doubleValue();
                                }
                            }
                        } catch (Throwable t) {
                            Log.e(TAG, "Error leyendo datos de ftm/set_agl", t);
                        }

                        if (Double.isNaN(agl)) {
                            Log.w(TAG, "ftm/set_agl recibido sin datos válidos");
                            return;
                        }

                        Log.i(TAG, "ftm/set_agl recibido, agl_m=" + agl);
                        AglChangeCallback cb = onAglChangeCallback;
                        if (cb != null) {
                            try {
                                cb.onAglChangeRequested(agl);
                            } catch (Throwable t) {
                                Log.e(TAG, "Error aplicando AGL", t);
                            }
                        } else {
                            Log.w(TAG, "ftm/set_agl recibido pero no hay AglChangeCallback registrado");
                        }
                    }
            );



            tmpHuber = this.node.<Trigger>createService(
                    Trigger.class,
                    resolveName("ftm/set_weighting_huber"),
                    (RMWRequestId header, Trigger_Request req, Trigger_Response res) -> {
                        Log.i(TAG, "set_weighting_huber recibido");
                        WeightingChangeCallback cb = onWeightingChangeCallback;
                        if (cb != null) {
                            try {
                                cb.onWeightingChangeRequested("Huber");
                                res.setSuccess(true);
                                res.setMessage("Weighting set to Huber");
                            } catch (Throwable t) {
                                Log.e(TAG, "Error aplicando Huber", t);
                                res.setSuccess(false);
                                res.setMessage("Exception setting Huber: " + t.getMessage());
                            }
                        } else {
                            res.setSuccess(false);
                            res.setMessage("No weighting callback registered");
                        }
                    }
            );

            tmpTrim = this.node.<Trigger>createService(
                    Trigger.class,
                    resolveName("ftm/set_weighting_trim"),
                    (RMWRequestId header, Trigger_Request req, Trigger_Response res) -> {
                        Log.i(TAG, "set_weighting_trim recibido");
                        WeightingChangeCallback cb = onWeightingChangeCallback;
                        if (cb != null) {
                            try {
                                cb.onWeightingChangeRequested("Trim");
                                res.setSuccess(true);
                                res.setMessage("Weighting set to Trim");
                            } catch (Throwable t) {
                                Log.e(TAG, "Error aplicando Trim", t);
                                res.setSuccess(false);
                                res.setMessage("Exception setting Trim: " + t.getMessage());
                            }
                        } else {
                            res.setSuccess(false);
                            res.setMessage("No weighting callback registered");
                        }
                    }
            );

            tmpSigma = this.node.<Trigger>createService(
                    Trigger.class,
                    resolveName("ftm/set_weighting_sigmaclip"),
                    (RMWRequestId header, Trigger_Request req, Trigger_Response res) -> {
                        Log.i(TAG, "set_weighting_sigmaclip recibido");
                        WeightingChangeCallback cb = onWeightingChangeCallback;
                        if (cb != null) {
                            try {
                                cb.onWeightingChangeRequested("SigmaClip");
                                res.setSuccess(true);
                                res.setMessage("Weighting set to SigmaClip");
                            } catch (Throwable t) {
                                Log.e(TAG, "Error aplicando SigmaClip", t);
                                res.setSuccess(false);
                                res.setMessage("Exception setting SigmaClip: " + t.getMessage());
                            }
                        } else {
                            res.setSuccess(false);
                            res.setMessage("No weighting callback registered");
                        }
                    }
            );

            Log.i(TAG, "Servicios Trigger creados OK: "
                    + resolveName("ftm/start_experiment") + ", "
                    + resolveName("ftm/stop_experiment") + ", "
                    + resolveName("ftm/set_weighting_wls") + ", "
                    + resolveName("ftm/set_weighting_huber") + ", "
                    + resolveName("ftm/set_weighting_trim") + ", "
                    + resolveName("ftm/set_weighting_sigmaclip"));

        } catch (NoSuchFieldException | IllegalAccessException e) {
            Log.e(TAG, "Error creando servicios Trigger", e);
        }

        // asignamos a los campos
        this.startSrv    = tmpStart;
        this.stopSrv     = tmpStop;
        this.setWlsSrv   = tmpWls;
        this.setHuberSrv = tmpHuber;
        this.setTrimSrv  = tmpTrim;
        this.setSigmaClipSrv = tmpSigma;
        this.setAglSub  = tmpAglSub;

        // ── Suscripción ftm/command (fallback fiable si los servicios Trigger
        //    no funcionan con ros2 service call desde el PC) ──
        // Mensajes aceptados: "start", "stop", "wls", "huber", "trim", "sigmaclip"
        this.commandSub = this.node.createSubscription(
                std_msgs.msg.String.class,
                resolveName("ftm/command"),
                cmdMsg -> {
                    String raw = cmdMsg.getData();
                    if (raw == null) return;
                    String cmd = raw.trim().toLowerCase();
                    Log.i(TAG, "ftm/command recibido: \"" + cmd + "\"");

                    switch (cmd) {
                        case "start": {
                            Runnable cb = onStartExperiment;
                            if (cb != null) cb.run();
                            else Log.w(TAG, "ftm/command=start pero no hay callback");
                            break;
                        }
                        case "stop": {
                            Runnable cb = onStopExperiment;
                            if (cb != null) cb.run();
                            else Log.w(TAG, "ftm/command=stop pero no hay callback");
                            break;
                        }
                        case "wls":
                        case "huber":
                        case "trim":
                        case "sigmaclip": {
                            WeightingChangeCallback cb = onWeightingChangeCallback;
                            if (cb != null) {
                                // Capitalizar para coincidir con el switch de MainActivity
                                String method = cmd.substring(0, 1).toUpperCase() + cmd.substring(1);
                                if (cmd.equals("wls")) method = "WLS";
                                else if (cmd.equals("sigmaclip")) method = "SigmaClip";
                                cb.onWeightingChangeRequested(method);
                            }
                            break;
                        }
                        default:
                            // set_sync_window:<ms>
                            if (cmd.startsWith("set_sync_window:")) {
                                try {
                                    long ms = Long.parseLong(cmd.substring("set_sync_window:".length()).trim());
                                    if (ms >= 10 && ms <= 30000) {
                                        SyncWindowChangeCallback swCb = onSyncWindowChangeCallback;
                                        if (swCb != null) swCb.onSyncWindowChangeRequested(ms);
                                        else Log.w(TAG, "set_sync_window pero no hay callback");
                                    } else {
                                        Log.w(TAG, "set_sync_window fuera de rango: " + ms);
                                    }
                                } catch (NumberFormatException e) {
                                    Log.w(TAG, "set_sync_window valor inválido: " + cmd);
                                }
                            } else {
                                Log.w(TAG, "ftm/command desconocido: \"" + cmd + "\"");
                            }
                    }
                }
        );
        Log.i(TAG, "Suscripción ftm/command creada: " + resolveName("ftm/command"));

        // marca el nodo como listo — ahora también funciona si los servicios fallaron,
        // mientras que la suscripción de comando siempre se crea.
        this.started = true;



        // Hilo de spin en background

        // Hilo de spin en background
        if (spinInBackground) {
            this.spinThread = new Thread(() -> {
                Log.i(TAG, "spin() arrancando en hilo " + Thread.currentThread().getName());
                try {
                    while (!disposed.get() && RCLJava.ok()) {
                        try {
                            // Procesa callbacks de ROS 2 (servicios, suscripciones, etc.)
                            RCLJava.spinSome(node);
                        } catch  (Throwable t) {
                            if (!disposed.get()) {
                                Log.e(TAG, "Error en spinSome()", t);
                            }
                            // Si hay un fallo gordo en el executor, salimos del bucle
                            break;
                        }

                        // Pequeña pausa para no quemar CPU
                        try {
                            Thread.sleep(10);
                        } catch (InterruptedException ignored) {
                        }
                    }
                    Log.i(TAG, "spin() terminado (disposed=" + disposed.get() + ")");
                } catch (Throwable t) {
                    if (!disposed.get()) {
                        Log.e(TAG, "Error inesperado en spin()", t);
                    } else {
                        Log.i(TAG, "spin() terminó tras dispose()", t);
                    }
                }
            }, "ros2-spin-" + nodeName);
            this.spinThread.setDaemon(true);
            this.spinThread.start();
        } else {
            this.spinThread = null;
        }


    }

    /* --------------------------------------------------------------------- */
    /*  API pública                                                          */
    /* --------------------------------------------------------------------- */

    public boolean isReady() {
        return started && !disposed.get();
    }

    public Node getNode() {
        return node;
    }

    public long getRttPublishCount() { return rttPublishCount.get(); }
    public long getLocationPublishCount() { return locationPublishCount.get(); }
    public long getAnchorPublishCount() { return anchorPublishCount.get(); }
    public long getPublishFailureCount() { return publishFailureCount.get(); }
    public long getLastRttPublishWallMs() { return lastRttPublishWallMs; }
    public long getLastLocationPublishWallMs() { return lastLocationPublishWallMs; }
    public long getLastAnchorPublishWallMs() { return lastAnchorPublishWallMs; }
    public long getLastPublishErrorWallMs() { return lastPublishErrorWallMs; }
    public String getLastPublishError() { return lastPublishError == null ? "" : lastPublishError; }

    /**
     * Subscribe to a peer OSA's GNSS location and raw RTT topics.
     *
     * Creates two subscriptions:
     *   /<peerAgentId>/phone/location (std_msgs/Float64MultiArray)
     *   /<peerAgentId>/ftm_rtt        (std_msgs/String JSON)
     *
     * Can be called at most once per instance.  Subsequent calls are ignored.
     * The peer topics are absolute (namespace-less) so both OSAs can locate
     * each other regardless of their own namespace.
     */
    public synchronized void startPeerSubscriptions(String peerAgentIdIn) {
        if (!isReady()) return;
        if (peerLocSub != null || peerRttSub != null) {
            Log.w(TAG, "Peer subscriptions already active, ignoring startPeerSubscriptions(" + peerAgentIdIn + ")");
            return;
        }
        String peer = peerAgentIdIn == null ? "" : peerAgentIdIn.trim();
        if (peer.isEmpty()) {
            Log.w(TAG, "startPeerSubscriptions: empty peer agent id, ignoring");
            return;
        }
        // Sanitize: peer namespace must not start with '/'
        peer = peer.replaceAll("^/+", "").replaceAll("/+$", "");
        this.peerAgentId = peer;

        String locTopic = "/" + peer + "/phone/location";
        String rttTopic = "/" + peer + "/ftm_rtt";

        try {
            peerLocSub = node.createSubscription(
                    Float64MultiArray.class,
                    locTopic,
                    msg -> {
                        try {
                            double[] arr = extractDoubleArray(msg);
                            if (arr == null || arr.length < 5) return;
                            PeerLocationCallback cb = onPeerLocation;
                            if (cb != null) {
                                cb.onPeerLocation(arr[0], arr[1], arr[2], arr[3], (long) arr[4]);
                            }
                        } catch (Throwable t) {
                            Log.w(TAG, "Error in peer location callback", t);
                        }
                    }
            );
            Log.i(TAG, "Peer location subscription created: " + locTopic);
        } catch (Throwable t) {
            Log.e(TAG, "Cannot create peer location subscription on " + locTopic, t);
        }

        try {
            peerRttSub = node.createSubscription(
                    std_msgs.msg.String.class,
                    rttTopic,
                    msg -> {
                        try {
                            PeerRttCallback cb = onPeerRtt;
                            if (cb != null && msg.getData() != null) {
                                cb.onPeerRttJson(msg.getData());
                            }
                        } catch (Throwable t) {
                            Log.w(TAG, "Error in peer RTT callback", t);
                        }
                    }
            );
            Log.i(TAG, "Peer RTT subscription created: " + rttTopic);
        } catch (Throwable t) {
            Log.e(TAG, "Cannot create peer RTT subscription on " + rttTopic, t);
        }
    }

    /** Return the active peer agent id, or empty string if none. */
    public String getPeerAgentId() {
        return peerAgentId == null ? "" : peerAgentId;
    }

    /** Extract a double[] from a Float64MultiArray regardless of its underlying list/array. */
    private static double[] extractDoubleArray(Float64MultiArray msg) {
        Object raw = msg.getData();
        if (raw instanceof double[]) {
            return (double[]) raw;
        } else if (raw instanceof java.util.List) {
            java.util.List<?> list = (java.util.List<?>) raw;
            double[] out = new double[list.size()];
            for (int i = 0; i < list.size(); i++) {
                Object v = list.get(i);
                if (v instanceof Number) out[i] = ((Number) v).doubleValue();
                else out[i] = Double.NaN;
            }
            return out;
        }
        return null;
    }

    public void setOnStartExperimentCallback(Runnable cb) {
        this.onStartExperiment = cb;
    }

    public void setOnStopExperimentCallback(Runnable cb) {
        this.onStopExperiment = cb;
    }

    /** Publica JSON RTT crudo en ftm_rtt (std_msgs/String). */
    public void publishRttJson(String json) {
        if (!isReady()) return;
        std_msgs.msg.String msg = new std_msgs.msg.String();
        msg.setData(json);
        if (safePublish(rttPub, msg)) {
            rttPublishCount.incrementAndGet();
            lastRttPublishWallMs = System.currentTimeMillis();
        }
    }

    /** Publica posición del teléfono en phone/location: [lat, lon, alt, acc_m, t_ms]. */
    public void publishLocation(
            double lat,
            double lon,
            double alt,
            double accM,
            long tMs
    ) {
        if (!isReady()) return;

        double[] payload = new double[]{
                lat,
                lon,
                alt,
                accM,
                (double) tMs
        };

        Float64MultiArray msg = new Float64MultiArray();
        setDataCompat(msg, payload);
        if (safePublish(locPub, msg)) {
            locationPublishCount.incrementAndGet();
            lastLocationPublishWallMs = System.currentTimeMillis();
        }
    }

    /**
     * Publica la estimación de anchor para un BSSID en:
     *
     *   ftm/anchor/<ap_xxxxxxxxxxxx>/estimate
     *
     * con data = [lat, lon, alt, cov_xx, cov_yy, cov_xy, nSamples, bssidHash].
     */
    public void publishAnchorEstimateFor(
            String bssid,
            double lat,
            double lon,
            double alt,
            double covXx,
            double covYy,
            double covXy,
            double nSamples,
            double bssidHash
    ) {
        if (!isReady()) return;

        String anchorId = sanitizeBssid(bssid);  // ap_xxxxxxxxxxxx
        String topic = resolveName("ftm/anchor/" + anchorId + "/estimate");

        Publisher<Float64MultiArray> pub = anchorPubs.get(topic);
        if (pub == null) {
            pub = node.createPublisher(Float64MultiArray.class, topic);
            anchorPubs.put(topic, pub);
        }

        double[] payload = new double[]{
                lat, lon, alt,
                covXx, covYy, covXy,
                nSamples, bssidHash
        };

        Float64MultiArray msg = new Float64MultiArray();
        setDataCompat(msg, payload);
        if (safePublish(pub, msg)) {
            anchorPublishCount.incrementAndGet();
            lastAnchorPublishWallMs = System.currentTimeMillis();
        }
    }

    /**
     * Publica el estado intermedio (no necesariamente estable) de un anchor en:
     *
     *   ftm/anchor/<ap_xxxxxxxxxxxx>/state
     *
     * con data = [lat, lon, alt, sigma_h, sigma_z, nAccepted, converged (0/1), bssidHash].
     *
     * Esto permite a la GUI mostrar estimaciones en progreso antes de
     * que la convergencia se confirme.
     */
    public void publishAnchorStateFor(
            String bssid,
            double lat,
            double lon,
            double alt,
            double sigmaH,
            double sigmaZ,
            long nAccepted,
            boolean converged,
            double bssidHash
    ) {
        if (!isReady()) return;

        String anchorId = sanitizeBssid(bssid);
        String topic = resolveName("ftm/anchor/" + anchorId + "/state");

        Publisher<Float64MultiArray> pub = anchorPubs.get(topic);
        if (pub == null) {
            pub = node.createPublisher(Float64MultiArray.class, topic);
            anchorPubs.put(topic, pub);
        }

        double[] payload = new double[]{
                lat, lon, alt,
                sigmaH, sigmaZ,
                (double) nAccepted,
                converged ? 1.0 : 0.0,
                bssidHash
        };

        Float64MultiArray msg = new Float64MultiArray();
        setDataCompat(msg, payload);
        if (safePublish(pub, msg)) {
            anchorPublishCount.incrementAndGet();
            lastAnchorPublishWallMs = System.currentTimeMillis();
        }
    }

    /* --------------------------------------------------------------------- */
    /*  Utilidades internas                                                  */
    /* --------------------------------------------------------------------- */

    /** Resuelve un nombre relativo usando namespacePrefix. */
    private String resolveName(String relative) {
        String rel = relative.replaceAll("^/+", ""); // quita barras iniciales
        String ns = namespacePrefix;
        if (ns == null || ns.trim().isEmpty()) {
            return "/" + rel;
        } else {
            ns = ns.replaceAll("^/+", "").replaceAll("/+$", "");
            return "/" + ns + "/" + rel;
        }
    }

    /** Convierte BSSID a id seguro de tópico (sólo [a-z0-9_], longitud limitada). */
    private String sanitizeBssid(String bssid) {
        String raw = (bssid == null ? "" : bssid).toLowerCase();
        String hexOnly = raw.replaceAll("[^0-9a-f:]", "");
        String compact = hexOnly.replace(":", "");
        String core = compact.isEmpty() ? "unknown" : compact;
        return "ap_" + core.substring(0, Math.min(core.length(), 32));
    }

    /** Publish que no hace crash si el publisher se invalida o el nodo se está cerrando. */
    private <T extends MessageDefinition> boolean safePublish(Publisher<T> pub, T msg) {
        try {
            pub.publish(msg);
            return true;
        } catch (Throwable t) {
            long now = System.currentTimeMillis();
            publishFailureCount.incrementAndGet();
            lastPublishErrorWallMs = now;
            lastPublishError = t.getClass().getSimpleName() + ": " + t.getMessage();
            if (now - lastPublishErrorLogWallMs >= 5_000L) {
                lastPublishErrorLogWallMs = now;
                Log.w(TAG, "ROS publish failed", t);
            }
            return false;
        }
    }

    /**
     * Compatibilidad con distintas implementaciones de std_msgs/Float64MultiArray.
     *
     * Primero intenta setData(double[]). Si falla por cualquier motivo,
     * prueba un setter setData(List<Double>) por reflexión.
     */
    private void setDataCompat(Float64MultiArray msg, double[] values) {
        // 1) Intento directo: setData(double[])
        try {
            msg.setData(values);
            return;
        } catch (Throwable ignored) {
            // seguimos probando
        }

        // 2) setter List<Double> vía reflexión
        try {
            java.lang.reflect.Method listSetter =
                    msg.getClass().getMethod("setData", java.util.List.class);
            java.util.List<Double> list = new java.util.ArrayList<>(values.length);
            for (double v : values) {
                list.add(v);
            }
            listSetter.invoke(msg, list);
            return;
        } catch (Exception ignored) {
            // si falla también, tiramos excepción controlada
        }

        throw new IllegalStateException(
                "No se pudo asignar Float64MultiArray.data (ni double[] ni List<Double>)"
        );
    }

    /* --------------------------------------------------------------------- */
    /*  Ciclo de vida                                                        */
    /* --------------------------------------------------------------------- */

    /** Cierra publishers, servicios, nodo y detiene el hilo de spin. */
    public void dispose() {
        if (!disposed.compareAndSet(false, true)) return;

        started = false;

        // 1) Pedimos shutdown del contexto ROS 2 para que ok() sea false
        try {
            RCLJava.shutdown();
        } catch (Throwable t) {
            Log.w(TAG, "Error en RCLJava.shutdown()", t);
        }

        // 2) Esperar a que el hilo de spin termine
        if (spinThread != null && spinThread.isAlive()) {
            try {
                spinThread.join(1000);  // esperamos hasta 1 segundo
            } catch (InterruptedException ignored) {
            }
        }

        // 3) Ahora es seguro liberar recursos ROS 2
        try {
            // Publishers dinámicos
            for (Map.Entry<String, Publisher<Float64MultiArray>> entry : anchorPubs.entrySet()) {
                try {
                    entry.getValue().dispose();
                } catch (Throwable ignored) {
                }
            }
            anchorPubs.clear();

            // Publishers base
            try { rttPub.dispose(); } catch (Throwable ignored) {}
            try { locPub.dispose(); } catch (Throwable ignored) {}

            // Servicios
            try { if (startSrv != null) startSrv.dispose(); } catch (Throwable ignored) {}
            try { if (stopSrv != null) stopSrv.dispose(); } catch (Throwable ignored) {}
            try { if (setWlsSrv != null) setWlsSrv.dispose(); } catch (Throwable ignored) {}
            try { if (setHuberSrv != null) setHuberSrv.dispose(); } catch (Throwable ignored) {}
            try { if (setTrimSrv != null) setTrimSrv.dispose(); } catch (Throwable ignored) {}
            try { if (setSigmaClipSrv != null) setSigmaClipSrv.dispose(); } catch (Throwable ignored) {}

            // Suscripción ftm/set_agl
            try { if (setAglSub != null) setAglSub.dispose(); } catch (Throwable ignored) {}

            // Peer subscriptions
            try { if (peerLocSub != null) peerLocSub.dispose(); } catch (Throwable ignored) {}
            try { if (peerRttSub != null) peerRttSub.dispose(); } catch (Throwable ignored) {}

            // Nodo
            try { node.dispose(); } catch (Throwable ignored) {}

        } finally {
            // Nada más que hacer: el hilo de spin ya está parado
        }
    }


}
