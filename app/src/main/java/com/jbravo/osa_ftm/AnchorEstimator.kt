package com.jbravo.osa_ftm

import kotlin.math.abs
import kotlin.math.ceil
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.PI
import kotlin.math.sqrt
data class GpsPose(
    val lat: Double,
    val lon: Double,
    val alt: Double,
    val tMs: Long,       // epoch ms
    val monoMs: Long,    // elapsedRealtime ms
    val accM: Double,
    val source: String   // "internal" / "external" / "mixed"
)


data class RttSample(val tMs: Long, val distance: Double, val std: Double, val rssi: Int)

/**
 * Muestra en coordenadas ENU (East-North-Up) respecto a (lat0, lon0, alt0).
 *
 * @param x East (m)
 * @param y North (m)
 * @param z Up (m)
 * @param r distancia medida por RTT (m)
 * @param sigma desviación estándar asociada a r (m)
 */
data class ENUSample(
    val x: Double,
    val y: Double,
    val z: Double,
    val r: Double,
    val sigma: Double
)

/**
 * Métodos de ponderación robusta:
 * - WLS: pesos clásicos 1/σ²
 * - Huber(delta): suaviza outliers (cuadrático cerca de 0, lineal en colas)
 * - Trim(fraction): descarta el top 'fraction' (0..1) de residuos normalizados |r|/σ en cada iteración
 * - SigmaClip(k): descarta muestras con |r|/σ > k (ladder/step-down)
 */
sealed class Weighting {
    data object WLS : Weighting()

    data class Huber(val delta: Double = 1.345) : Weighting()

    data class Trim(val fraction: Double = 0.2) : Weighting() {
        init {
            require(fraction in 0.0..0.9) { "fraction must be in [0, 0.9]" }
        }
    }

    data class SigmaClip(val k: Double = 3.0) : Weighting() {
        init {
            require(k > 0.0) { "k must be > 0" }
        }
    }
}

/**
 * Estimador robusto de la posición de un anchor FTM en ENU (3D completo).
 *
 * Modelo 3D:
 *  - Las observaciones RTT se interpretan como distancias 3D entre el anchor
 *    (x, y, z) y el teléfono (x_i, y_i, z_i).
 *  - Se estiman simultáneamente (x, y, z) del anchor en el sistema ENU local.
 *  - La altitud del teléfono viene en WGS84 y se proyecta a ENU respecto a (lat0, lon0, alt0).
 *
 * Uso de z_dron (AGL):
 *  - Se asume que el dron vuela a una altura aproximadamente constante H_AGL
 *    por encima del anchor (o del suelo donde está el AP).
 *  - Se pasa este valor como droneAglMeters.
 *  - Se usa para:
 *      • construir una mejor solución lineal inicial en XY (corrigiendo el radio RTT
 *        por la componente vertical conocida),
 *      • inicializar la z del anchor alrededor de z_anchor ≈ z_dron - H_AGL (mediana).
 *
 * NOTA:
 *  - lat0, lon0, alt0 definen el origen ENU.
 *    Un uso típico es fijar alt0 = altitud GNSS del primer fix del dron.
 *  - Si droneAglMeters = 0.0, el algoritmo sigue funcionando, pero sin aprovechar
 *    la información de AGL (revierte a un inicializado más "ciego" en z).
 */
class AnchorEstimator(
    private val lat0: Double,
    private val lon0: Double,
    private val alt0: Double,
    private val droneAglMeters: Double   // altura dron-anchor estimada (AGL)
) {

    private val R = 6_378_137.0 // radio aproximado WGS84 (m)

    // Estado interno (protegido por 'lock')
    private val lock = Any()
    private val samples = mutableListOf<ENUSample>()
    private var weighting: Weighting = Weighting.WLS

    /**
     * Poses ENU del *peer* drone (cooperative mode).  Se usan únicamente
     * para ampliar la medida de diversidad geométrica ([geometricSpread2D],
     * [geometricSpread3D], [azimuthCoverageDeg]) y *no* se añaden al GN ni
     * al IEKF locales — eso sólo ocurre si el modo es "fused_onboard" y
     * las RTT del peer también se insertan vía [addSampleFromGpsAndRtt].
     *
     * Buffer acotado para evitar crecimiento sin límite.
     */
    private data class PeerPose(val x: Double, val y: Double, val z: Double)
    private val peerPoses = ArrayDeque<PeerPose>()
    private val peerPoseCap = 512

    /**
     * Estimación 3D del anchor en ENU.
     *
     * @param x East (m)
     * @param y North (m)
     * @param z Up (m)
     * @param sigma2 varianza residual (aprox. del ruido sobre las distancias)
     */
    data class Estimate3D(
        val x: Double,
        val y: Double,
        val z: Double,
        val sigma2: Double
    )

    /* ---------------------------------------------------------------------
     *  Configuración de pesos y gestión de muestras (thread-safe)
     * ------------------------------------------------------------------ */

    /** Cambia el esquema de ponderación. Seguro frente a hilos. */
    fun setWeighting(w: Weighting) {
        synchronized(lock) {
            weighting = w
        }
    }

    /** Borra todas las muestras acumuladas. */
    fun clear() {
        synchronized(lock) {
            samples.clear()
            peerPoses.clear()
        }
    }

    /** Número de muestras actualmente acumuladas. */
    fun sampleCount(): Int =
        synchronized(lock) { samples.size }

    /**
     * Añade una pose ENU del peer drone al buffer de diversidad geométrica.
     *
     * No afecta al GN/IEKF: sólo se usa para ampliar el spread usado por el
     * gate de bootstrap.  Llamar desde el subscriber ROS 2 de
     * /<peer>/phone/location.
     */
    fun addPeerPose(x: Double, y: Double, z: Double) {
        if (!x.isFinite() || !y.isFinite() || !z.isFinite()) return
        synchronized(lock) {
            peerPoses.addLast(PeerPose(x, y, z))
            while (peerPoses.size > peerPoseCap) peerPoses.removeFirst()
        }
    }

    /** Number of peer poses currently in buffer. */
    fun peerPoseCount(): Int = synchronized(lock) { peerPoses.size }

    /**
     * Diagonal del bounding-box 2D (EN) de las posiciones del dron acumuladas.
     * Sirve para evaluar si hay suficiente diversidad geométrica antes del
     * bootstrap GN.
     *
     * Si hay poses del peer añadidas vía [addPeerPose], se incluyen en el
     * bounding-box — esto permite superar el umbral de bootstrap en vuelos
     * cooperativos cortos donde ningún dron individual acumula suficiente
     * spread.
     */
    fun geometricSpread2D(): Double = synchronized(lock) {
        if (samples.isEmpty() && peerPoses.isEmpty()) return 0.0
        if ((samples.size + peerPoses.size) < 2) return 0.0
        var minX = Double.POSITIVE_INFINITY
        var maxX = Double.NEGATIVE_INFINITY
        var minY = Double.POSITIVE_INFINITY
        var maxY = Double.NEGATIVE_INFINITY
        for (s in samples) {
            if (s.x < minX) minX = s.x; if (s.x > maxX) maxX = s.x
            if (s.y < minY) minY = s.y; if (s.y > maxY) maxY = s.y
        }
        for (p in peerPoses) {
            if (p.x < minX) minX = p.x; if (p.x > maxX) maxX = p.x
            if (p.y < minY) minY = p.y; if (p.y > maxY) maxY = p.y
        }
        val dx = (maxX - minX)
        val dy = (maxY - minY)
        sqrt(dx * dx + dy * dy)
    }

    /**
     * Diagonal del bounding-box 3D (EN-Up) incluyendo poses del peer.
     * Valor alto ⇒ buena observabilidad vertical.
     */
    fun geometricSpread3D(): Double = synchronized(lock) {
        if ((samples.size + peerPoses.size) < 2) return 0.0
        var minX = Double.POSITIVE_INFINITY
        var maxX = Double.NEGATIVE_INFINITY
        var minY = Double.POSITIVE_INFINITY
        var maxY = Double.NEGATIVE_INFINITY
        var minZ = Double.POSITIVE_INFINITY
        var maxZ = Double.NEGATIVE_INFINITY
        for (s in samples) {
            if (s.x < minX) minX = s.x; if (s.x > maxX) maxX = s.x
            if (s.y < minY) minY = s.y; if (s.y > maxY) maxY = s.y
            if (s.z < minZ) minZ = s.z; if (s.z > maxZ) maxZ = s.z
        }
        for (p in peerPoses) {
            if (p.x < minX) minX = p.x; if (p.x > maxX) maxX = p.x
            if (p.y < minY) minY = p.y; if (p.y > maxY) maxY = p.y
            if (p.z < minZ) minZ = p.z; if (p.z > maxZ) maxZ = p.z
        }
        val dx = (maxX - minX); val dy = (maxY - minY); val dz = (maxZ - minZ)
        sqrt(dx * dx + dy * dy + dz * dz)
    }

    /**
     * Cobertura angular (en grados) de los azimuts de las observaciones
     * respecto a un punto de referencia (normalmente, la última estimación
     * del anchor).  Si [refX] / [refY] son NaN, se usa el centroide de las
     * muestras como aproximación.
     *
     * Devuelve el rango cubierto en [0, 360].  Un rango pequeño (< 90°)
     * indica que el vuelo es prácticamente colineal respecto al AP y la
     * geometría está mal condicionada para triangular.
     */
    fun azimuthCoverageDeg(refX: Double = Double.NaN, refY: Double = Double.NaN): Double =
        synchronized(lock) {
            val n = samples.size + peerPoses.size
            if (n < 2) return 0.0

            val cx: Double
            val cy: Double
            if (refX.isFinite() && refY.isFinite()) {
                cx = refX; cy = refY
            } else {
                var sx = 0.0; var sy = 0.0
                for (s in samples) { sx += s.x; sy += s.y }
                for (p in peerPoses) { sx += p.x; sy += p.y }
                cx = sx / n; cy = sy / n
            }

            // Bucketea los ángulos en bins de 5° y mide el arco cubierto.
            val bins = BooleanArray(72) // 360 / 5
            fun add(x: Double, y: Double) {
                val dx = x - cx; val dy = y - cy
                if (dx == 0.0 && dy == 0.0) return
                var ang = kotlin.math.atan2(dy, dx) * 180.0 / PI
                if (ang < 0.0) ang += 360.0
                val idx = ((ang / 5.0).toInt()).coerceIn(0, bins.size - 1)
                bins[idx] = true
            }
            for (s in samples) add(s.x, s.y)
            for (p in peerPoses) add(p.x, p.y)

            val covered = bins.count { it }
            covered * 5.0
        }

    /**
     * Añade una muestra RTT usando la posición GNSS del teléfono (en WGS84).
     *
     * @param gps  posición GNSS del teléfono (lat, lon, alt WGS84)
     * @param rtt  medición RTT con distancia [m] y desviación estándar [m]
     */
    fun addSampleFromGpsAndRtt(gps: GpsPose, rtt: RttSample) {
        val (x, y, z) = latLonToENU(gps.lat, gps.lon, gps.alt)
        val sigma = if (rtt.std > 0.0) rtt.std else 1.0
        val s = ENUSample(x, y, z, rtt.distance, sigma)
        synchronized(lock) {
            samples.add(s)
        }
    }

    /**
     * Crea una copia inmutable del conjunto de muestras y del esquema de pesos
     * para operar fuera del bloqueo (evita mantener el lock durante el cómputo).
     */
    private fun snapshot(): Pair<List<ENUSample>, Weighting> =
        synchronized(lock) {
            samples.toList() to weighting
        }

    /* ---------------------------------------------------------------------
     *  Conversión geográfica ↔ ENU (local)
     * ------------------------------------------------------------------ */

    private fun deg2rad(d: Double): Double = d * PI / 180.0
    private fun rad2deg(r: Double): Double = r * 180.0 / PI

    /**
     * Proyección local simple: lat/lon/alt → ENU respecto a (lat0, lon0, alt0).
     * Válido para distancias locales (decenas / pocos cientos de metros).
     */
    fun latLonToENU(
        lat: Double,
        lon: Double,
        alt: Double
    ): Triple<Double, Double, Double> {
        val dLat = deg2rad(lat - lat0)
        val dLon = deg2rad(lon - lon0)
        val meanLat = deg2rad((lat + lat0) / 2.0)
        val x = dLon * cos(meanLat) * R   // East
        val y = dLat * R                  // North
        val z = alt - alt0                // Up (relativo a alt0)
        return Triple(x, y, z)
    }

    /**
     * ENU → lat/lon/alt (inversa aproximada).
     */
    fun enuToLatLon(
        x: Double,
        y: Double,
        z: Double
    ): Triple<Double, Double, Double> {
        val dLat = y / R
        val dLon = x / (R * cos(deg2rad(lat0)))
        val lat = lat0 + rad2deg(dLat)
        val lon = lon0 + rad2deg(dLon)
        val alt = alt0 + z
        return Triple(lat, lon, alt)
    }

    /* ---------------------------------------------------------------------
     *  Inicialización lineal en 2D (solo plano EN, corrigiendo AGL si se conoce)
     * ------------------------------------------------------------------ */

    /**
     * Corrige el radio RTT al radio horizontal, usando la AGL del dron si se conoce.
     *
     * r_total^2 = r_horizontal^2 + H_AGL^2  →  r_horizontal^2 ≈ r_total^2 - H_AGL^2
     *
     * Si el resultado es negativo (por ruido o AGL demasiado grande), se revierte
     * a r_total^2 para evitar números complejos.
     */
    private fun adjustedRadiusSquared(r: Double): Double {
        val r2 = r * r
        if (droneAglMeters <= 0.0) return r2

        val v2 = droneAglMeters * droneAglMeters
        val horiz2 = r2 - v2
        return if (horiz2 > 1e-6) horiz2 else r2
    }

    /**
     * Solución lineal aproximada en 2D (x,y) usando el plano EN y WLS clásico.
     *
     * - Usa las mismas ecuaciones de multilateración 2D que antes, pero sustituyendo
     *   r_i^2 por r_horizontal_i^2 ≈ r_i^2 - H_AGL^2 cuando H_AGL > 0.
     * - Esto mejora bastante la inicialización en XY cuando el dron está alto sobre el AP.
     */
    private fun linear2DSolution(samplesSnap: List<ENUSample>): Pair<Double, Double>? {
        if (samplesSnap.size < 3) return null

        val s1 = samplesSnap[0]
        val r1sq = adjustedRadiusSquared(s1.r)

        var ata00 = 0.0
        var ata01 = 0.0
        var ata11 = 0.0
        var atb0 = 0.0
        var atb1 = 0.0

        for (i in 1 until samplesSnap.size) {
            val si = samplesSnap[i]

            val rIsq = adjustedRadiusSquared(si.r)

            val A0 = 2.0 * (si.x - s1.x)
            val A1 = 2.0 * (si.y - s1.y)
            val C = r1sq - rIsq +
                    si.x * si.x - s1.x * s1.x +
                    si.y * si.y - s1.y * s1.y

            val w = 1.0 / (si.sigma * si.sigma) // WLS clásico
            ata00 += w * A0 * A0
            ata01 += w * A0 * A1
            ata11 += w * A1 * A1
            atb0 += w * A0 * C
            atb1 += w * A1 * C
        }

        val det = ata00 * ata11 - ata01 * ata01
        if (abs(det) < 1e-9) return null

        val inv00 = ata11 / det
        val inv01 = -ata01 / det
        val inv11 = ata00 / det

        val x = inv00 * atb0 + inv01 * atb1
        val y = inv01 * atb0 + inv11 * atb1
        return x to y
    }

    /**
     * Estima una z inicial para el anchor.
     *
     * Idea:
     *  - El dron está aproximadamente H_AGL por encima del anchor.
     *  - En ENU, z_dron_i = alt_dron_i - alt0.
     *  - Entonces z_anchor ≈ z_dron_i - H_AGL.
     *  - Tomamos la mediana de (z_i - H_AGL) para ser robustos.
     *
     * Si H_AGL no se ha configurado (<= 0), devolvemos 0.0 como valor neutro.
     */
    private fun initialZGuess(samplesSnap: List<ENUSample>): Double {
        if (samplesSnap.isEmpty()) return 0.0
        if (droneAglMeters <= 0.0) return 0.0

        val candidates = samplesSnap
            .map { it.z - droneAglMeters }
            .sorted()

        return candidates[candidates.size / 2]
    }

    /* ---------------------------------------------------------------------
     *  Gauss-Newton robusto 3D (x, y, z)
     * ------------------------------------------------------------------ */

    /**
     * Estimación 3D robusta del anchor mediante Gauss-Newton.
     *
     * Modelo:
     *  d_i(x, y, z) = sqrt( (x - x_i)² + (y - y_i)² + (z - z_i)² )
     *  r_i: distancia RTT medida.
     *
     * Se minimiza:
     *  Σ w_i * (d_i - r_i)²
     *
     * con pesos robustos definidos por 'weighting'.
     *
     * @param minSpread  spread mínimo 2D (m) de las posiciones del dron para
     *                   intentar el GN.  Con spread insuficiente la geometría
     *                   es degenerada y GN diverge.
     * @param minVerticalSpread  spread mínimo vertical (m) (max z − min z) de
     *                   las posiciones del dron. Relevante sólo cuando el Z
     *                   del anchor se estima sin prior (altitude_mode=auto):
     *                   con diversidad vertical insuficiente la dimensión Z
     *                   queda indeterminada. 0.0 deshabilita la puerta.
     * @return Estimate3D(x_anchor, y_anchor, z_anchor, sigma2_residual),
     *         o null si la geometría es degenerada o no hay suficientes muestras.
     */
    fun refineGaussNewton3D(
        maxIters: Int = 10,
        tol: Double = 1e-4,
        minSpread: Double = 15.0,
        minVerticalSpread: Double = 0.0
    ): Estimate3D? {
        val (samplesSnap, wScheme) = snapshot()
        if (samplesSnap.size < 4) return null

        // Diversidad geométrica: si las posiciones del dron están demasiado
        // agrupadas, la solución lineal / GN diverge.
        val xs = samplesSnap.map { it.x }
        val ys = samplesSnap.map { it.y }
        val dx = (xs.max() - xs.min())
        val dy = (ys.max() - ys.min())
        val spread = sqrt(dx * dx + dy * dy)
        if (spread < minSpread) return null

        if (minVerticalSpread > 0.0) {
            val zs = samplesSnap.map { it.z }
            val dz = zs.max() - zs.min()
            if (dz < minVerticalSpread) return null
        }

        // Inicialización en XY usando solución lineal 2D
        val init2D = linear2DSolution(samplesSnap) ?: return null
        var ax = init2D.first
        var ay = init2D.second

        // Inicialización en Z usando AGL (si está disponible)
        var az = initialZGuess(samplesSnap)

        data class Row(
            val idx: Int,
            val dx: Double,
            val dy: Double,
            val dz: Double,
            val di: Double,
            val resid: Double,
            val absrNorm: Double
        )

        for (iter in 0 until maxIters) {
            val rows = ArrayList<Row>(samplesSnap.size)

            // 1) Calcula residuos y residuos normalizados
            for ((i, s) in samplesSnap.withIndex()) {
                val dx = ax - s.x
                val dy = ay - s.y
                val dz = az - s.z
                val di = sqrt(dx * dx + dy * dy + dz * dz + 1e-12)
                val resid = di - s.r
                val absrNorm = abs(resid) / s.sigma
                rows += Row(i, dx, dy, dz, di, resid, absrNorm)
            }

            // 2) Para Trim / SigmaClip, determina qué muestras excluir (peso 0)
            val excluded = BooleanArray(samplesSnap.size) { false }
            when (wScheme) {
                is Weighting.Trim -> {
                    val n = samplesSnap.size
                    val k = ceil(wScheme.fraction * n).toInt().coerceAtMost(n)
                    if (k > 0) {
                        val sorted = rows.sortedByDescending { it.absrNorm }
                        for (j in 0 until k) {
                            excluded[sorted[j].idx] = true
                        }
                    }
                }

                is Weighting.SigmaClip -> {
                    for (r in rows) {
                        if (r.absrNorm > wScheme.k) excluded[r.idx] = true
                    }
                }

                is Weighting.WLS,
                is Weighting.Huber -> {
                    // sin exclusión dura
                }
            }

            // 3) Acumula J^T W J (3x3 simétrica) y J^T W r (3x1) con pesos robustos
            var j00 = 0.0
            var j01 = 0.0
            var j02 = 0.0
            var j11 = 0.0
            var j12 = 0.0
            var j22 = 0.0

            var g0 = 0.0
            var g1 = 0.0
            var g2 = 0.0

            var anyIncluded = false

            for (r in rows) {
                if (excluded[r.idx]) continue
                val s = samplesSnap[r.idx]

                val j0 = r.dx / r.di // ∂d/∂x
                val j1 = r.dy / r.di // ∂d/∂y
                val j2 = r.dz / r.di // ∂d/∂z

                val base = 1.0 / (s.sigma * s.sigma)
                val robustFactor = when (wScheme) {
                    is Weighting.WLS -> 1.0
                    is Weighting.Huber -> {
                        val t = r.absrNorm
                        if (t <= wScheme.delta) 1.0 else wScheme.delta / t
                    }

                    is Weighting.Trim,
                    is Weighting.SigmaClip -> 1.0 // ya excluimos arriba
                }
                val wTot = base * robustFactor

                anyIncluded = true

                j00 += wTot * j0 * j0
                j01 += wTot * j0 * j1
                j02 += wTot * j0 * j2
                j11 += wTot * j1 * j1
                j12 += wTot * j1 * j2
                j22 += wTot * j2 * j2

                g0 += wTot * j0 * r.resid
                g1 += wTot * j1 * r.resid
                g2 += wTot * j2 * r.resid
            }

            if (!anyIncluded) return null

            val inv = invertSymmetric3x3(j00, j01, j02, j11, j12, j22) ?: return null
            val inv00 = inv[0]
            val inv01 = inv[1]
            val inv02 = inv[2]
            val inv11 = inv[3]
            val inv12 = inv[4]
            val inv22 = inv[5]

            // Paso de Gauss-Newton: - (J^T W J)^(-1) J^T W r
            val stepX = -(inv00 * g0 + inv01 * g1 + inv02 * g2)
            val stepY = -(inv01 * g0 + inv11 * g1 + inv12 * g2)
            val stepZ = -(inv02 * g0 + inv12 * g1 + inv22 * g2)

            ax += stepX
            ay += stepY
            az += stepZ

            if (sqrt(stepX * stepX + stepY * stepY + stepZ * stepZ) < tol) {
                break // convergencia
            }
        }

        // 4) Varianza residual (chi² / dof) con los mismos pesos robustos
        val n = samplesSnap.size
        val dof = max(1, n - 3)
        var chi2 = 0.0

        data class R2(val idx: Int, val resid: Double, val absrNorm: Double)
        val residuals = samplesSnap.mapIndexed { i, s ->
            val dx = ax - s.x
            val dy = ay - s.y
            val dz = az - s.z
            val d = sqrt(dx * dx + dy * dy + dz * dz + 1e-12)
            val resid = d - s.r
            R2(i, resid, abs(resid) / s.sigma)
        }

        val excluded = BooleanArray(n) { false }
        when (wScheme) {
            is Weighting.Trim -> {
                val k = ceil(wScheme.fraction * n).toInt().coerceAtMost(n)
                if (k > 0) {
                    val sorted = residuals.sortedByDescending { it.absrNorm }
                    for (j in 0 until k) {
                        excluded[sorted[j].idx] = true
                    }
                }
            }

            is Weighting.SigmaClip -> {
                for (rr in residuals) {
                    if (rr.absrNorm > wScheme.k) excluded[rr.idx] = true
                }
            }

            is Weighting.WLS,
            is Weighting.Huber -> {
                // sin exclusión dura
            }
        }

        for (rr in residuals) {
            if (excluded[rr.idx]) continue
            val s = samplesSnap[rr.idx]

            val base = 1.0 / (s.sigma * s.sigma)
            val robustFactor = when (wScheme) {
                is Weighting.WLS -> 1.0
                is Weighting.Huber -> {
                    val t = rr.absrNorm
                    if (t <= wScheme.delta) 1.0 else wScheme.delta / t
                }

                is Weighting.Trim,
                is Weighting.SigmaClip -> 1.0
            }
            val wTot = base * robustFactor

            chi2 += wTot * rr.resid * rr.resid
        }

        // Sanity check: el resultado debe estar dentro de un rango razonable
        // respecto al centroide de las observaciones.  Si GN diverge (spread
        // marginal) el resultado puede quedar a millones de metros.
        val cx = samplesSnap.map { it.x }.average()
        val cy = samplesSnap.map { it.y }.average()
        val maxRange = samplesSnap.maxOf { it.r }
        val distFromCentroid = sqrt((ax - cx) * (ax - cx) + (ay - cy) * (ay - cy))
        if (distFromCentroid > 2.0 * maxRange) return null

        val sigma2 = chi2 / dof.toDouble()
        return Estimate3D(ax, ay, az, sigma2)
    }

    /**
     * Versión 2.5D legacy: sólo devuelve (x, y, sigma2).
     *
     * Mantiene compatibilidad con el código antiguo. Internamente llama a
     * refineGaussNewton3D() y descarta la z.
     */
    @Deprecated(
        message = "Usa refineGaussNewton3D() para obtener (x,y,z,sigma2)",
        replaceWith = ReplaceWith("refineGaussNewton3D(maxIters, tol)")
    )
    fun refineGaussNewton(
        maxIters: Int = 10,
        tol: Double = 1e-4
    ): Triple<Double, Double, Double>? {
        val est = refineGaussNewton3D(maxIters, tol) ?: return null
        return Triple(est.x, est.y, est.sigma2)
    }

    /* ---------------------------------------------------------------------
     *  Covarianzas (3x3 completa y 2x2 en plano EN)
     * ------------------------------------------------------------------ */

    /**
     * Covarianza 3x3 en (x, y, z) evaluada en (x, y, z) usando el mismo esquema
     * de pesos robustos. Devuelve [Σxx, Σxy, Σxz, Σyy, Σyz, Σzz] o null si la
     * geometría es degenerada.
     */
    fun covariance3x3At(
        x: Double,
        y: Double,
        z: Double,
        sigma2: Double
    ): DoubleArray? {
        val (samplesSnap, wScheme) = snapshot()
        if (samplesSnap.size < 3) return null

        data class Row(
            val idx: Int,
            val dx: Double,
            val dy: Double,
            val dz: Double,
            val di: Double,
            val resid: Double,
            val absrNorm: Double
        )

        val rows = ArrayList<Row>(samplesSnap.size)
        for ((i, s) in samplesSnap.withIndex()) {
            val dx = x - s.x
            val dy = y - s.y
            val dz = z - s.z
            val di = sqrt(dx * dx + dy * dy + dz * dz + 1e-12)
            val resid = di - s.r
            val absrNorm = abs(resid) / s.sigma
            rows += Row(i, dx, dy, dz, di, resid, absrNorm)
        }

        val excluded = BooleanArray(samplesSnap.size) { false }
        when (wScheme) {
            is Weighting.Trim -> {
                val n = samplesSnap.size
                val k = ceil(wScheme.fraction * n).toInt().coerceAtMost(n)
                if (k > 0) {
                    val sorted = rows.sortedByDescending { it.absrNorm }
                    for (j in 0 until k) {
                        excluded[sorted[j].idx] = true
                    }
                }
            }

            is Weighting.SigmaClip -> {
                for (r in rows) {
                    if (r.absrNorm > wScheme.k) excluded[r.idx] = true
                }
            }

            is Weighting.WLS,
            is Weighting.Huber -> {
                // sin exclusión dura
            }
        }

        var j00 = 0.0
        var j01 = 0.0
        var j02 = 0.0
        var j11 = 0.0
        var j12 = 0.0
        var j22 = 0.0

        var anyIncluded = false

        for (r in rows) {
            if (excluded[r.idx]) continue
            val s = samplesSnap[r.idx]

            val j0 = r.dx / r.di
            val j1 = r.dy / r.di
            val j2 = r.dz / r.di

            val base = 1.0 / (s.sigma * s.sigma)
            val robustFactor = when (wScheme) {
                is Weighting.WLS -> 1.0
                is Weighting.Huber -> {
                    val t = r.absrNorm
                    if (t <= wScheme.delta) 1.0 else wScheme.delta / t
                }

                is Weighting.Trim,
                is Weighting.SigmaClip -> 1.0
            }
            val wTot = base * robustFactor

            anyIncluded = true
            j00 += wTot * j0 * j0
            j01 += wTot * j0 * j1
            j02 += wTot * j0 * j2
            j11 += wTot * j1 * j1
            j12 += wTot * j1 * j2
            j22 += wTot * j2 * j2
        }

        if (!anyIncluded) return null

        val inv = invertSymmetric3x3(j00, j01, j02, j11, j12, j22) ?: return null
        val inv00 = inv[0]
        val inv01 = inv[1]
        val inv02 = inv[2]
        val inv11 = inv[3]
        val inv12 = inv[4]
        val inv22 = inv[5]

        // Escala por la varianza residual
        return doubleArrayOf(
            inv00 * sigma2, // Σxx
            inv01 * sigma2, // Σxy
            inv02 * sigma2, // Σxz
            inv11 * sigma2, // Σyy
            inv12 * sigma2, // Σyz
            inv22 * sigma2  // Σzz
        )
    }

    /**
     * Covarianza 2x2 en (x, y), extraída de la covarianza 3x3.
     *
     * Devuelve [Σxx, Σxy, Σyy] o null si la geometría es degenerada.
     */
    fun covariance2x2AtXY(
        x: Double,
        y: Double,
        z: Double,
        sigma2: Double
    ): DoubleArray? {
        val cov3 = covariance3x3At(x, y, z, sigma2) ?: return null
        val sxx = cov3[0]
        val sxy = cov3[1]
        val syy = cov3[3]
        return doubleArrayOf(sxx, sxy, syy)
    }

    /**
     * Versión legacy: covarianza 2x2 sin z (equivalente a z = 0).
     * Mantiene compatibilidad con el código viejo.
     */
    @Deprecated(
        message = "Usa covariance2x2AtXY(x, y, z, sigma2) con la z estimada del anchor",
        replaceWith = ReplaceWith("covariance2x2AtXY(x, y, 0.0, sigma2)")
    )
    fun covariance2x2At(
        x: Double,
        y: Double,
        sigma2: Double
    ): DoubleArray? = covariance2x2AtXY(x, y, 0.0, sigma2)

    /* ---------------------------------------------------------------------
     *  Utilidad: inversión de matriz 3x3 simétrica
     * ------------------------------------------------------------------ */

    /**
     * Invierte una matriz 3x3 simétrica:
     *
     *   [ a  b  c ]
     *   [ b  d  e ]
     *   [ c  e  f ]
     *
     * Recibe (a, b, c, d, e, f) = (j00, j01, j02, j11, j12, j22)
     * Devuelve [inv00, inv01, inv02, inv11, inv12, inv22] o null si det ~ 0.
     */
    private fun invertSymmetric3x3(
        a: Double, b: Double, c: Double,
        d: Double, e: Double,
        f: Double
    ): DoubleArray? {
        val det =
            a * (d * f - e * e) -
                    b * (b * f - c * e) +
                    c * (b * e - c * d)

        if (abs(det) < 1e-18) return null

        val inv00 = (d * f - e * e) / det
        val inv01 = (c * e - b * f) / det
        val inv02 = (b * e - c * d) / det
        val inv11 = (a * f - c * c) / det
        val inv12 = (b * c - a * e) / det
        val inv22 = (a * d - b * b) / det

        return doubleArrayOf(inv00, inv01, inv02, inv11, inv12, inv22)
    }
}
