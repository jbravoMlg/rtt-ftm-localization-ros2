package com.jbravo.osa_ftm

import android.content.Context
import android.util.Log
import java.io.BufferedWriter
import java.io.File
import java.io.FileOutputStream
import java.io.OutputStreamWriter
import java.nio.charset.StandardCharsets
import java.text.SimpleDateFormat
import java.util.Locale
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong

/**
 * Logger asíncrono a ficheros JSONL:
 *
 *  - Un hilo de trabajo con cola (no bloquea la UI ni el hilo de cómputo).
 *  - BufferedWriter por archivo, abiertos bajo demanda.
 *  - Flush periódico y fsync periódico.
 *  - Opcionalmente, flush inmediato por línea (flushNow).
 *
 * Carpeta de salida:
 *   /sdcard/Android/data/<package>/files/ftm_logs  (si hay almacenamiento externo)
 *   o bien almacenamiento interno privado de la app.
 */
class FileLogger(ctx: Context) {

    private val tag = "FileLogger"

    /** Directorio base: externo si existe, interno en caso contrario. */
    private val baseDir: File = File(
        (ctx.getExternalFilesDir(null) ?: ctx.filesDir),
        "ftm_logs"
    ).apply {
        // mkdirs() es idempotente; si falla no lanzamos, pero se verá en el log
        if (!exists() && !mkdirs()) {
            Log.w(tag, "No se pudo crear directorio de logs: $absolutePath")
        }
    }

    /** Ruta para mostrarla en la UI/logs. */
    fun logsDirPath(): String = baseDir.absolutePath

    /**
     * Comando para el hilo worker:
     *
     *  - file: nombre del archivo relativo a baseDir.
     *  - line: línea a escribir (con '\n' incluido); si es null => mantenimiento global.
     *  - flushNow: si true, se fuerza flush tras escribir la línea.
     *  - fsyncNow: si true, se fuerza fsync tras escribir la línea.
     */
    private data class Cmd(
        val file: String,
        val line: String?,     // si es null => mantenimiento global (flush/fsync)
        val flushNow: Boolean,
        val fsyncNow: Boolean
    )

    // Cola con backpressure (descarta el más antiguo si está llena)
    private val queue = LinkedBlockingQueue<Cmd>(2_000)

    // Drop counter: tracks how many entries were lost due to queue overflow
    private val droppedCount = AtomicLong(0L)
    @Volatile private var lastDropWarnMs = 0L

    // Estado de ejecución del worker
    private val running = AtomicBoolean(true)
    private val workerThread: Thread

    // Estado por archivo (solo desde el hilo worker)
    private val writers = HashMap<String, BufferedWriter>()
    private val streams = HashMap<String, FileOutputStream>() // para fd.sync()

    // Mantenimiento periódico
    private var lastFlushMs = System.currentTimeMillis()
    private var lastFsyncMs = System.currentTimeMillis()
    private val FLUSH_EVERY_MS = 3_000L
    private val FSYNC_EVERY_MS = 10_000L

    init {
        workerThread = Thread(::workerLoop, "file-logger").apply {
            isDaemon = true
            start()
        }
    }

    /* ---------------------------------------------------------------------
     *  Worker
     * ------------------------------------------------------------------ */

    /** Abre (o reutiliza) el writer para 'fileName'. Solo desde el hilo worker. */
    private fun openWriter(fileName: String): BufferedWriter {
        writers[fileName]?.let { return it }

        val f = File(baseDir, fileName)
        val fos = FileOutputStream(f, /*append*/ true)
        streams[fileName] = fos

        val osw = OutputStreamWriter(fos, StandardCharsets.UTF_8)
        val bw = BufferedWriter(osw, 16 * 1024)
        writers[fileName] = bw

        return bw
    }

    /** Bucle principal del hilo de trabajo. */
    private fun workerLoop() {
        try {
            while (running.get() || queue.isNotEmpty()) {
                try {
                    val cmd = queue.poll(500, TimeUnit.MILLISECONDS)
                    val now = System.currentTimeMillis()

                    if (cmd != null) {
                        if (cmd.line != null) {
                            // Escritura normal
                            val w = openWriter(cmd.file)
                            w.write(cmd.line)
                            if (cmd.flushNow) {
                                runCatching { w.flush() }
                            }
                            if (cmd.fsyncNow) {
                                streams[cmd.file]?.fd?.let { fd ->
                                    runCatching { fd.sync() }
                                }
                            }
                        } else {
                            // Cmd de mantenimiento explícito (flush+fsync global)
                            writers.values.forEach { runCatching { it.flush() } }
                            streams.values.forEach { s -> runCatching { s.fd.sync() } }
                            lastFlushMs = now
                            lastFsyncMs = now
                        }
                    }

                    // Mantenimiento periódico (aunque no haya comandos)
                    if (now - lastFlushMs >= FLUSH_EVERY_MS) {
                        writers.values.forEach { runCatching { it.flush() } }
                        lastFlushMs = now
                    }
                    if (now - lastFsyncMs >= FSYNC_EVERY_MS) {
                        streams.values.forEach { s -> runCatching { s.fd.sync() } }
                        lastFsyncMs = now
                    }
                } catch (t: Throwable) {
                    Log.e(tag, "workerLoop iteration error", t)
                }
            }
        } finally {
            // Vaciado final y cierre ordenado
            try {
                while (true) {
                    val c = queue.poll() ?: break
                    val line = c.line ?: continue
                    val w = openWriter(c.file)
                    w.write(line)
                }
            } catch (_: Throwable) {
                // Ignorado
            }

            runCatching { writers.values.forEach { it.flush() } }
            runCatching { streams.values.forEach { it.fd.sync() } }
            runCatching { writers.values.forEach { it.close() } }
            runCatching { streams.values.forEach { it.close() } }

            writers.clear()
            streams.clear()
        }
    }

    /* ---------------------------------------------------------------------
     *  API pública
     * ------------------------------------------------------------------ */

    /** Total de entradas descartadas por overflow de cola. */
    fun droppedTotal(): Long = droppedCount.get()

    /** Inserta en cola con backpressure (descarta el más antiguo si está llena). */
    private fun offer(cmd: Cmd) {
        if (!running.get()) return
        if (!queue.offer(cmd)) {
            // Cola llena: descarta el comando más antiguo y reintenta
            queue.poll()
            val totalDropped = droppedCount.incrementAndGet()
            queue.offer(cmd)

            // Warn at most once per second to avoid log spam
            val now = System.currentTimeMillis()
            if (now - lastDropWarnMs > 1_000L) {
                lastDropWarnMs = now
                Log.w(tag, "Queue overflow: $totalDropped entries dropped so far")
            }
        }
    }

    /**
     * Escribe una línea JSON (se añade '\n' si no lo tiene).
     *
     * @param flushNow si true, fuerza flush inmediato tras esta línea (no garantiza fsync).
     */
    fun appendJsonLine(
        fileName: String,
        json: String,
        flushNow: Boolean = false
    ) {
        val line = if (json.endsWith('\n')) json else "$json\n"
        offer(
            Cmd(
                file = fileName,
                line = line,
                flushNow = flushNow,
                fsyncNow = false
            )
        )
    }

    /**
     * Envuelve el cuerpo en:
     *
     *   {"ts":"...","type":"...","data": <jsonBody>}
     *
     * donde 'jsonBody' debe ser JSON válido (se inserta tal cual, sin comillas extra).
     *
     * @param flushNow si true, fuerza flush inmediato tras esta línea (no garantiza fsync).
     */
    fun appendTaggedJson(
        fileName: String,
        type: String,
        jsonBody: String,
        flushNow: Boolean = false
    ) {
        val ts = isoNow()
        val sb = StringBuilder(24 + type.length + jsonBody.length)
        sb.append("{\"ts\":\"")
            .append(ts)
            .append("\",\"type\":\"")
            .append(type)
            .append("\",\"data\":")
            .append(jsonBody)
            .append('}')
        appendJsonLine(fileName, sb.toString(), flushNow)
    }

    /**
     * Fuerza flush + fsync de *todos* los archivos abiertos.
     * Útil al pausar/ocultar UI o antes de cerrar.
     */
    fun flushAll() {
        // Cmd especial (line=null) => mantenimiento global
        offer(
            Cmd(
                file = "__all__",
                line = null,
                flushNow = true,
                fsyncNow = true
            )
        )
    }

    /**
     * Cierra el logger.
     *
     * Llamar en onTrimMemory(UI_HIDDEN) y onDestroy.
     * Bloquea hasta vaciar la cola y cerrar streams (con timeout corto).
     */
    fun close() {
        if (!running.compareAndSet(true, false)) return

        // Desbloquea el poll() si está esperando
        queue.offer(
            Cmd(
                file = "__quit__",
                line = null,
                flushNow = true,
                fsyncNow = true
            )
        )
        try {
            workerThread.join(2_000)
        } catch (_: InterruptedException) {
            // El finally del worker se encargará igualmente
        }
    }

    /* ---------------------------------------------------------------------
     *  Timestamp ISO-8601
     * ------------------------------------------------------------------ */

    /** Timestamp ISO-8601 con zona local (ej: 2025-09-25T14:13:50.959+02:00). */
    companion object {
        private val ISO_FMT =
            SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSSXXX", Locale.US)
    }

    @Synchronized
    private fun isoNow(): String =
        ISO_FMT.format(System.currentTimeMillis())
}
