package com.jdlc.gpsgnss

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorManager
import android.location.GnssClock
import android.location.GnssMeasurementsEvent
import android.location.GnssStatus
import android.location.Location
import android.os.SystemClock
import java.io.BufferedWriter
import java.io.File
import java.io.FileWriter
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.TimeZone

/**
 * Writes a subset of the Google GnssLogger text format.
 *
 * Field order follows google/gps-measurement-tools LOGGING_FORMAT.md so logs can
 * be consumed by existing GNSS tooling instead of creating a project-specific
 * raw-measurement format. Unsupported Android observables are emitted empty.
 */
class SessionRecorder(private val context: Context) {

    private var activeSessionDir: File? = null
    private var lastSessionDir: File? = null
    private var logWriter: BufferedWriter? = null
    private var lastFixUnixMs: Long? = null
    private var lastFixElapsedNs: Long? = null

    val isRecording: Boolean get() = activeSessionDir != null

    fun start(): File {
        stop()
        val root = File(context.getExternalFilesDir(null), "sessions").apply { mkdirs() }
        val timestamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).apply {
            timeZone = TimeZone.getTimeZone("UTC")
        }.format(Date())
        val dir = File(root, "session_$timestamp")
        check(dir.mkdirs()) { "Cannot create session directory" }
        activeSessionDir = dir
        lastSessionDir = dir
        logWriter = BufferedWriter(FileWriter(File(dir, "gnss_log.txt"), false), 128 * 1024).also {
            writeHeader(it)
        }
        lastFixUnixMs = null
        lastFixElapsedNs = null
        return dir
    }

    fun stop() {
        try {
            logWriter?.flush()
            logWriter?.close()
        } catch (_: Exception) {
        }
        logWriter = null
        activeSessionDir = null
    }

    fun currentFiles(): List<File> = lastSessionDir?.listFiles()?.filter { it.isFile }?.sortedBy { it.name }.orEmpty()
    fun currentDirectory(): File? = activeSessionDir

    fun recordLocation(location: Location) {
        val writer = logWriter ?: return
        lastFixUnixMs = location.time
        lastFixElapsedNs = location.elapsedRealtimeNanos
        row(
            writer,
            "Fix",
            location.provider ?: "",
            location.latitude,
            location.longitude,
            if (location.hasAltitude()) location.altitude else "",
            if (location.hasSpeed()) location.speed else "",
            location.accuracy,
            if (location.hasBearing()) location.bearing else "",
            location.time,
            if (location.hasSpeedAccuracy()) location.speedAccuracyMetersPerSecond else "",
            if (location.hasBearingAccuracy()) location.bearingAccuracyDegrees else "",
            location.elapsedRealtimeNanos,
            if (location.hasVerticalAccuracy()) location.verticalAccuracyMeters else "",
            if (location.isMock) 1 else 0
        )
    }

    fun recordRaw(event: GnssMeasurementsEvent) {
        val writer = logWriter ?: return
        val c = event.clock
        val utcMs = gnssUtcMillis(c)
        event.measurements.forEach { m ->
            row(
                writer,
                "Raw", utcMs, c.timeNanos,
                if (c.hasLeapSecond()) c.leapSecond else "",
                if (c.hasTimeUncertaintyNanos()) c.timeUncertaintyNanos else "",
                if (c.hasFullBiasNanos()) c.fullBiasNanos else "",
                if (c.hasBiasNanos()) c.biasNanos else "",
                if (c.hasBiasUncertaintyNanos()) c.biasUncertaintyNanos else "",
                if (c.hasDriftNanosPerSecond()) c.driftNanosPerSecond else "",
                if (c.hasDriftUncertaintyNanosPerSecond()) c.driftUncertaintyNanosPerSecond else "",
                c.hardwareClockDiscontinuityCount,
                m.svid, m.timeOffsetNanos, m.state, m.receivedSvTimeNanos,
                m.receivedSvTimeUncertaintyNanos, m.cn0DbHz,
                m.pseudorangeRateMetersPerSecond, m.pseudorangeRateUncertaintyMetersPerSecond,
                m.accumulatedDeltaRangeState, m.accumulatedDeltaRangeMeters,
                m.accumulatedDeltaRangeUncertaintyMeters,
                if (m.hasCarrierFrequencyHz()) m.carrierFrequencyHz else "",
                if (m.hasCarrierCycles()) m.carrierCycles else "",
                if (m.hasCarrierPhase()) m.carrierPhase else "",
                if (m.hasCarrierPhaseUncertainty()) m.carrierPhaseUncertainty else "",
                m.multipathIndicator,
                if (m.hasSnrInDb()) m.snrInDb else "",
                m.constellationType,
                if (m.hasAutomaticGainControlLevelDb()) m.automaticGainControlLevelDb else "",
                if (m.hasBasebandCn0DbHz()) m.basebandCn0DbHz else "",
                if (m.hasFullInterSignalBiasNanos()) m.fullInterSignalBiasNanos else "",
                if (m.hasFullInterSignalBiasUncertaintyNanos()) m.fullInterSignalBiasUncertaintyNanos else "",
                if (m.hasSatelliteInterSignalBiasNanos()) m.satelliteInterSignalBiasNanos else "",
                if (m.hasSatelliteInterSignalBiasUncertaintyNanos()) m.satelliteInterSignalBiasUncertaintyNanos else "",
                if (m.hasCodeType()) m.codeType else "",
                if (c.hasElapsedRealtimeNanos()) c.elapsedRealtimeNanos else ""
            )
        }
    }

    fun recordGnssStatus(status: GnssStatus, elapsedRealtimeNs: Long) {
        val writer = logWriter ?: return
        val fixUtc = if (lastFixElapsedNs != null && elapsedRealtimeNs - lastFixElapsedNs!! <= 2_000_000_000L) lastFixUnixMs ?: "" else ""
        for (i in 0 until status.satelliteCount) {
            row(
                writer,
                "Status", fixUtc, status.satelliteCount, i,
                status.getConstellationType(i), status.getSvid(i),
                if (status.hasCarrierFrequencyHz(i)) status.getCarrierFrequencyHz(i) else "",
                status.getCn0DbHz(i), status.getAzimuthDegrees(i), status.getElevationDegrees(i),
                if (status.usedInFix(i)) 1 else 0,
                if (status.hasAlmanacData(i)) 1 else 0,
                if (status.hasEphemerisData(i)) 1 else 0,
                if (status.hasBasebandCn0DbHz(i)) status.getBasebandCn0DbHz(i) else ""
            )
        }
    }

    fun recordSensor(event: SensorEvent) {
        val writer = logWriter ?: return
        val utcMs = bootUtcMillis() + event.timestamp / 1_000_000L
        val v = event.values
        when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER_UNCALIBRATED -> row(writer, "UncalAccel", utcMs, event.timestamp, *values(v, 6))
            Sensor.TYPE_ACCELEROMETER -> row(writer, "Accel", utcMs, event.timestamp, *values(v, 3))
            Sensor.TYPE_GYROSCOPE_UNCALIBRATED -> row(writer, "UncalGyro", utcMs, event.timestamp, *values(v, 6))
            Sensor.TYPE_GYROSCOPE -> row(writer, "Gyro", utcMs, event.timestamp, *values(v, 3))
            Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED -> row(writer, "UncalMag", utcMs, event.timestamp, *values(v, 6))
            Sensor.TYPE_MAGNETIC_FIELD -> row(writer, "Mag", utcMs, event.timestamp, *values(v, 3))
            Sensor.TYPE_ROTATION_VECTOR -> {
                val rotation = FloatArray(9)
                val orientation = FloatArray(3)
                SensorManager.getRotationMatrixFromVector(rotation, v)
                SensorManager.getOrientation(rotation, orientation)
                val yaw = Math.toDegrees(orientation[0].toDouble())
                val pitch = Math.toDegrees(orientation[1].toDouble())
                val roll = Math.toDegrees(orientation[2].toDouble())
                row(writer, "OrientationDeg", utcMs, event.timestamp, yaw, roll, pitch)
            }
        }
    }

    private fun gnssUtcMillis(c: GnssClock): Any {
        if (!c.hasFullBiasNanos()) return ""
        val bias = if (c.hasBiasNanos()) c.biasNanos else 0.0
        val gpsNanos = c.timeNanos - c.fullBiasNanos - bias
        val leapSeconds = if (c.hasLeapSecond()) c.leapSecond else 0
        return GPS_EPOCH_UNIX_MS + (gpsNanos / 1_000_000.0).toLong() - leapSeconds * 1000L
    }

    private fun bootUtcMillis(): Long = System.currentTimeMillis() - SystemClock.elapsedRealtime()

    private fun values(v: FloatArray, count: Int): Array<Any> = Array(count) { i -> v.getOrNull(i) ?: "" }

    private fun row(writer: BufferedWriter, vararg fields: Any) {
        writer.appendLine(fields.joinToString(",") { field -> csv(field.toString()) })
    }

    private fun csv(value: String): String = if (value.contains(',') || value.contains('"')) {
        "\"${value.replace("\"", "\"\"")}\""
    } else value

    private fun writeHeader(writer: BufferedWriter) {
        writer.appendLine("# GPS_GNSS POC - GnssLogger compatible subset")
        writer.appendLine("# Raw,utcTimeMillis,TimeNanos,LeapSecond,TimeUncertaintyNanos,FullBiasNanos,BiasNanos,BiasUncertaintyNanos,DriftNanosPerSecond,DriftUncertaintyNanosPerSecond,HardwareClockDiscontinuityCount,Svid,TimeOffsetNanos,State,ReceivedSvTimeNanos,ReceivedSvTimeUncertaintyNanos,Cn0DbHz,PseudorangeRateMetersPerSecond,PseudorangeRateUncertaintyMetersPerSecond,AccumulatedDeltaRangeState,AccumulatedDeltaRangeMeters,AccumulatedDeltaRangeUncertaintyMeters,CarrierFrequencyHz,CarrierCycles,CarrierPhase,CarrierPhaseUncertainty,MultipathIndicator,SnrInDb,ConstellationType,AgcDb,BasebandCn0DbHz,FullInterSignalBiasNanos,FullInterSignalBiasUncertaintyNanos,SatelliteInterSignalBiasNanos,SatelliteInterSignalBiasUncertaintyNanos,CodeType,ChipsetElapsedRealtimeNanos")
        writer.appendLine("# UncalAccel,utcTimeMillis,elapsedRealtimeNanos,UncalAccelXMps2,UncalAccelYMps2,UncalAccelZMps2,BiasXMps2,BiasYMps2,BiasZMps2")
        writer.appendLine("# Accel,utcTimeMillis,elapsedRealtimeNanos,AccelXMps2,AccelYMps2,AccelZMps2")
        writer.appendLine("# UncalGyro,utcTimeMillis,elapsedRealtimeNanos,UncalGyroXRadPerSec,UncalGyroYRadPerSec,UncalGyroZRadPerSec,DriftXRadPerSec,DriftYRadPerSec,DriftZRadPerSec")
        writer.appendLine("# Gyro,utcTimeMillis,elapsedRealtimeNanos,GyroXRadPerSec,GyroYRadPerSec,GyroZRadPerSec")
        writer.appendLine("# UncalMag,utcTimeMillis,elapsedRealtimeNanos,UncalMagXMicroT,UncalMagYMicroT,UncalMagZMicroT,BiasXMicroT,BiasYMicroT,BiasZMicroT")
        writer.appendLine("# Mag,utcTimeMillis,elapsedRealtimeNanos,MagXMicroT,MagYMicroT,MagZMicroT")
        writer.appendLine("# OrientationDeg,utcTimeMillis,elapsedRealtimeNanos,yawDeg,rollDeg,pitchDeg")
        writer.appendLine("# Fix,Provider,LatitudeDegrees,LongitudeDegrees,AltitudeMeters,SpeedMps,AccuracyMeters,BearingDegrees,UnixTimeMillis,SpeedAccuracyMps,BearingAccuracyDegrees,elapsedRealtimeNanos,VerticalAccuracyMeters,MockLocation")
        writer.appendLine("# Status,UnixTimeMillis,SignalCount,SignalIndex,ConstellationType,Svid,CarrierFrequencyHz,Cn0DbHz,AzimuthDegrees,ElevationDegrees,UsedInFix,HasAlmanacData,HasEphemerisData,BasebandCn0DbHz")
    }

    companion object {
        private const val GPS_EPOCH_UNIX_MS = 315_964_800_000L

        fun constellationName(type: Int): String = when (type) {
            GnssStatus.CONSTELLATION_GPS -> "GPS"
            GnssStatus.CONSTELLATION_GLONASS -> "GLONASS"
            GnssStatus.CONSTELLATION_GALILEO -> "GALILEO"
            GnssStatus.CONSTELLATION_BEIDOU -> "BEIDOU"
            GnssStatus.CONSTELLATION_QZSS -> "QZSS"
            GnssStatus.CONSTELLATION_SBAS -> "SBAS"
            GnssStatus.CONSTELLATION_IRNSS -> "IRNSS"
            else -> "UNKNOWN"
        }
    }
}
