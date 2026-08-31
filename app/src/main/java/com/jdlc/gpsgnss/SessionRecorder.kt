package com.jdlc.gpsgnss

import android.content.Context
import android.hardware.SensorEvent
import android.location.GnssClock
import android.location.GnssMeasurement
import android.location.GnssMeasurementsEvent
import android.location.GnssStatus
import android.location.Location
import java.io.BufferedWriter
import java.io.File
import java.io.FileWriter
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.TimeZone

class SessionRecorder(private val context: Context) {

    private var sessionDir: File? = null
    private var locationWriter: BufferedWriter? = null
    private var rawWriter: BufferedWriter? = null
    private var imuWriter: BufferedWriter? = null
    private var statusWriter: BufferedWriter? = null

    val isRecording: Boolean
        get() = sessionDir != null

    fun start(): File {
        stop()
        val root = File(context.getExternalFilesDir(null), "sessions")
        root.mkdirs()
        val timestamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).apply {
            timeZone = TimeZone.getTimeZone("UTC")
        }.format(Date())
        val dir = File(root, "session_$timestamp")
        check(dir.mkdirs()) { "Cannot create session directory" }
        sessionDir = dir

        locationWriter = writer(dir, "location.csv").also {
            it.write("utc_time_ms,elapsed_realtime_ns,latitude_deg,longitude_deg,altitude_m,horizontal_accuracy_m,vertical_accuracy_m,speed_mps,speed_accuracy_mps,bearing_deg,bearing_accuracy_deg,provider\n")
        }
        rawWriter = writer(dir, "gnss_raw.csv").also {
            it.write("event_elapsed_realtime_ns,clock_time_ns,full_bias_ns,bias_ns,drift_nsps,svid,constellation,time_offset_ns,state,received_sv_time_ns,received_sv_time_uncertainty_ns,cn0_dbhz,pseudorange_rate_mps,pseudorange_rate_uncertainty_mps,adr_state,adr_m,adr_uncertainty_m,carrier_frequency_hz\n")
        }
        imuWriter = writer(dir, "imu.csv").also {
            it.write("sensor_timestamp_ns,sensor_type,x,y,z,w,accuracy\n")
        }
        statusWriter = writer(dir, "gnss_status.csv").also {
            it.write("elapsed_realtime_ns,svid,constellation,used_in_fix,cn0_dbhz,carrier_frequency_hz,azimuth_deg,elevation_deg\n")
        }
        return dir
    }

    fun stop() {
        listOf(locationWriter, rawWriter, imuWriter, statusWriter).forEach { writer ->
            try {
                writer?.flush()
                writer?.close()
            } catch (_: Exception) {
            }
        }
        locationWriter = null
        rawWriter = null
        imuWriter = null
        statusWriter = null
        sessionDir = null
    }

    fun currentFiles(): List<File> = sessionDir?.listFiles()?.filter { it.isFile }?.sortedBy { it.name }.orEmpty()

    fun currentDirectory(): File? = sessionDir

    fun recordLocation(location: Location) {
        val writer = locationWriter ?: return
        val verticalAccuracy = if (location.hasVerticalAccuracy()) location.verticalAccuracyMeters else Double.NaN
        val speedAccuracy = if (location.hasSpeedAccuracy()) location.speedAccuracyMetersPerSecond else Double.NaN
        val bearingAccuracy = if (location.hasBearingAccuracy()) location.bearingAccuracyDegrees else Double.NaN
        writer.appendLine(
            listOf(
                location.time,
                location.elapsedRealtimeNanos,
                location.latitude,
                location.longitude,
                if (location.hasAltitude()) location.altitude else Double.NaN,
                location.accuracy,
                verticalAccuracy,
                if (location.hasSpeed()) location.speed else Double.NaN,
                speedAccuracy,
                if (location.hasBearing()) location.bearing else Double.NaN,
                bearingAccuracy,
                csv(location.provider ?: "")
            ).joinToString(",")
        )
    }

    fun recordRaw(event: GnssMeasurementsEvent) {
        val writer = rawWriter ?: return
        val clock = event.clock
        val eventElapsedRealtime = if (clock.hasElapsedRealtimeNanos()) clock.elapsedRealtimeNanos else -1L
        event.measurements.forEach { measurement ->
            writer.appendLine(rawRow(eventElapsedRealtime, clock, measurement))
        }
    }

    private fun rawRow(eventElapsedRealtime: Long, clock: GnssClock, m: GnssMeasurement): String {
        return listOf(
            eventElapsedRealtime,
            clock.timeNanos,
            if (clock.hasFullBiasNanos()) clock.fullBiasNanos else "",
            if (clock.hasBiasNanos()) clock.biasNanos else "",
            if (clock.hasDriftNanosPerSecond()) clock.driftNanosPerSecond else "",
            m.svid,
            constellationName(m.constellationType),
            m.timeOffsetNanos,
            m.state,
            m.receivedSvTimeNanos,
            m.receivedSvTimeUncertaintyNanos,
            m.cn0DbHz,
            m.pseudorangeRateMetersPerSecond,
            m.pseudorangeRateUncertaintyMetersPerSecond,
            m.accumulatedDeltaRangeState,
            m.accumulatedDeltaRangeMeters,
            m.accumulatedDeltaRangeUncertaintyMeters,
            if (m.hasCarrierFrequencyHz()) m.carrierFrequencyHz else ""
        ).joinToString(",")
    }

    fun recordSensor(event: SensorEvent) {
        val writer = imuWriter ?: return
        val v = event.values
        writer.appendLine(
            listOf(
                event.timestamp,
                event.sensor.type,
                v.getOrNull(0) ?: "",
                v.getOrNull(1) ?: "",
                v.getOrNull(2) ?: "",
                v.getOrNull(3) ?: "",
                event.accuracy
            ).joinToString(",")
        )
    }

    fun recordGnssStatus(status: GnssStatus, elapsedRealtimeNs: Long) {
        val writer = statusWriter ?: return
        for (i in 0 until status.satelliteCount) {
            writer.appendLine(
                listOf(
                    elapsedRealtimeNs,
                    status.getSvid(i),
                    constellationName(status.getConstellationType(i)),
                    status.usedInFix(i),
                    status.getCn0DbHz(i),
                    if (status.hasCarrierFrequencyHz(i)) status.getCarrierFrequencyHz(i) else "",
                    status.getAzimuthDegrees(i),
                    status.getElevationDegrees(i)
                ).joinToString(",")
            )
        }
    }

    private fun writer(dir: File, name: String) = BufferedWriter(FileWriter(File(dir, name), false), 64 * 1024)

    private fun csv(value: String): String = "\"${value.replace("\"", "\"\"")}\""

    companion object {
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
