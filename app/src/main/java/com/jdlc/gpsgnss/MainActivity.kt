package com.jdlc.gpsgnss

import android.Manifest
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.location.GnssMeasurementsEvent
import android.location.GnssStatus
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Bundle
import android.os.SystemClock
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import androidx.core.content.FileProvider
import com.jdlc.gpsgnss.databinding.ActivityMainBinding
import kotlin.math.PI
import kotlin.math.cos

class MainActivity : AppCompatActivity(), SensorEventListener {

    private lateinit var binding: ActivityMainBinding
    private lateinit var locationManager: LocationManager
    private lateinit var sensorManager: SensorManager
    private lateinit var recorder: SessionRecorder

    private var locationRegistered = false
    private var gnssStatusRegistered = false
    private var rawGnssRegistered = false

    private var originLat: Double? = null
    private var originLon: Double? = null

    private var accel = floatArrayOf(Float.NaN, Float.NaN, Float.NaN)
    private var gyro = floatArrayOf(Float.NaN, Float.NaN, Float.NaN)
    private var magnet = floatArrayOf(Float.NaN, Float.NaN, Float.NaN)
    private var headingDeg = Float.NaN

    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestPermission()
    ) { granted ->
        if (granted) registerGnss()
        else binding.statusText.text = "Status: precise location permission denied"
    }

    private val locationListener = LocationListener { location ->
        updateLocationUi(location)
        recorder.recordLocation(location)
    }

    private val gnssStatusCallback = object : GnssStatus.Callback() {
        override fun onSatelliteStatusChanged(status: GnssStatus) {
            updateSatelliteUi(status)
            recorder.recordGnssStatus(status, SystemClock.elapsedRealtimeNanos())
        }
    }

    private val rawCallback = object : GnssMeasurementsEvent.Callback() {
        override fun onGnssMeasurementsReceived(eventArgs: GnssMeasurementsEvent) {
            recorder.recordRaw(eventArgs)
        }

        override fun onStatusChanged(status: Int) {
            val label = when (status) {
                STATUS_READY -> "ready"
                STATUS_LOCATION_DISABLED -> "location disabled"
                STATUS_NOT_ALLOWED -> "not allowed"
                STATUS_NOT_SUPPORTED -> "not supported"
                else -> "status=$status"
            }
            binding.statusText.text = "GNSS raw: $label"
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        recorder = SessionRecorder(this)
        locationManager = getSystemService(Context.LOCATION_SERVICE) as LocationManager
        sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager

        binding.startButton.setOnClickListener { startRecording() }
        binding.stopButton.setOnClickListener { stopRecording() }
        binding.exportButton.setOnClickListener { exportLastSession() }

        registerSensors()
        ensureLocationPermission()
    }

    override fun onDestroy() {
        super.onDestroy()
        recorder.stop()
        sensorManager.unregisterListener(this)
        if (locationRegistered) locationManager.removeUpdates(locationListener)
        if (gnssStatusRegistered) locationManager.unregisterGnssStatusCallback(gnssStatusCallback)
        if (rawGnssRegistered) locationManager.unregisterGnssMeasurementsCallback(rawCallback)
    }

    private fun ensureLocationPermission() {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED) {
            registerGnss()
        } else {
            permissionLauncher.launch(Manifest.permission.ACCESS_FINE_LOCATION)
        }
    }

    private fun registerGnss() {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) return

        if (!locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER)) {
            binding.statusText.text = "Status: enable GNSS/GPS in Android settings"
        }

        if (!locationRegistered) {
            locationManager.requestLocationUpdates(
                LocationManager.GPS_PROVIDER,
                0L,
                0f,
                locationListener,
                mainLooper
            )
            locationRegistered = true
        }

        if (!gnssStatusRegistered) {
            gnssStatusRegistered = locationManager.registerGnssStatusCallback(gnssStatusCallback, android.os.Handler(mainLooper))
        }

        if (!rawGnssRegistered) {
            rawGnssRegistered = locationManager.registerGnssMeasurementsCallback(rawCallback, android.os.Handler(mainLooper))
        }

        binding.statusText.text = "Location active | Raw GNSS callback: ${if (rawGnssRegistered) "registered" else "unavailable"}"
    }

    private fun registerSensors() {
        registerPreferredSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED, Sensor.TYPE_ACCELEROMETER)
        registerPreferredSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED, Sensor.TYPE_GYROSCOPE)
        registerPreferredSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED, Sensor.TYPE_MAGNETIC_FIELD)
        sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)?.let {
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
        }
    }

    private fun registerPreferredSensor(preferredType: Int, fallbackType: Int) {
        val sensor = sensorManager.getDefaultSensor(preferredType) ?: sensorManager.getDefaultSensor(fallbackType)
        sensor?.let { sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) }
    }

    private fun startRecording() {
        originLat = null
        originLon = null
        try {
            val dir = recorder.start()
            binding.startButton.isEnabled = false
            binding.stopButton.isEnabled = true
            binding.sessionText.text = "Recording: ${dir.name}"
        } catch (e: Exception) {
            Toast.makeText(this, "Cannot start recording: ${e.message}", Toast.LENGTH_LONG).show()
        }
    }

    private fun stopRecording() {
        val dir = recorder.currentDirectory()
        recorder.stop()
        binding.startButton.isEnabled = true
        binding.stopButton.isEnabled = false
        binding.sessionText.text = if (dir != null) "Saved: ${dir.absolutePath}" else "No active session"
    }

    private fun exportLastSession() {
        val files = recorder.currentFiles()
        if (files.isEmpty()) {
            Toast.makeText(this, "Record a session first", Toast.LENGTH_SHORT).show()
            return
        }
        val uris = ArrayList(files.map { file ->
            FileProvider.getUriForFile(this, "$packageName.files", file)
        })
        val intent = Intent(Intent.ACTION_SEND_MULTIPLE).apply {
            type = "text/plain"
            putParcelableArrayListExtra(Intent.EXTRA_STREAM, uris)
            addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
        }
        startActivity(Intent.createChooser(intent, "Export GnssLogger dataset"))
    }

    private fun updateLocationUi(location: Location) {
        binding.gnssText.text = buildString {
            appendLine("Lat: %.8f".format(location.latitude))
            appendLine("Lon: %.8f".format(location.longitude))
            appendLine("Accuracy: %.2f m".format(location.accuracy))
            appendLine("Speed: %s".format(if (location.hasSpeed()) "%.2f m/s".format(location.speed) else "--"))
            append("Bearing: %s".format(if (location.hasBearing()) "%.1f°".format(location.bearing) else "--"))
        }

        if (recorder.isRecording && originLat == null) {
            originLat = location.latitude
            originLon = location.longitude
        }

        val lat0 = originLat
        val lon0 = originLon
        if (lat0 != null && lon0 != null) {
            val (x, y) = localXY(lat0, lon0, location.latitude, location.longitude)
            binding.positionText.text = "Local origin: first fix of session\nX East: %.3f m\nY North: %.3f m\nFiltered: pending offline analysis".format(x, y)
        }
    }

    private fun updateSatelliteUi(status: GnssStatus) {
        val counts = mutableMapOf<String, Int>()
        var used = 0
        var l1 = 0
        var l5 = 0
        for (i in 0 until status.satelliteCount) {
            val constellation = SessionRecorder.constellationName(status.getConstellationType(i))
            counts[constellation] = (counts[constellation] ?: 0) + 1
            if (status.usedInFix(i)) used++
            if (status.hasCarrierFrequencyHz(i)) {
                val mhz = status.getCarrierFrequencyHz(i) / 1_000_000.0
                if (mhz in 1550.0..1610.0) l1++
                if (mhz in 1160.0..1220.0) l5++
            }
        }
        binding.satellitesText.text = buildString {
            appendLine("Visible: ${status.satelliteCount} | used: $used")
            appendLine("GPS ${counts["GPS"] ?: 0} | Galileo ${counts["GALILEO"] ?: 0} | BeiDou ${counts["BEIDOU"] ?: 0} | GLONASS ${counts["GLONASS"] ?: 0}")
            append("L1/E1-like: $l1 | L5/E5-like: $l5")
        }
    }

    override fun onSensorChanged(event: SensorEvent) {
        when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER, Sensor.TYPE_ACCELEROMETER_UNCALIBRATED -> accel = event.values.copyOf(3)
            Sensor.TYPE_GYROSCOPE, Sensor.TYPE_GYROSCOPE_UNCALIBRATED -> gyro = event.values.copyOf(3)
            Sensor.TYPE_MAGNETIC_FIELD, Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED -> magnet = event.values.copyOf(3)
            Sensor.TYPE_ROTATION_VECTOR -> {
                val rotation = FloatArray(9)
                val orientation = FloatArray(3)
                SensorManager.getRotationMatrixFromVector(rotation, event.values)
                SensorManager.getOrientation(rotation, orientation)
                headingDeg = ((orientation[0] * 180f / PI.toFloat()) + 360f) % 360f
            }
        }
        recorder.recordSensor(event)
        binding.imuText.text = buildString {
            appendLine("Acceleration: ${vec(accel)} m/s²")
            appendLine("Gyroscope: ${vec(gyro)} rad/s")
            appendLine("Magnetometer: ${vec(magnet)} µT")
            append("Heading: ${if (headingDeg.isNaN()) "--" else "%.1f°".format(headingDeg)}")
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    private fun vec(v: FloatArray): String = "[%.3f, %.3f, %.3f]".format(v[0], v[1], v[2])

    private fun localXY(lat0: Double, lon0: Double, lat: Double, lon: Double): Pair<Double, Double> {
        val earthRadius = 6_378_137.0
        val dLat = Math.toRadians(lat - lat0)
        val dLon = Math.toRadians(lon - lon0)
        val meanLat = Math.toRadians((lat + lat0) / 2.0)
        val xEast = earthRadius * dLon * cos(meanLat)
        val yNorth = earthRadius * dLat
        return xEast to yNorth
    }
}
