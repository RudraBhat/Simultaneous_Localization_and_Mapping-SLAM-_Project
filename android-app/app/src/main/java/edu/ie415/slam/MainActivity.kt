package edu.ie415.slam

import android.Manifest
import android.app.Activity
import android.content.Intent
import android.content.pm.PackageManager
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.opengl.GLSurfaceView
import android.os.Bundle
import android.view.Gravity
import android.widget.Button
import android.widget.LinearLayout
import android.widget.TextView
import android.widget.Toast
import com.google.ar.core.ArCoreApk
import com.google.ar.core.Config
import com.google.ar.core.Session

class MainActivity : Activity(), SensorEventListener {
    private lateinit var glView: GLSurfaceView
    private lateinit var renderer: CameraRenderer
    private lateinit var captureWriter: CaptureWriter
    private lateinit var sensorManager: SensorManager
    private lateinit var statusView: TextView
    private lateinit var recordView: TextView

    @Volatile
    private var arSession: Session? = null
    private var installRequested = false
    private var sensorListening = false
    private var sessionResumed = false
    private var sessionError: String? = null

    companion object {
        private const val CAMERA_PERMISSION_REQUEST = 4101
        private const val EXPORT_REQUEST = 4102
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        sensorManager = getSystemService(SENSOR_SERVICE) as SensorManager
        captureWriter = CaptureWriter(this) { message ->
            runOnUiThread { recordView.text = message }
        }
        buildInterface()
    }

    private fun buildInterface() {
        val root = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setBackgroundColor(0xFF101820.toInt())
        }

        statusView = TextView(this).apply {
            text = "Checking camera access..."
            textSize = 15f
            setTextColor(0xFFFFFFFF.toInt())
            setPadding(18, 12, 18, 12)
        }
        root.addView(statusView)

        glView = GLSurfaceView(this).apply {
            setEGLContextClientVersion(2)
            renderer = CameraRenderer(
                sessionProvider = { arSession },
                frameConsumer = { frame ->
                    captureWriter.onFrame(frame)
                    if (frame.timestamp % 500_000_000L < 20_000_000L) {
                        val state = frame.camera.trackingState.name
                        runOnUiThread {
                            statusView.text = "Tracking: " + state +
                                "  |  camera: " + captureWriter.frameCount +
                                "  |  accel: " + captureWriter.accelerometerSampleCount +
                                "  |  gyro: " + captureWriter.gyroscopeSampleCount
                        }
                    }
                },
                errorConsumer = { message -> runOnUiThread { showError(message) } }
            ).also { renderer = it }
            setRenderer(renderer)
            renderMode = GLSurfaceView.RENDERMODE_CONTINUOUSLY
        }
        root.addView(glView, LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT,
            0,
            1f
        ))

        recordView = TextView(this).apply {
            text = "Ready. Start a run, walk steadily, then stop and export."
            textSize = 14f
            setTextColor(0xFFD9E2EC.toInt())
            setPadding(18, 10, 18, 10)
        }
        root.addView(recordView)

        val buttons = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER
            setPadding(8, 8, 8, 16)
        }
        buttons.addView(makeButton("Start", ::startRecording))
        buttons.addView(makeButton("Stop", ::stopRecording))
        buttons.addView(makeButton("Export ZIP", ::chooseExportLocation))
        root.addView(buttons)
        setContentView(root)
    }

    private fun makeButton(label: String, action: () -> Unit): Button {
        return Button(this).apply {
            text = label
            isAllCaps = false
            setOnClickListener { action() }
            layoutParams = LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f)
                .apply { setMargins(4, 0, 4, 0) }
        }
    }

    override fun onResume() {
        super.onResume()
        glView.onResume()
        if (checkSelfPermission(Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
            if (arSession == null) ensureArSession() else resumeArSession()
        } else {
            requestPermissions(arrayOf(Manifest.permission.CAMERA), CAMERA_PERMISSION_REQUEST)
        }
    }

    override fun onPause() {
        if (captureWriter.isRecording) stopRecording()
        try {
            arSession?.pause()
        } catch (_: Exception) {
        }
        sessionResumed = false
        glView.onPause()
        super.onPause()
    }

    private fun ensureArSession() {
        if (arSession != null) return
        try {
            val installStatus = ArCoreApk.getInstance().requestInstall(this, !installRequested)
            if (installStatus == ArCoreApk.InstallStatus.INSTALL_REQUESTED) {
                installRequested = true
                return
            }
            val newSession = Session(this)
            val config = Config(newSession).apply {
                updateMode = Config.UpdateMode.LATEST_CAMERA_IMAGE
            }
            newSession.configure(config)
            arSession = newSession
            resumeArSession()
            sessionError = null
            statusView.text = "ARCore ready. Camera preview and sensor logger are active."
        } catch (error: Exception) {
            sessionError = error.message ?: error.javaClass.simpleName
            showError("ARCore could not start: " + sessionError)
        }
    }

    private fun resumeArSession() {
        if (sessionResumed) return
        try {
            arSession?.resume()
            sessionResumed = arSession != null
        } catch (error: Exception) {
            sessionError = error.message ?: error.javaClass.simpleName
            showError("ARCore could not resume: " + sessionError)
        }
    }

    private fun startRecording() {
        if (captureWriter.isRecording) {
            showError("This session is already recording. Stop it before starting another.")
            return
        }
        if (arSession == null) {
            ensureArSession()
            if (arSession == null) {
                showError(sessionError ?: "Allow camera access and install Google Play Services for AR.")
                return
            }
        }
        try {
            captureWriter.start()
            val accel = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
            val gyro = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
            if (accel == null || gyro == null) {
                captureWriter.stop()
                showError("This phone needs both an accelerometer and gyroscope.")
                return
            }
            val accelRegistered = sensorManager.registerListener(this, accel, 10_000)
            val gyroRegistered = sensorManager.registerListener(this, gyro, 10_000)
            if (!accelRegistered || !gyroRegistered) {
                sensorManager.unregisterListener(this)
                captureWriter.stop()
                showError("Android could not start both motion sensors for this session.")
                return
            }
            sensorListening = true
            recordView.text = "Recording camera, accelerometer, and gyroscope data."
            statusView.text = "Recording. Keep the phone in one camera mode and move smoothly."
        } catch (error: Exception) {
            showError("Could not start recording: " + error.message)
        }
    }

    private fun stopRecording() {
        if (!captureWriter.isRecording) {
            showError("There is no active recording to stop.")
            return
        }
        if (sensorListening) {
            sensorManager.unregisterListener(this)
            sensorListening = false
        }
        captureWriter.stop()
        recordView.text = captureWriter.latestSessionDescription()
        statusView.text = "Stopped. Export the ZIP and process it on the laptop."
    }

    private fun chooseExportLocation() {
        if (captureWriter.isRecording) {
            showError("Stop the recording before exporting its ZIP.")
            return
        }
        if (captureWriter.latestSessionDir == null) {
            showError("Record and stop a session before exporting.")
            return
        }
        val intent = Intent(Intent.ACTION_CREATE_DOCUMENT).apply {
            addCategory(Intent.CATEGORY_OPENABLE)
            type = "application/zip"
            putExtra(Intent.EXTRA_TITLE, captureWriter.suggestedZipName())
        }
        startActivityForResult(intent, EXPORT_REQUEST)
    }

    @Deprecated("Uses the platform document picker for broad Android compatibility.")
    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        super.onActivityResult(requestCode, resultCode, data)
        if (requestCode == EXPORT_REQUEST && resultCode == RESULT_OK) {
            val uri = data?.data ?: return
            recordView.text = "Creating export..."
            Thread {
                try {
                    captureWriter.exportTo(uri)
                    runOnUiThread { recordView.text = "Export complete. Move the ZIP to the laptop." }
                } catch (error: Exception) {
                    runOnUiThread { showError("Export failed: " + error.message) }
                }
            }.start()
        }
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == CAMERA_PERMISSION_REQUEST) {
            if (grantResults.firstOrNull() == PackageManager.PERMISSION_GRANTED) {
                ensureArSession()
            } else {
                showError("Camera access is required to capture a SLAM dataset.")
            }
        }
    }

    override fun onSensorChanged(event: SensorEvent) {
        if (captureWriter.isRecording) captureWriter.onSensorSample(event)
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    override fun onDestroy() {
        try {
            arSession?.close()
        } catch (_: Exception) {
        }
        arSession = null
        super.onDestroy()
    }

    private fun showError(message: String) {
        statusView.text = message
        Toast.makeText(this, message, Toast.LENGTH_LONG).show()
    }
}
