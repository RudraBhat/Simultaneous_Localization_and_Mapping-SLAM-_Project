package edu.ie415.slam

import android.content.Context
import android.hardware.SensorEvent
import android.graphics.ImageFormat
import android.graphics.Rect
import android.graphics.YuvImage
import android.net.Uri
import android.os.Build
import com.google.ar.core.Frame
import com.google.ar.core.exceptions.NotYetAvailableException
import java.io.ByteArrayOutputStream
import java.io.File
import java.io.FileInputStream
import java.io.FileOutputStream
import java.util.Locale
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import java.util.zip.ZipEntry
import java.util.zip.ZipOutputStream

class CaptureWriter(
    private val context: Context,
    private val statusConsumer: (String) -> Unit
) {
    @Volatile
    var isRecording: Boolean = false
        private set

    @Volatile
    var frameCount: Int = 0
        private set

    @Volatile
    var accelerometerSampleCount: Int = 0
        private set

    @Volatile
    var gyroscopeSampleCount: Int = 0
        private set

    var latestSessionDir: File? = null
        private set

    private var framesWriter: java.io.BufferedWriter? = null
    private var imuWriter: java.io.BufferedWriter? = null
    private var imageExecutor: ExecutorService? = null
    private var lastImageTimestampNs = 0L

    @Synchronized
    @Throws(java.io.IOException::class)
    fun start() {
        check(!isRecording) { "A capture is already running." }
        val runId = java.text.SimpleDateFormat("yyyyMMdd_HHmmss_SSS", Locale.US)
            .format(java.util.Date())
        val runDir = File(File(context.filesDir, "sessions"), "ie415_$runId")
        val imagesDir = File(runDir, "images")
        if (!imagesDir.mkdirs() && !imagesDir.isDirectory) {
            throw java.io.IOException("Could not create capture directory.")
        }

        framesWriter = File(runDir, "frames.csv").bufferedWriter()
        imuWriter = File(runDir, "imu.csv").bufferedWriter()
        framesWriter?.write(
            "timestamp_ns,image_file,tx,ty,tz,qx,qy,qz,qw,tracking_state," +
                "fx,fy,cx,cy,width,height\n"
        )
        imuWriter?.write("timestamp_ns,sensor,x,y,z,accuracy\n")
        imageExecutor = Executors.newSingleThreadExecutor()
        latestSessionDir = runDir
        lastImageTimestampNs = 0L
        frameCount = 0
        accelerometerSampleCount = 0
        gyroscopeSampleCount = 0
        isRecording = true
        statusConsumer("Recording session " + runId)
    }

    fun onFrame(frame: Frame) {
        if (!isRecording) return
        val timestamp = frame.timestamp
        if (lastImageTimestampNs != 0L && timestamp - lastImageTimestampNs < 100_000_000L) return

        val image = try {
            frame.acquireCameraImage()
        } catch (_: NotYetAvailableException) {
            return
        } catch (_: Exception) {
            return
        }

        try {
            val nv21 = imageToNv21(image)
            val width = image.width
            val height = image.height
            val camera = frame.camera
            val pose = camera.pose
            val translation = pose.translation
            val quaternion = pose.rotationQuaternion
            val intrinsics = camera.imageIntrinsics
            val focal = intrinsics.focalLength
            val principal = intrinsics.principalPoint
            val dimensions = intrinsics.imageDimensions
            val index = frameCount + 1
            val filename = "frame_%06d.jpg".format(Locale.US, index)
            val row = listOf(
                timestamp.toString(),
                filename,
                format(translation[0]),
                format(translation[1]),
                format(translation[2]),
                format(quaternion[0]),
                format(quaternion[1]),
                format(quaternion[2]),
                format(quaternion[3]),
                camera.trackingState.name,
                format(focal[0]),
                format(focal[1]),
                format(principal[0]),
                format(principal[1]),
                dimensions[0].toString(),
                dimensions[1].toString()
            ).joinToString(",")

            synchronized(this) {
                if (!isRecording) return
                framesWriter?.write(row)
                framesWriter?.newLine()
                framesWriter?.flush()
                frameCount = index
                lastImageTimestampNs = timestamp
            }

            val runDir = latestSessionDir ?: return
            val target = File(runDir, "images/$filename")
            imageExecutor?.execute { writeJpeg(nv21, width, height, target) }
        } catch (_: Exception) {
            // A malformed or unavailable camera frame is skipped; later frames remain usable.
        } finally {
            image.close()
        }
    }

    @Synchronized
    fun onSensorSample(event: SensorEvent) {
        if (!isRecording) return
        val label = when (event.sensor.type) {
            android.hardware.Sensor.TYPE_ACCELEROMETER -> "accelerometer"
            android.hardware.Sensor.TYPE_GYROSCOPE -> "gyroscope"
            else -> return
        }
        val row = listOf(
            event.timestamp.toString(),
            label,
            format(event.values.getOrElse(0) { 0f }),
            format(event.values.getOrElse(1) { 0f }),
            format(event.values.getOrElse(2) { 0f }),
            event.accuracy.toString()
        ).joinToString(",")
        imuWriter?.write(row)
        imuWriter?.newLine()
        if (label == "accelerometer") {
            accelerometerSampleCount += 1
        } else {
            gyroscopeSampleCount += 1
        }
    }

    @Synchronized
    fun stop() {
        if (!isRecording) return
        isRecording = false
        framesWriter?.flush()
        imuWriter?.flush()
        framesWriter?.close()
        imuWriter?.close()
        framesWriter = null
        imuWriter = null

        imageExecutor?.shutdown()
        imageExecutor?.awaitTermination(30, TimeUnit.SECONDS)
        imageExecutor = null
        latestSessionDir?.let { writeManifest(it) }
        statusConsumer(latestSessionDescription())
    }

    fun latestSessionDescription(): String {
        val folder = latestSessionDir?.name ?: "none"
        return "Session " + folder + ": " + frameCount + " camera frames, " +
            accelerometerSampleCount + " accelerometer samples, " +
            gyroscopeSampleCount + " gyroscope samples. Export its ZIP to the laptop."
    }

    fun suggestedZipName(): String {
        return (latestSessionDir?.name ?: "ie415_session") + ".zip"
    }

    @Throws(Exception::class)
    fun exportTo(uri: Uri) {
        val source = latestSessionDir ?: throw IllegalStateException("No completed session is available.")
        check(!isRecording) { "Stop recording before exporting." }
        val temporaryZip = File(context.cacheDir, suggestedZipName())
        ZipOutputStream(FileOutputStream(temporaryZip)).use { zip ->
            val root = source.parentFile ?: source
            source.walkTopDown().filter { it.isFile }.forEach { file ->
                val relative = file.relativeTo(root).invariantSeparatorsPath
                zip.putNextEntry(ZipEntry(relative))
                FileInputStream(file).use { it.copyTo(zip) }
                zip.closeEntry()
            }
        }
        context.contentResolver.openOutputStream(uri)?.use { output ->
            FileInputStream(temporaryZip).use { it.copyTo(output) }
        } ?: throw java.io.IOException("The selected export location could not be opened.")
        temporaryZip.delete()
    }

    private fun imageToNv21(image: android.media.Image): ByteArray {
        val width = image.width
        val height = image.height
        val ySize = width * height
        val out = ByteArray(ySize + width * height / 2)
        copyPlane(image.planes[0], width, height, out, 0, 1)

        val uPlane = image.planes[1]
        val vPlane = image.planes[2]
        val u = uPlane.buffer.duplicate()
        val v = vPlane.buffer.duplicate()
        val uStart = u.position()
        val vStart = v.position()
        val chromaWidth = width / 2
        val chromaHeight = height / 2
        var outputIndex = ySize
        for (row in 0 until chromaHeight) {
            for (col in 0 until chromaWidth) {
                val vIndex = vStart + row * vPlane.rowStride + col * vPlane.pixelStride
                val uIndex = uStart + row * uPlane.rowStride + col * uPlane.pixelStride
                out[outputIndex++] = v.get(vIndex)
                out[outputIndex++] = u.get(uIndex)
            }
        }
        return out
    }

    private fun copyPlane(
        plane: android.media.Image.Plane,
        width: Int,
        height: Int,
        output: ByteArray,
        outputOffset: Int,
        outputPixelStride: Int
    ) {
        val buffer = plane.buffer.duplicate()
        val start = buffer.position()
        var outputIndex = outputOffset
        for (row in 0 until height) {
            val rowStart = start + row * plane.rowStride
            for (col in 0 until width) {
                output[outputIndex] = buffer.get(rowStart + col * plane.pixelStride)
                outputIndex += outputPixelStride
            }
        }
    }

    private fun writeJpeg(nv21: ByteArray, width: Int, height: Int, target: File) {
        try {
            ByteArrayOutputStream().use { bytes ->
                val image = YuvImage(nv21, ImageFormat.NV21, width, height, null)
                if (image.compressToJpeg(Rect(0, 0, width, height), 84, bytes)) {
                    target.outputStream().use { output -> bytes.writeTo(output) }
                }
            }
        } catch (_: Exception) {
            target.delete()
        }
    }

    private fun writeManifest(directory: File) {
        val model = jsonEscape(Build.MODEL ?: "unknown")
        val manufacturer = jsonEscape(Build.MANUFACTURER ?: "unknown")
        val release = jsonEscape(Build.VERSION.RELEASE ?: "unknown")
        val text = """
            {
              "format_version": 1,
              "device": "$manufacturer $model",
              "android_release": "$release",
              "camera_stream": "ARCore CPU camera image; JPEG frames sampled at approximately 10 Hz",
              "camera_pose": "ARCore physical camera pose in the session world frame",
              "frame_time": "ARCore Frame.timestamp in nanoseconds",
              "imu_time": "SensorEvent.timestamp in nanoseconds",
              "imu_axes": "Android sensor coordinate system; accelerometer in m/s^2; gyroscope in rad/s",
              "images": "images/frame_XXXXXX.jpg",
              "calibration_note": "frames.csv includes ARCore image intrinsics. Supply a separately measured distortion model in laptop/camera_calibration.json when available."
            }
        """.trimIndent()
        File(directory, "manifest.json").writeText(text)
    }

    private fun format(value: Float): String = String.format(Locale.US, "%.9g", value)

    private fun jsonEscape(value: String): String {
        return value.replace("\\", "\\\\").replace("\"", "\\\"")
    }
}
