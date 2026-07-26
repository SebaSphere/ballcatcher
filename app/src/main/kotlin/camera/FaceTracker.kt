package dev.sebastianb.ballcatcher.app.camera

import dev.sebastianb.ballcatcher.app.ship.YawJointController
import org.opencv.core.Mat
import org.opencv.core.MatOfRect
import org.opencv.core.Point
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc
import org.opencv.objdetect.CascadeClassifier
import java.io.File

/** Tracks the largest detected face using both stereo cameras. See [StereoPanTracker] for the shared control loop. */
class FaceTracker(
    yawController: YawJointController,
    rightCameraId: Int = 0,
    leftCameraId: Int = 2,
    cameraVFov: Double = 67.0,
    baselineMeters: Double = 0.12,
    deadbandDeg: Double = 2.0,
    updateIntervalMs: Long = 100L,
    smoothingAlpha: Double = 0.5,
    flipPanDirection: Boolean = false,
    boundMarginDeg: Double = 5.0,
    trackingMaxFreq: Int = 100,
    maxStepDeg: Double = 8.0,
    missToleranceTicks: Int = 5,
) : StereoPanTracker(
    yawController = yawController,
    rightCameraId = rightCameraId,
    leftCameraId = leftCameraId,
    cameraVFov = cameraVFov,
    baselineMeters = baselineMeters,
    deadbandDeg = deadbandDeg,
    updateIntervalMs = updateIntervalMs,
    smoothingAlpha = smoothingAlpha,
    flipPanDirection = flipPanDirection,
    boundMarginDeg = boundMarginDeg,
    trackingMaxFreq = trackingMaxFreq,
    maxStepDeg = maxStepDeg,
    missToleranceTicks = missToleranceTicks,
    logTag = "FaceTracker",
) {
    companion object {
        // CascadeClassifier.load() requires a real filesystem path, so the bundled classpath
        // resource is extracted to a temp file once per JVM.
        private val cascade: CascadeClassifier by lazy {
            val resource = FaceTracker::class.java.getResourceAsStream("/cascades/haarcascade_frontalface_default.xml")
                ?: throw IllegalStateException("haarcascade_frontalface_default.xml not found on classpath")
            val cascadeFile = File.createTempFile("haarcascade_frontalface_default", ".xml")
            cascadeFile.deleteOnExit()
            resource.use { input -> cascadeFile.outputStream().use { output -> input.copyTo(output) } }
            CascadeClassifier(cascadeFile.absolutePath)
        }
    }

    private val gray = Mat()

    override fun detectCentroid(frame: Mat): Point? {
        Imgproc.cvtColor(frame, gray, Imgproc.COLOR_BGR2GRAY)
        Imgproc.equalizeHist(gray, gray)

        // A minimum size (as a fraction of frame width) rejects small false-positive detections,
        // which otherwise cut in and out frame-to-frame and make the tracker flicker between targets.
        val minFaceSize = (frame.cols() * 0.15).let { Size(it, it) }

        val faces = MatOfRect()
        cascade.detectMultiScale(gray, faces, 1.1, 3, 0, minFaceSize)

        val largest = faces.toArray().maxByOrNull { it.width.toLong() * it.height } ?: return null
        return Point(largest.x + largest.width / 2.0, largest.y + largest.height / 2.0)
    }
}
