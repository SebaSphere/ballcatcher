package dev.sebastianb.ballcatcher.app.camera

import dev.sebastianb.ballcatcher.app.ship.YawJointController
import org.opencv.core.Mat
import org.opencv.core.MatOfDouble
import org.opencv.core.MatOfInt
import org.opencv.core.MatOfRect
import org.opencv.core.Point
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc
import org.opencv.objdetect.CascadeClassifier
import java.io.File

/**
 * Tracks the largest, most-confident detected face using both stereo cameras. See [StereoPanTracker]
 * for the shared control loop.
 *
 * @param minDetectionConfidence Minimum Haar cascade level-weight (roughly a log-likelihood; higher is
 *                               stronger) for a detection to be trusted. Marginal detections below this
 *                               are treated as no detection at all, rather than feeding a shaky low-
 *                               confidence position into the control loop. Defaults to 0.0 (effectively
 *                               disabled) because the real range of weights this cascade reports varies
 *                               by camera/lighting and can't be guessed — watch the console for
 *                               "rejected" log lines after raising this above 0 to see what values your
 *                               setup actually produces, then tune from there.
 */
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
    reacquireDeg: Double = 5.0,
    private val minDetectionConfidence: Double = 0.0,
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
    reacquireDeg = reacquireDeg,
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
        val rejectLevels = MatOfInt()
        val levelWeights = MatOfDouble()
        cascade.detectMultiScale3(gray, faces, rejectLevels, levelWeights, 1.1, 3, 0, minFaceSize, Size(), true)

        val faceArr = faces.toArray()
        val weights = levelWeights.toArray()

        // Only trust detections at or above minDetectionConfidence — a shaky, marginal detection is
        // treated as no detection at all (feeding into StereoPanTracker's miss-tolerance grace period)
        // rather than jittering the head toward a low-confidence guess.
        val bestIndex = faceArr.indices
            .filter { weights.getOrElse(it) { 0.0 } >= minDetectionConfidence }
            .maxByOrNull { faceArr[it].width.toLong() * faceArr[it].height }

        if (bestIndex == null) {
            if (faceArr.isNotEmpty()) {
                println("FaceTracker: rejected ${faceArr.size} detection(s) below minDetectionConfidence=$minDetectionConfidence — weights=${weights.toList()}")
            }
            return null
        }

        val best = faceArr[bestIndex]
        return Point(best.x + best.width / 2.0, best.y + best.height / 2.0)
    }
}
