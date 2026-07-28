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
 * Face position is averaged over a trailing 1.5 s window (see `bearingWindowMs`) so the head settles
 * precisely on a stationary person instead of chasing per-frame Haar box jitter. That averaging happens
 * on the face's absolute bearing in [StereoPanTracker], not on raw pixel coordinates — see the comment
 * there for why the distinction matters while the head is panning.
 *
 * Three defaults here are deliberately faster than [StereoPanTracker]'s, and together they set how
 * quickly the head reacts. They trade settle precision for responsiveness, so treat them as a set:
 *
 *  - `bearingWindowMs` (1.5 s) — a trailing average lags a *moving* face by roughly half its window,
 *    so this is the single largest source of reaction delay. Shrink it for snappier tracking, grow it
 *    if the head visibly hunts around a stationary person.
 *  - `proportionalGain` (0.75) — each tick commands this fraction of the remaining error, so the error
 *    decays by `(1 - gain)` per tick: 0.75 converges in half the ticks 0.5 would take. Keep it below
 *    1.0 — at 1.0 the loop commands the entire measured error every tick with no damping for
 *    capture/actuation latency, which invites overshoot and hunting.
 *  - `trackingMaxFreq` (200 Hz) — the pulse-rate ceiling, and the real limit on physical slew speed.
 *    At 0.3°/step (1200 steps/rev) this is 60°/s, versus 30°/s at 100 Hz. Homing already drives the
 *    motor continuously at 200 Hz, so this rate is within what the stepper handles without skipping.
 *
 * `maxStepDeg` is intentionally *not* raised alongside these: at 200 Hz the motor covers at most ~6°
 * in one 100 ms tick, so the existing 8° cap already sits above what the hardware can execute per tick
 * and no longer binds. It stays as a backstop against a single wild detection.
 *
 * @param minDetectionConfidence Minimum Haar cascade level-weight (roughly a log-likelihood; higher is
 *                               stronger) for a detection to be trusted. Marginal detections below this
 *                               are treated as no detection at all, rather than feeding a shaky low-
 *                               confidence position into the control loop. The weights this cascade
 *                               reports vary by camera and lighting, so this is tuned by observation:
 *                               watch the console for "rejected ... weights=[...]" lines to see what
 *                               your setup actually produces, then raise or lower it from there.
 */
class FaceTracker(
    yawController: YawJointController,
    rightCameraId: Int = 0,
    leftCameraId: Int = 2,
    cameraVFov: Double = 67.0,
    baselineMeters: Double = 0.12,
    deadbandDeg: Double = 1.0,
    updateIntervalMs: Long = 100L,
    smoothingAlpha: Double = 1.0,
    flipPanDirection: Boolean = false,
    boundMarginDeg: Double = 5.0,
    trackingMaxFreq: Int = 200,
    maxStepDeg: Double = 8.0,
    missToleranceTicks: Int = 5,
    reacquireDeg: Double = 1.5,
    proportionalGain: Double = 0.75,
    bearingWindowMs: Long = 1500L,
    private val minDetectionConfidence: Double = -1.0,
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
    proportionalGain = proportionalGain,
    bearingWindowMs = bearingWindowMs,
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
        if (faceArr.isEmpty()) return null

        // With zero detections, OpenCV hands back a degenerate 0x1 levelWeights Mat that fails this
        // binding's checkVector() and makes MatOfDouble.toArray() throw — only touch it once we know
        // there's at least one detection to have produced a real vector.
        val weights = levelWeights.toArray()

        // Only trust detections at or above minDetectionConfidence — a shaky, marginal detection is
        // treated as no detection at all (feeding into StereoPanTracker's miss-tolerance grace period)
        // rather than jittering the head toward a low-confidence guess.
        val bestIndex = faceArr.indices
            .filter { weights.getOrElse(it) { 0.0 } >= minDetectionConfidence }
            .maxByOrNull { faceArr[it].width.toLong() * faceArr[it].height }

        if (bestIndex == null) {
            println("FaceTracker: rejected ${faceArr.size} detection(s) below minDetectionConfidence=$minDetectionConfidence — weights=${weights.toList()}")
            return null
        }

        val best = faceArr[bestIndex]
        return Point(best.x + best.width / 2.0, best.y + best.height / 2.0)
    }
}
