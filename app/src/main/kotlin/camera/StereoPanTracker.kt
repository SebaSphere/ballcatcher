package dev.sebastianb.ballcatcher.app.camera

import dev.sebastianb.ballcatcher.app.ship.YawJointController
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import nu.pattern.OpenCV
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.videoio.VideoCapture
import org.opencv.videoio.VideoWriter
import org.opencv.videoio.Videoio
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.sqrt
import kotlin.math.tan

/**
 * Base for trackers that use both stereo cameras to find some target (a ball, a face, ...) and pan
 * the head to keep it centred. Subclasses only implement [detectCentroid] — capture, averaging both
 * cameras' pixel offsets, rate-limited stepping, safety bounds, and triangulated 3D position for
 * reporting are all shared.
 *
 * Coordinate frame (head-local):
 *   +X = right, +Y = up, +Z = forward (away from cameras)
 *
 * @param rightCameraId    OpenCV device ID for the right camera (default 0)
 * @param leftCameraId     OpenCV device ID for the left camera (default 2)
 * @param cameraVFov       Vertical field of view in degrees (default 67°)
 * @param baselineMeters   Distance between the two cameras in metres (default 0.12 = 120 mm)
 * @param deadbandDeg      Ignore pan corrections smaller than this many degrees (default 3°)
 * @param updateIntervalMs How often to update the motor target in ms (default 150 ms = ~7 Hz)
 * @param smoothingAlpha   Exponential smoothing factor for the angular error (0 = frozen, 1 = raw). Default 0.2.
 * @param flipPanDirection Set true if the head pans the wrong way — swaps which camera is treated as left/right.
 * @param boundMarginDeg   Soft safety margin in degrees kept inside each calibrated limit (default 5°).
 * @param trackingMaxFreq  Max motor pulse frequency during tracking in Hz (default 100 — avoids step-skipping).
 * @param maxStepDeg       Max degrees the target angle may move per update tick (default 4°). Caps how far
 *                         a single bad detection can push the head, regardless of the computed error.
 * @param missToleranceTicks Number of consecutive missed detections to tolerate before giving up and
 *                           holding position (default 0 — freeze immediately on any miss). Detectors
 *                           that flicker frame-to-frame (e.g. a Haar cascade) benefit from a small
 *                           grace period so a single dropped frame doesn't stop tracking.
 * @param reacquireDeg     Once centred (error within [deadbandDeg]), the error must exceed this wider
 *                         threshold before tracking resumes (default equal to deadbandDeg — no
 *                         hysteresis). This is a Schmitt trigger: it stops small jitter right at the
 *                         deadband edge from causing repeated tiny corrections while already on target.
 * @param proportionalGain Fraction of the measured error commanded per tick (default 0.5). Values < 1
 *                         damp overshoot from capture/actuation latency; each tick re-measures, so the
 *                         head converges by repeated shrinking micro-adjustments rather than one jump.
 * @param bearingWindowMs  If > 0, average the target's *absolute bearing* over this trailing time
 *                         window instead of using the latest reading (default 0 = off). Averaging
 *                         absolute bearing rather than raw pixel offset is what makes this safe while
 *                         the head is panning — see the note at the averaging code below.
 * @param logTag           Prefix used in log lines so it's clear which tracker is running.
 */
abstract class StereoPanTracker(
    private val yawController: YawJointController,
    private val rightCameraId: Int = 0,
    private val leftCameraId: Int = 2,
    private val cameraVFov: Double = 67.0,
    private val baselineMeters: Double = 0.12,
    private val deadbandDeg: Double = 3.0,
    private val updateIntervalMs: Long = 150L,
    private val smoothingAlpha: Double = 0.2,
    private val flipPanDirection: Boolean = false,
    private val boundMarginDeg: Double = 5.0,
    private val trackingMaxFreq: Int = 100,
    private val maxStepDeg: Double = 4.0,
    private val missToleranceTicks: Int = 0,
    private val reacquireDeg: Double = deadbandDeg,
    private val proportionalGain: Double = 0.5,
    private val bearingWindowMs: Long = 0L,
    private val logTag: String = "StereoPanTracker",
) {
    companion object {
        init {
            OpenCV.loadLocally()
        }
    }

    /** Locate the target's pixel centroid in a single camera frame, or null if not found. */
    protected abstract fun detectCentroid(frame: Mat): Point?

    @Volatile
    private var trackingJob: Job? = null

    /** Most recently triangulated target position in head-local space (X, Y, Z) metres, null if not seen. */
    @Volatile
    var targetPosition3D: DoubleArray? = null
        private set

    val isTracking: Boolean get() = trackingJob?.isActive == true

    fun start(scope: CoroutineScope) {
        if (isTracking) return
        trackingJob = scope.launch {
            val camR = VideoCapture(rightCameraId)
            val camL = VideoCapture(leftCameraId)
            camR.set(Videoio.CAP_PROP_FOURCC, VideoWriter.fourcc('M', 'J', 'P', 'G').toDouble())
            camL.set(Videoio.CAP_PROP_FOURCC, VideoWriter.fourcc('M', 'J', 'P', 'G').toDouble())

            if (!camR.isOpened || !camL.isOpened) {
                println("$logTag: failed to open cameras — right: ${camR.isOpened}, left: ${camL.isOpened}")
                camR.release(); camL.release()
                return@launch
            }
            println("$logTag: stereo tracking started (right=$rightCameraId, left=$leftCameraId, baseline=${baselineMeters * 1000}mm)")

            // Left camera at -baseline/2 on X, right at +baseline/2; both pointing +Z
            val leftPos  = doubleArrayOf(-baselineMeters / 2.0, 0.0, 0.0)
            val rightPos = doubleArrayOf( baselineMeters / 2.0, 0.0, 0.0)
            val forward  = doubleArrayOf(0.0, 0.0, 1.0)
            val up       = doubleArrayOf(0.0, 1.0, 0.0)

            val ctrl = yawController.motorControl as YawJointController.HardwarePwmMotorControl
            val previousMaxFreq = ctrl.moveMaxFreq
            ctrl.moveMaxFreq = trackingMaxFreq  // cap speed during tracking — prevents step-skipping

            val frameR = Mat()
            val frameL = Mat()

            // Smoothed instantaneous angular error (offset of the target from head-forward), not an
            // absolute target — this is what gets rate-limited into a step, so a single bad reading
            // can't jump the target far.
            var smoothedError: Double? = null
            var missCount = 0

            // Trailing window of (timestampMs, absolute bearing) samples, used when bearingWindowMs > 0.
            val bearingHistory = ArrayDeque<Pair<Long, Double>>()

            // Schmitt-trigger lock: once the error falls within deadbandDeg we latch "locked" and
            // ignore further corrections until the error grows past the wider reacquireDeg. Without
            // this, an error hovering right at the deadband edge (e.g. detector bounding-box jitter
            // on an otherwise stationary target) would flicker in and out of correcting every tick.
            var locked = false

            try {
                while (isActive) {
                    val readR = camR.read(frameR)
                    val readL = camL.read(frameL)

                    if (!readR || !readL || frameR.empty() || frameL.empty()) {
                        delay(updateIntervalMs)
                        continue
                    }

                    // Sampled here, before detection runs, so it reflects where the head was pointing
                    // when these frames were actually captured — detection takes long enough that the
                    // head can have panned a degree or two by the time it finishes.
                    val angleAtCapture = yawController.motorFeedback.currentAngle

                    // A detector bug/quirk (e.g. a native binding edge case on zero detections) throwing
                    // here shouldn't kill the whole tracking coroutine — treat it as a missed frame.
                    val centerL: Point?
                    val centerR: Point?
                    try {
                        centerL = detectCentroid(frameL)
                        centerR = detectCentroid(frameR)
                    } catch (e: Exception) {
                        println("$logTag: detection error — ${e.message}")
                        delay(updateIntervalMs)
                        continue
                    }

                    if (centerL != null && centerR != null) {
                        missCount = 0
                        val w = frameL.cols().toDouble()
                        val h = frameL.rows().toDouble()
                        val pos3D = triangulate(centerL, centerR, w, h, leftPos, rightPos, forward, up)
                        targetPosition3D = pos3D

                        // Drive yaw off each camera's own pixel offset from its image center, averaged,
                        // rather than off the triangulated pos3D[0]/pos3D[2]. Triangulated depth (pos3D[2])
                        // can read near zero on a bad frame, sending atan2(x, z) to a wild angle — pixel
                        // offset has no such singularity, and since we only pan (no tilt/depth-dependent
                        // move), we don't need the triangulated bearing to drive the motor at all.
                        val angleOffsetL = angularOffsetDeg(centerL, w, h)
                        val angleOffsetR = angularOffsetDeg(centerR, w, h)
                        val rawAngle = (angleOffsetL + angleOffsetR) / 2.0
                        val angleToTarget = if (flipPanDirection) -rawAngle else rawAngle

                        // Convert the camera-relative offset into an absolute bearing before averaging.
                        // Raw pixel offsets are only meaningful relative to the head orientation that
                        // produced them, so averaging them across a window during which the head panned
                        // would blend measurements from different frames of reference and lag behind by
                        // however far the head travelled. Absolute bearing is head-motion invariant, so
                        // a stationary target yields a constant value no matter how the head moves.
                        val absoluteBearing = angleAtCapture + angleToTarget

                        val targetBearing = if (bearingWindowMs > 0) {
                            val now = System.currentTimeMillis()
                            bearingHistory.addLast(now to absoluteBearing)
                            while (bearingHistory.isNotEmpty() && now - bearingHistory.first().first > bearingWindowMs) {
                                bearingHistory.removeFirst()
                            }
                            bearingHistory.sumOf { it.second } / bearingHistory.size
                        } else {
                            absoluteBearing
                        }

                        // Error is recomputed against the live angle (not the angle at capture) so it
                        // accounts for movement already made toward the target since the frame was taken.
                        val currentAngle = yawController.motorFeedback.currentAngle
                        val rawError = targetBearing - currentAngle

                        // Exponential smoothing to damp out noisy detections
                        smoothedError = if (smoothedError == null) {
                            rawError
                        } else {
                            smoothedError!! * (1.0 - smoothingAlpha) + rawError * smoothingAlpha
                        }

                        val error = smoothedError!!
                        if (locked && abs(error) > reacquireDeg) locked = false

                        if (!locked && abs(error) > deadbandDeg) {
                            // Command a fraction of the remaining error, anchored to where the head
                            // actually IS — never accumulated onto ctrl.targetAngle. The motor lags well
                            // behind targetAngle during a correction (freq is capped at trackingMaxFreq),
                            // so adding each new error onto the previous, unreached target compounds every
                            // tick: the setpoint runs away far past the target and the head sails past it
                            // before unwinding. Re-anchoring each tick makes the command self-correcting —
                            // as the head closes in, the error shrinks and so do the steps.
                            val step = (error * proportionalGain).coerceIn(-maxStepDeg, maxStepDeg)
                            var newTarget = (currentAngle + step).toFloat()

                            // Hard stop if a limit switch is physically triggered — clamp to the current
                            // position so the motor stops immediately. Direction comes from the sign of
                            // the step (negative = toward the left switch), matching how tick() derives
                            // direction from targetAngle - currentAngle.
                            val feedback = yawController.motorFeedback
                            if (feedback.isAtLeftSwitch && step < 0) {
                                println("$logTag: left limit switch triggered — blocking further left movement")
                                newTarget = currentAngle.toFloat()
                            } else if (feedback.isAtRightSwitch && step > 0) {
                                println("$logTag: right limit switch triggered — blocking further right movement")
                                newTarget = currentAngle.toFloat()
                            } else {
                                // Soft bounds: stay boundMarginDeg inside each calibrated limit
                                val leftBound = yawController.calibratedLeftAngle
                                val rightBound = yawController.calibratedRightAngle
                                if (leftBound != null && rightBound != null) {
                                    val lo = (minOf(leftBound, rightBound) + boundMarginDeg).toFloat()
                                    val hi = (maxOf(leftBound, rightBound) - boundMarginDeg).toFloat()
                                    newTarget = newTarget.coerceIn(lo, hi)
                                }
                            }

                            ctrl.targetAngle = newTarget
                            println("$logTag: bearing=%.1f° (avg of %d), at=%.1f°, error=%.1f° → %.1f°".format(
                                targetBearing, bearingHistory.size.coerceAtLeast(1), currentAngle, error, newTarget
                            ))
                        } else if (!locked) {
                            // Within deadband and not yet locked — latch so subsequent jitter up to
                            // reacquireDeg is ignored instead of re-triggering a correction each tick.
                            locked = true
                        }
                    } else if (++missCount > missToleranceTicks) {
                        // Missed for longer than the grace period — give up and hold current
                        // position so the motor has nothing to chase.
                        targetPosition3D = null
                        smoothedError = null
                        locked = false
                        bearingHistory.clear()
                        ctrl.targetAngle = yawController.motorFeedback.currentAngle.toFloat()
                    }
                    // else: within the miss-tolerance grace period — coast toward the last commanded
                    // target instead of resetting, so a single flickered detection doesn't stop tracking.

                    delay(updateIntervalMs)
                }
            } finally {
                ctrl.moveMaxFreq = previousMaxFreq
                ctrl.targetAngle = yawController.motorFeedback.currentAngle.toFloat()
                camR.release()
                camL.release()
                frameR.release()
                frameL.release()
                targetPosition3D = null
                println("$logTag: stopped")
            }
        }
    }

    fun stop() {
        trackingJob?.cancel()
        trackingJob = null
    }

    // ── Single-camera horizontal bearing (used to drive yaw directly) ───────

    /** Horizontal angle in degrees from this camera's own optical axis to the pixel, ignoring depth. */
    private fun angularOffsetDeg(center: Point, width: Double, height: Double): Double {
        val aspect = width / height
        val tanHalfVfov = tan(Math.toRadians(cameraVFov) / 2.0)
        val tanHalfHfov = aspect * tanHalfVfov
        val nx = (center.x - width / 2.0) / (width / 2.0)
        val dx = nx * tanHalfHfov
        return Math.toDegrees(atan2(dx, 1.0))
    }

    // ── Stereo triangulation (math ported from opencv/TriangulateApp.kt) ────

    private fun triangulate(
        centerL: Point, centerR: Point,
        width: Double, height: Double,
        leftPos: DoubleArray, rightPos: DoubleArray,
        forward: DoubleArray, up: DoubleArray,
    ): DoubleArray {
        val vfovRad = Math.toRadians(cameraVFov)
        val fwd = normalize(forward)
        val right = normalize(cross(fwd, up))
        val upN = normalize(cross(right, fwd))

        val rayL = pixelToWorldRay(centerL.x, centerL.y, width, height, vfovRad, fwd, right, upN)
        val rayR = pixelToWorldRay(centerR.x, centerR.y, width, height, vfovRad, fwd, right, upN)

        return closestPointBetweenRays(leftPos, rayL, rightPos, rayR)
    }

    private fun pixelToWorldRay(
        u: Double, v: Double,
        width: Double, height: Double,
        vfovRad: Double,
        forward: DoubleArray, right: DoubleArray, up: DoubleArray,
    ): DoubleArray {
        val aspect = width / height
        val tanHalfVfov = tan(vfovRad / 2.0)
        val tanHalfHfov = aspect * tanHalfVfov
        val nx = (u - width / 2.0) / (width / 2.0)
        val ny = -(v - height / 2.0) / (height / 2.0)
        val dx = nx * tanHalfHfov
        val dy = ny * tanHalfVfov
        return normalize(doubleArrayOf(
            dx * right[0] + dy * up[0] + forward[0],
            dx * right[1] + dy * up[1] + forward[1],
            dx * right[2] + dy * up[2] + forward[2],
        ))
    }

    private fun closestPointBetweenRays(
        p1: DoubleArray, d1: DoubleArray,
        p2: DoubleArray, d2: DoubleArray,
    ): DoubleArray {
        val w0 = doubleArrayOf(p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2])
        val a = dot(d1, d1); val b = dot(d1, d2); val c = dot(d2, d2)
        val d = dot(d1, w0); val e = dot(d2, w0)
        val denom = a * c - b * b
        if (denom < 1e-10) return p1
        val t = (b * e - c * d) / denom
        val s = (a * e - b * d) / denom
        val c1 = doubleArrayOf(p1[0] + t * d1[0], p1[1] + t * d1[1], p1[2] + t * d1[2])
        val c2 = doubleArrayOf(p2[0] + s * d2[0], p2[1] + s * d2[1], p2[2] + s * d2[2])
        return doubleArrayOf((c1[0] + c2[0]) / 2, (c1[1] + c2[1]) / 2, (c1[2] + c2[2]) / 2)
    }

    private fun normalize(v: DoubleArray): DoubleArray {
        val len = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
        return doubleArrayOf(v[0] / len, v[1] / len, v[2] / len)
    }

    private fun cross(a: DoubleArray, b: DoubleArray) = doubleArrayOf(
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )

    private fun dot(a: DoubleArray, b: DoubleArray) = a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
