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

            try {
                while (isActive) {
                    val readR = camR.read(frameR)
                    val readL = camL.read(frameL)

                    if (!readR || !readL || frameR.empty() || frameL.empty()) {
                        delay(updateIntervalMs)
                        continue
                    }

                    val centerL = detectCentroid(frameL)
                    val centerR = detectCentroid(frameR)

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

                        // Exponential smoothing on the raw offset itself to damp out noisy detections
                        smoothedError = if (smoothedError == null) {
                            angleToTarget
                        } else {
                            smoothedError!! * (1.0 - smoothingAlpha) + angleToTarget * smoothingAlpha
                        }

                        if (abs(smoothedError!!) > deadbandDeg) {
                            // Rate-limit: move at most maxStepDeg per tick toward the target, regardless of
                            // how large the computed error is — a second line of defense against any single
                            // noisy detection swinging the head too far.
                            val step = smoothedError!!.coerceIn(-maxStepDeg, maxStepDeg)
                            var newTarget = (ctrl.targetAngle + step).toFloat()

                            // Hard stop if a limit switch is physically triggered —
                            // clamp to current position so the motor stops immediately.
                            val feedback = yawController.motorFeedback
                            if (feedback.isAtLeftSwitch && newTarget < ctrl.targetAngle) {
                                println("$logTag: left limit switch triggered — blocking further left movement")
                                newTarget = ctrl.targetAngle
                            } else if (feedback.isAtRightSwitch && newTarget > ctrl.targetAngle) {
                                println("$logTag: right limit switch triggered — blocking further right movement")
                                newTarget = ctrl.targetAngle
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
                            println("$logTag: target at (%.3f, %.3f, %.3f)m, error=%.1f° → step to %.1f°".format(
                                pos3D[0], pos3D[1], pos3D[2], smoothedError, newTarget
                            ))
                        }
                    } else if (++missCount > missToleranceTicks) {
                        // Missed for longer than the grace period — give up and hold current
                        // position so the motor has nothing to chase.
                        targetPosition3D = null
                        smoothedError = null
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
