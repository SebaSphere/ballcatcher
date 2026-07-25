package dev.sebastianb.ballcatcher.app.camera

import dev.sebastianb.ballcatcher.app.ship.YawJointController
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.yield
import nu.pattern.OpenCV
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.opencv.videoio.VideoCapture
import org.opencv.videoio.VideoWriter
import org.opencv.videoio.Videoio
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.sqrt
import kotlin.math.tan

/**
 * Tracks a green ball using both stereo cameras to compute its 3D position,
 * then pans the head to keep it centered.
 *
 * Coordinate frame (head-local):
 *   +X = right, +Y = up, +Z = forward (away from cameras)
 *
 * @param rightCameraId  OpenCV device ID for the right camera (default 0)
 * @param leftCameraId   OpenCV device ID for the left camera (default 2)
 * @param cameraVFov     Vertical field of view in degrees (default 67°)
 * @param baselineMeters Distance between the two cameras in metres (default 0.12 = 120 mm)
 * @param deadbandDeg    Ignore pan corrections smaller than this many degrees (default 1°)
 */
class BallTracker(
    private val yawController: YawJointController,
    private val rightCameraId: Int = 0,
    private val leftCameraId: Int = 2,
    private val cameraVFov: Double = 67.0,
    private val baselineMeters: Double = 0.12,
    private val deadbandDeg: Double = 1.0,
) {
    companion object {
        init {
            OpenCV.loadLocally()
        }
    }

    @Volatile
    private var trackingJob: Job? = null

    /** The most recently triangulated ball position in head-local space (X, Y, Z) in metres, or null if not seen. */
    @Volatile
    var ballPosition3D: DoubleArray? = null
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
                println("BallTracker: failed to open cameras — right: ${camR.isOpened}, left: ${camL.isOpened}")
                camR.release(); camL.release()
                return@launch
            }
            println("BallTracker: stereo tracking started (right=$rightCameraId, left=$leftCameraId, baseline=${baselineMeters * 1000}mm)")

            // Left camera sits at -baseline/2 on the X axis, right at +baseline/2
            val leftPos  = doubleArrayOf(-baselineMeters / 2.0, 0.0, 0.0)
            val rightPos = doubleArrayOf( baselineMeters / 2.0, 0.0, 0.0)
            val forward  = doubleArrayOf(0.0, 0.0, 1.0)
            val up       = doubleArrayOf(0.0, 1.0, 0.0)

            val frameR = Mat()
            val frameL = Mat()
            try {
                while (isActive) {
                    val readR = camR.read(frameR)
                    val readL = camL.read(frameL)

                    if (!readR || !readL || frameR.empty() || frameL.empty()) {
                        yield()
                        continue
                    }

                    val centerL = detectBallCentroid(frameL)
                    val centerR = detectBallCentroid(frameR)

                    if (centerL != null && centerR != null) {
                        val w = frameL.cols().toDouble()
                        val h = frameL.rows().toDouble()
                        val pos3D = triangulate(centerL, centerR, w, h, leftPos, rightPos, forward, up)
                        ballPosition3D = pos3D

                        // atan2(X, Z) gives horizontal angle to the ball from the camera midpoint
                        val panOffsetDeg = Math.toDegrees(atan2(pos3D[0], pos3D[2]))
                        println("BallTracker: ball at (%.3f, %.3f, %.3f)m — pan offset %.1f°".format(
                            pos3D[0], pos3D[1], pos3D[2], panOffsetDeg
                        ))

                        if (abs(panOffsetDeg) > deadbandDeg) {
                            applyPanOffset(panOffsetDeg)
                        }
                    } else {
                        ballPosition3D = null
                    }

                    yield()
                }
            } finally {
                camR.release()
                camL.release()
                frameR.release()
                frameL.release()
                ballPosition3D = null
                println("BallTracker: stopped")
            }
        }
    }

    fun stop() {
        trackingJob?.cancel()
        trackingJob = null
    }

    private fun applyPanOffset(offsetDeg: Double) {
        val ctrl = yawController.motorControl as YawJointController.HardwarePwmMotorControl
        val currentAngle = yawController.motorFeedback.currentAngle
        val leftBound = yawController.calibratedLeftAngle
        val rightBound = yawController.calibratedRightAngle

        var newTarget = (currentAngle + offsetDeg).toFloat()
        if (leftBound != null && rightBound != null) {
            val lo = minOf(leftBound, rightBound).toFloat()
            val hi = maxOf(leftBound, rightBound).toFloat()
            newTarget = newTarget.coerceIn(lo, hi)
        }
        ctrl.targetAngle = newTarget
    }

    // ── Detection ────────────────────────────────────────────────────────────

    private fun detectBallCentroid(img: Mat): Point? {
        val hsv = Mat()
        Imgproc.cvtColor(img, hsv, Imgproc.COLOR_BGR2HSV)

        val mask = Mat()
        Core.inRange(hsv, Scalar(35.0, 80.0, 80.0), Scalar(85.0, 255.0, 255.0), mask)
        hsv.release()

        val contours = mutableListOf<MatOfPoint>()
        val hierarchy = Mat()
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)
        mask.release()
        hierarchy.release()

        if (contours.isEmpty()) return null
        val largest = contours.maxBy { Imgproc.contourArea(it) }
        val m = Imgproc.moments(largest)
        if (m.m00 == 0.0) return null
        return Point(m.m10 / m.m00, m.m01 / m.m00)
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
        if (denom < 1e-10) return p1 // rays nearly parallel — can't triangulate
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
