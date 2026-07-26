package dev.sebastianb.ballcatcher.app.camera

import dev.sebastianb.ballcatcher.app.ship.YawJointController
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc

/** Tracks a green ball using both stereo cameras. See [StereoPanTracker] for the shared control loop. */
class BallTracker(
    yawController: YawJointController,
    rightCameraId: Int = 0,
    leftCameraId: Int = 2,
    cameraVFov: Double = 67.0,
    baselineMeters: Double = 0.12,
    deadbandDeg: Double = 3.0,
    updateIntervalMs: Long = 150L,
    smoothingAlpha: Double = 0.2,
    flipPanDirection: Boolean = false,
    boundMarginDeg: Double = 5.0,
    trackingMaxFreq: Int = 100,
    maxStepDeg: Double = 4.0,
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
    logTag = "BallTracker",
) {
    override fun detectCentroid(frame: Mat): Point? {
        val hsv = Mat()
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_BGR2HSV)

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
}
