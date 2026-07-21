package dev.sebastianb.ballcatcher.app.camera

import dev.sebastianb.ballcatcher.app.ship.YawJointController
import kotlinx.coroutines.delay
import kotlinx.coroutines.yield
import nu.pattern.OpenCV
import org.opencv.core.Mat
import org.opencv.imgcodecs.Imgcodecs
import org.opencv.videoio.VideoCapture
import org.opencv.videoio.VideoWriter
import org.opencv.videoio.Videoio
import java.io.File
import java.time.LocalDateTime
import java.time.format.DateTimeFormatter

class ScanCycle(
    val leftAngle: Double,
    val rightAngle: Double,
    private val yawController: YawJointController,
    private val rightCameraId: Int = 0,
    private val leftCameraId: Int = 2,
) {
    companion object {
        private val TIMESTAMP_FORMAT = DateTimeFormatter.ofPattern("yyyy-MM-dd_HH-mm-ss-SSS")

        init {
            OpenCV.loadLocally()
        }
    }

    // Tracks which direction the next sweep should go (alternates each cycle call)
    private var sweepForward = true

    /**
     * Returns [count] evenly-spaced angles between [leftAngle] and [rightAngle] (inclusive).
     * e.g. count=5, range=-90..0 → [-90, -67.5, -45, -22.5, 0]
     */
    fun spreadAngles(count: Int): List<Double> {
        val n = count.coerceAtLeast(2)
        return List(n) { i -> leftAngle + (rightAngle - leftAngle) * (i.toDouble() / (n - 1)) }
    }

    /**
     * Sweeps the head back and forth (left→right on odd cycles, right→left on even cycles),
     * moving to each angle [stepDegrees] apart. At each position the head pauses [capturePauseMs]
     * before capturing both cameras.
     *
     * On [firstRun], moves to the starting position before beginning.
     *
     * Folder structure:
     *   {baseDir}/{sessionTimestamp}_cycle{cycleNumber}/{stepTimestamp}_step{stepIndex}/
     *     left.png
     *     right.png
     *     info.txt
     */
    suspend fun cycle(
        sessionTimestamp: String,
        cycleNumber: Int,
        count: Int,
        sweepSpeed: Int = 200,
        capturePauseMs: Long = 500,
        firstRun: Boolean = false,
        baseDir: String = "scan_logs",
    ) {
        val angles = spreadAngles(count)
        // Alternate direction each cycle
        val orderedAngles = if (sweepForward) angles else angles.reversed()
        sweepForward = !sweepForward

        val sessionDir = File("$baseDir/${sessionTimestamp}_cycle$cycleNumber")
        sessionDir.mkdirs()

        val camR = VideoCapture(rightCameraId)
        val camL = VideoCapture(leftCameraId)

        println("ScanCycle — right camera (id=$rightCameraId) opened: ${camR.isOpened}")
        println("ScanCycle — left camera (id=$leftCameraId) opened: ${camL.isOpened}")

        if (!camR.isOpened || !camL.isOpened) {
            camR.release()
            camL.release()
            error("ScanCycle: failed to open cameras — right: ${camR.isOpened}, left: ${camL.isOpened}")
        }

        camR.set(Videoio.CAP_PROP_FOURCC, VideoWriter.fourcc('M', 'J', 'P', 'G').toDouble())
        camL.set(Videoio.CAP_PROP_FOURCC, VideoWriter.fourcc('M', 'J', 'P', 'G').toDouble())

        val frameR = Mat()
        val frameL = Mat()

        try {
            // On first run, move to the starting position before capturing
            if (firstRun) {
                val startAngle = orderedAngles.first()
                val startFraction = angleFraction(startAngle)
                println("ScanCycle — first run, moving to start position ${startAngle}°")
                yawController.moveToPosition(startFraction, sweepSpeed)
            }

            orderedAngles.forEachIndexed { index, targetAngle ->
                val stepIndex = index + 1
                val fraction = angleFraction(targetAngle)

                // Move to this angle and wait until reached
                yawController.moveToPosition(fraction, sweepSpeed)

                val stepTimestamp = LocalDateTime.now().format(TIMESTAMP_FORMAT)
                val stepDir = File("${sessionDir.path}/${stepTimestamp}_step$stepIndex")
                stepDir.mkdirs()

                val currentAngle = yawController.motorFeedback.currentAngle

                // Grab frames (retry up to 100 times)
                var readFailures = 0
                while (true) {
                    val readR = camR.read(frameR)
                    val readL = camL.read(frameL)
                    if (!readR || !readL) {
                        readFailures++
                        if (readFailures > 100) error("ScanCycle: too many frame read failures at step $stepIndex")
                        continue
                    }
                    break
                }

                Imgcodecs.imwrite(File("${stepDir.path}/right.png").absolutePath, frameR)
                Imgcodecs.imwrite(File("${stepDir.path}/left.png").absolutePath, frameL)

                File("${stepDir.path}/info.txt").writeText(
                    """
                    Timestamp: $stepTimestamp
                    Cycle: $cycleNumber
                    Step: $stepIndex / ${orderedAngles.size}
                    Target angle: $targetAngle
                    Current angle: $currentAngle
                    Left boundary: $leftAngle
                    Right boundary: $rightAngle
                    """.trimIndent()
                )

                println("ScanCycle — step $stepIndex captured at ${currentAngle}° (target ${targetAngle}°)")
                yield()
            }

            // Pause before direction alternates on the next cycle
            delay(capturePauseMs)
            println("ScanCycle — cycle $cycleNumber complete, ${orderedAngles.size} steps captured")
        } finally {
            camR.release()
            camL.release()
            frameR.release()
            frameL.release()
        }
    }

    private fun angleFraction(angle: Double): Float {
        return ((angle - leftAngle) / (rightAngle - leftAngle)).toFloat().coerceIn(0f, 1f)
    }
}
