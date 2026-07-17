package dev.sebastianb.ballcatcher.app.camera

import nu.pattern.OpenCV
import org.opencv.core.Mat
import org.opencv.imgcodecs.Imgcodecs
import org.opencv.videoio.VideoCapture
import org.opencv.videoio.VideoWriter
import org.opencv.videoio.Videoio
import java.io.File
import java.time.LocalDateTime
import java.time.format.DateTimeFormatter

class RawPhotoCapture(
    private val rightCameraId: Int = 0,
    private val leftCameraId: Int = 2,
) {
    companion object {
        private val TIMESTAMP_FORMAT = DateTimeFormatter.ofPattern("yyyy-MM-dd_HH-mm-ss-SSS")
        private val BASE_DIR = "raw_photo_logs"

        init {
            OpenCV.loadLocally()
        }
    }

    data class CaptureResult(val timestamp: String, val rightPath: String, val leftPath: String)

    fun capture(): CaptureResult {
        val timestamp = LocalDateTime.now().format(TIMESTAMP_FORMAT)
        val outputDir = File("$BASE_DIR/$timestamp")
        outputDir.mkdirs()

        val camR = VideoCapture(rightCameraId)
        val camL = VideoCapture(leftCameraId)

        println("Raw capture — right camera (id=$rightCameraId) opened: ${camR.isOpened}")
        println("Raw capture — left camera (id=$leftCameraId) opened: ${camL.isOpened}")

        if (!camR.isOpened || !camL.isOpened) {
            camR.release()
            camL.release()
            error("Failed to open cameras — right: ${camR.isOpened}, left: ${camL.isOpened}")
        }

        camR.set(Videoio.CAP_PROP_FOURCC, VideoWriter.fourcc('M', 'J', 'P', 'G').toDouble())
        camL.set(Videoio.CAP_PROP_FOURCC, VideoWriter.fourcc('M', 'J', 'P', 'G').toDouble())

        val frameR = Mat()
        val frameL = Mat()

        try {
            var readFailures = 0
            while (true) {
                val readR = camR.read(frameR)
                val readL = camL.read(frameL)
                if (!readR || !readL) {
                    readFailures++
                    if (readFailures > 100) error("Too many frame read failures")
                    continue
                }
                break
            }

            val rightPath = "${outputDir.path}/right.png"
            val leftPath = "${outputDir.path}/left.png"

            Imgcodecs.imwrite(rightPath, frameR)
            Imgcodecs.imwrite(leftPath, frameL)

            println("Raw photos saved to ${outputDir.path}/")
            return CaptureResult(timestamp, rightPath, leftPath)
        } finally {
            camR.release()
            camL.release()
            frameR.release()
            frameL.release()
        }
    }
}
