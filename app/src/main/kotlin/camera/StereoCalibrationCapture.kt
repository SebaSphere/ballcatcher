package dev.sebastianb.ballcatcher.app.camera

import nu.pattern.OpenCV
import org.opencv.calib3d.Calib3d
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Size
import org.opencv.core.TermCriteria
import org.opencv.highgui.HighGui
import org.opencv.imgcodecs.Imgcodecs
import org.opencv.imgproc.Imgproc
import org.opencv.videoio.VideoCapture
import org.opencv.videoio.VideoWriter
import org.opencv.videoio.Videoio

class StereoCalibrationCapture(
    private val rightCameraId: Int = 0,
    private val leftCameraId: Int = 2,
    private val chessBoardSize: Size = Size(7.0, 7.0),
    private val outputDirectory: String = "."
) {
    companion object {
        init {
            OpenCV.loadLocally()
        }
    }

    fun capture(count: Int, delayMs: Long = 0): Int {
        println("[CalibCapture] Starting: target=$count pairs, delay=${delayMs}ms, board=${chessBoardSize.width.toInt()}x${chessBoardSize.height.toInt()}, outputDir=$outputDirectory")
        val sessionStart = System.currentTimeMillis()

        var imageId = 0
        val criteria = TermCriteria(TermCriteria.EPS + TermCriteria.MAX_ITER, 30, 0.001)

        val camR = VideoCapture(rightCameraId)
        val camL = VideoCapture(leftCameraId)

        println("[CalibCapture] Camera open — right(id=$rightCameraId): ${camR.isOpened}, left(id=$leftCameraId): ${camL.isOpened}")

        if (!camR.isOpened || !camL.isOpened) {
            camR.release()
            camL.release()
            error("Failed to open cameras — right: ${camR.isOpened}, left: ${camL.isOpened}")
        }

        camR.set(Videoio.CAP_PROP_FOURCC, VideoWriter.fourcc('M', 'J', 'P', 'G').toDouble())
        camL.set(Videoio.CAP_PROP_FOURCC, VideoWriter.fourcc('M', 'J', 'P', 'G').toDouble())

        val resRW = camR.get(Videoio.CAP_PROP_FRAME_WIDTH).toInt()
        val resRH = camR.get(Videoio.CAP_PROP_FRAME_HEIGHT).toInt()
        val resLW = camL.get(Videoio.CAP_PROP_FRAME_WIDTH).toInt()
        val resLH = camL.get(Videoio.CAP_PROP_FRAME_HEIGHT).toInt()
        println("[CalibCapture] Camera resolution — right: ${resRW}x${resRH}, left: ${resLW}x${resLH}")

        val frameR = Mat()
        val frameL = Mat()
        val grayR = Mat()
        val grayL = Mat()

        try {
            var readFailures = 0
            var totalFrames = 0
            var chessboardMisses = 0

            while (imageId < count) {
                val frameStart = System.currentTimeMillis()

                val readR = camR.read(frameR)
                val readL = camL.read(frameL)
                val readMs = System.currentTimeMillis() - frameStart

                if (!readR || !readL) {
                    readFailures++
                    println("[CalibCapture] Frame read FAILED (right=$readR, left=$readL) — failure #$readFailures, elapsed=${System.currentTimeMillis() - sessionStart}ms")
                    if (readFailures > 100) {
                        error("Too many consecutive read failures, aborting")
                    }
                    continue
                }
                readFailures = 0
                totalFrames++

                Imgproc.cvtColor(frameR, grayR, Imgproc.COLOR_BGR2GRAY)
                Imgproc.cvtColor(frameL, grayL, Imgproc.COLOR_BGR2GRAY)

                val cornersR = MatOfPoint2f()
                val cornersL = MatOfPoint2f()

                val detectStart = System.currentTimeMillis()
                val retR = Calib3d.findChessboardCorners(grayR, chessBoardSize, cornersR)
                val retL = Calib3d.findChessboardCorners(grayL, chessBoardSize, cornersL)
                val detectMs = System.currentTimeMillis() - detectStart

                if (!retR || !retL) {
                    chessboardMisses++
                    if (chessboardMisses % 10 == 0) {
                        println("[CalibCapture] Chessboard NOT found — right=$retR, left=$retL | frames=$totalFrames, misses=$chessboardMisses, captured=$imageId/$count, readMs=$readMs, detectMs=$detectMs, elapsed=${System.currentTimeMillis() - sessionStart}ms")
                    }
                    continue
                }

                val subpixStart = System.currentTimeMillis()
                Imgproc.cornerSubPix(grayR, cornersR, Size(11.0, 11.0), Size(-1.0, -1.0), criteria)
                Imgproc.cornerSubPix(grayL, cornersL, Size(11.0, 11.0), Size(-1.0, -1.0), criteria)
                val subpixMs = System.currentTimeMillis() - subpixStart

                println("[CalibCapture] Chessboard FOUND — saving pair $imageId | readMs=$readMs, detectMs=$detectMs, subpixMs=$subpixMs, missesSinceLastCapture=$chessboardMisses, totalFrames=$totalFrames")
                chessboardMisses = 0

                val writeStart = System.currentTimeMillis()
                Imgcodecs.imwrite("$outputDirectory/chessboard-R$imageId.png", frameR)
                Imgcodecs.imwrite("$outputDirectory/chessboard-L$imageId.png", frameL)
                val writeMs = System.currentTimeMillis() - writeStart

                imageId++
                println("[CalibCapture] Pair $imageId/$count saved (writeMs=$writeMs) → $outputDirectory/chessboard-[RL]${imageId - 1}.png | elapsed=${System.currentTimeMillis() - sessionStart}ms")

                if (delayMs > 0 && imageId < count) {
                    println("[CalibCapture] Sleeping ${delayMs}ms before next capture…")
                    Thread.sleep(delayMs)
                }
            }
        } finally {
            camR.release()
            camL.release()
            frameR.release()
            frameL.release()
            grayR.release()
            grayL.release()
        }

        println("[CalibCapture] Done — $imageId pairs saved in ${System.currentTimeMillis() - sessionStart}ms")
        return imageId
    }
}
//    fun run() {
//        println("Starting the Calibration. Press and maintain the space bar to exit the script\n")
//        println("Push (s) to save the image you want and push (c) to see next frame without saving the image")
//
//        var imageId = 0
//        val criteria = TermCriteria(TermCriteria.EPS + TermCriteria.MAX_ITER, 30, 0.001)
//
//        val camR = VideoCapture(rightCameraId)
//        val camL = VideoCapture(leftCameraId)
//
//        val frameR = Mat()
//        val frameL = Mat()
//        val grayR = Mat()
//        val grayL = Mat()
//
//        try {
//            while (true) {
//                if (!camR.read(frameR) || !camL.read(frameL)) continue
//
//                Imgproc.cvtColor(frameR, grayR, Imgproc.COLOR_BGR2GRAY)
//                Imgproc.cvtColor(frameL, grayL, Imgproc.COLOR_BGR2GRAY)
//
//                val cornersR = MatOfPoint2f()
//                val cornersL = MatOfPoint2f()
//                val retR = Calib3d.findChessboardCorners(grayR, chessBoardSize, cornersR)
//                val retL = Calib3d.findChessboardCorners(grayL, chessBoardSize, cornersL)
//
//                HighGui.imshow("imgR", frameR)
//                HighGui.imshow("imgL", frameL)
//
//                if (retR && retL) {
//                    Imgproc.cornerSubPix(grayR, cornersR, Size(11.0, 11.0), Size(-1.0, -1.0), criteria)
//                    Imgproc.cornerSubPix(grayL, cornersL, Size(11.0, 11.0), Size(-1.0, -1.0), criteria)
//
//                    Calib3d.drawChessboardCorners(grayR, chessBoardSize, cornersR, retR)
//                    Calib3d.drawChessboardCorners(grayL, chessBoardSize, cornersL, retL)
//                    HighGui.imshow("VideoR", grayR)
//                    HighGui.imshow("VideoL", grayL)
//
//                    val key = HighGui.waitKey(0) and 0xFF
//                    if (key == 's'.code) {
//                        println("Images $imageId saved for right and left cameras")
//                        Imgcodecs.imwrite("$outputDirectory/chessboard-R$imageId.png", frameR)
//                        Imgcodecs.imwrite("$outputDirectory/chessboard-L$imageId.png", frameL)
//                        imageId++
//                    } else {
//                        println("Images not saved")
//                    }
//                }
//
//                if (HighGui.waitKey(1) and 0xFF == ' '.code) {
//                    break
//                }
//            }
//        } finally {
//            camR.release()
//            camL.release()
//            HighGui.destroyAllWindows()
//            frameR.release()
//            frameL.release()
//            grayR.release()
//            grayL.release()
//        }
//    }
//}
//
//fun main() {
//    StereoCalibrationCapture().run()
//}
