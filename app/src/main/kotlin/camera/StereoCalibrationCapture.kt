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

    fun capture(count: Int): Int {
        println("Starting calibration capture: saving $count image pairs")

        var imageId = 0
        val criteria = TermCriteria(TermCriteria.EPS + TermCriteria.MAX_ITER, 30, 0.001)

        val camR = VideoCapture(rightCameraId)
        val camL = VideoCapture(leftCameraId)

        val frameR = Mat()
        val frameL = Mat()
        val grayR = Mat()
        val grayL = Mat()

        try {
            while (imageId < count) {
                if (!camR.read(frameR) || !camL.read(frameL)) continue

                Imgproc.cvtColor(frameR, grayR, Imgproc.COLOR_BGR2GRAY)
                Imgproc.cvtColor(frameL, grayL, Imgproc.COLOR_BGR2GRAY)

                val cornersR = MatOfPoint2f()
                val cornersL = MatOfPoint2f()
                val retR = Calib3d.findChessboardCorners(grayR, chessBoardSize, cornersR)
                val retL = Calib3d.findChessboardCorners(grayL, chessBoardSize, cornersL)

                if (retR && retL) {
                    Imgproc.cornerSubPix(grayR, cornersR, Size(11.0, 11.0), Size(-1.0, -1.0), criteria)
                    Imgproc.cornerSubPix(grayL, cornersL, Size(11.0, 11.0), Size(-1.0, -1.0), criteria)

                    println("Images $imageId saved for right and left cameras")
                    Imgcodecs.imwrite("$outputDirectory/chessboard-R$imageId.png", frameR)
                    Imgcodecs.imwrite("$outputDirectory/chessboard-L$imageId.png", frameL)
                    println("Images saved at $outputDirectory/chessboard-R$imageId.png and $outputDirectory/chessboard-L$imageId.png")
                    imageId++
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

        println("Calibration capture complete: $imageId image pairs saved")
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
