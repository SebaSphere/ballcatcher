package dev.sebastianb.ballcatcher.app.camera

import nu.pattern.OpenCV
import org.opencv.calib3d.Calib3d
import org.opencv.core.*
import org.opencv.imgcodecs.Imgcodecs
import org.opencv.imgproc.Imgproc
import java.io.File

data class FisheyeCalibrationResult(
    val kR: Mat,
    val dR: Mat,
    val kL: Mat,
    val dL: Mat,
    val imageSize: Size,
) {
    fun save(path: String) {
        val lines = buildString {
            appendLine("width=${imageSize.width}")
            appendLine("height=${imageSize.height}")
            appendLine("KR=${matToString(kR)}")
            appendLine("DR=${matToString(dR)}")
            appendLine("KL=${matToString(kL)}")
            appendLine("DL=${matToString(dL)}")
        }
        File(path).writeText(lines)
        println("FisheyeCalibrator: calibration saved to $path")
    }

    companion object {
        fun load(path: String): FisheyeCalibrationResult? {
            val file = File(path)
            if (!file.exists()) return null
            val props = file.readLines()
                .filter { it.contains('=') }
                .associate { line -> line.substringBefore('=') to line.substringAfter('=') }
            val width = props["width"]?.toDouble() ?: return null
            val height = props["height"]?.toDouble() ?: return null
            val kR = stringToMat(props["KR"] ?: return null, 3, 3)
            val dR = stringToMat(props["DR"] ?: return null, 4, 1)
            val kL = stringToMat(props["KL"] ?: return null, 3, 3)
            val dL = stringToMat(props["DL"] ?: return null, 4, 1)
            return FisheyeCalibrationResult(kR, dR, kL, dL, Size(width, height))
        }

        private fun matToString(mat: Mat): String {
            val total = mat.total().toInt() * mat.channels()
            val data = DoubleArray(total)
            mat.get(0, 0, data)
            return data.joinToString(",")
        }

        private fun stringToMat(s: String, rows: Int, cols: Int): Mat {
            val data = s.split(",").map { it.toDouble() }.toDoubleArray()
            val mat = Mat(rows, cols, CvType.CV_64F)
            mat.put(0, 0, *data)
            return mat
        }
    }
}

class FisheyeCalibrator(
    private val chessBoardSize: Size = Size(7.0, 7.0),
) {
    companion object {
        init {
            OpenCV.loadLocally()
        }
    }

    fun calibrate(imageDir: String): FisheyeCalibrationResult {
        val dir = File(imageDir)
        val rightFiles = dir.listFiles { f -> f.name.matches(Regex("chessboard-R\\d+\\.png")) }
            ?.sortedBy { it.name } ?: emptyList()
        val leftFiles = dir.listFiles { f -> f.name.matches(Regex("chessboard-L\\d+\\.png")) }
            ?.sortedBy { it.name } ?: emptyList()

        val count = minOf(rightFiles.size, leftFiles.size)
        require(count > 0) { "No matching chessboard image pairs found in $imageDir" }
        println("FisheyeCalibrator: found $count image pair(s) in $imageDir")

        val objp = buildObjectPoints()
        val criteria = TermCriteria(TermCriteria.EPS + TermCriteria.MAX_ITER, 30, 0.001)

        val objPointsR = mutableListOf<Mat>()
        val imgPointsR = mutableListOf<Mat>()
        val objPointsL = mutableListOf<Mat>()
        val imgPointsL = mutableListOf<Mat>()

        var imageSize: Size? = null
        val gray = Mat()

        for (i in 0 until count) {
            val imgR = Imgcodecs.imread(rightFiles[i].absolutePath)
            val imgL = Imgcodecs.imread(leftFiles[i].absolutePath)

            if (imageSize == null) imageSize = imgR.size()

            Imgproc.cvtColor(imgR, gray, Imgproc.COLOR_BGR2GRAY)
            val cornersR = MatOfPoint2f()
            if (Calib3d.findChessboardCorners(gray, chessBoardSize, cornersR)) {
                Imgproc.cornerSubPix(gray, cornersR, Size(11.0, 11.0), Size(-1.0, -1.0), criteria)
                objPointsR.add(objp)
                imgPointsR.add(cornersR)
            }

            Imgproc.cvtColor(imgL, gray, Imgproc.COLOR_BGR2GRAY)
            val cornersL = MatOfPoint2f()
            if (Calib3d.findChessboardCorners(gray, chessBoardSize, cornersL)) {
                Imgproc.cornerSubPix(gray, cornersL, Size(11.0, 11.0), Size(-1.0, -1.0), criteria)
                objPointsL.add(objp)
                imgPointsL.add(cornersL)
            }

            imgR.release()
            imgL.release()
        }
        gray.release()

        println("FisheyeCalibrator: corners detected in ${imgPointsR.size} right / ${imgPointsL.size} left images")
        require(imgPointsR.size >= 4) { "Need at least 4 right images with detected corners, got ${imgPointsR.size}" }
        require(imgPointsL.size >= 4) { "Need at least 4 left images with detected corners, got ${imgPointsL.size}" }

        val size = imageSize!!
        val kR = Mat.eye(3, 3, CvType.CV_64F)
        val dR = Mat.zeros(4, 1, CvType.CV_64F)
        val kL = Mat.eye(3, 3, CvType.CV_64F)
        val dL = Mat.zeros(4, 1, CvType.CV_64F)
        val flags = Calib3d.fisheye_CALIB_RECOMPUTE_EXTRINSIC + Calib3d.fisheye_CALIB_FIX_SKEW

        Calib3d.fisheye_calibrate(
            objPointsR, imgPointsR, size, kR, dR, ArrayList(), ArrayList(), flags
        )
        println("FisheyeCalibrator: right camera calibrated — K=$kR D=$dR")

        Calib3d.fisheye_calibrate(
            objPointsL, imgPointsL, size, kL, dL, ArrayList(), ArrayList(), flags
        )
        println("FisheyeCalibrator: left camera calibrated — K=$kL D=$dL")

        return FisheyeCalibrationResult(kR, dR, kL, dL, size)
    }

    private fun buildObjectPoints(): MatOfPoint3f {
        val pts = (0 until chessBoardSize.height.toInt()).flatMap { y ->
            (0 until chessBoardSize.width.toInt()).map { x ->
                Point3(x.toDouble(), y.toDouble(), 0.0)
            }
        }.toTypedArray()
        return MatOfPoint3f(*pts)
    }
}
