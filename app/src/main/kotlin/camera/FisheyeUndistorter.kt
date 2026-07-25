package dev.sebastianb.ballcatcher.app.camera

import org.opencv.calib3d.Calib3d
import org.opencv.core.*
import org.opencv.imgproc.Imgproc

class FisheyeUndistorter(K: Mat, D: Mat, imageSize: Size) {
    private val map1 = Mat()
    private val map2 = Mat()

    init {
        val newK = Mat()
        Calib3d.fisheye_estimateNewCameraMatrixForUndistortRectify(
            K, D, imageSize, Mat.eye(3, 3, CvType.CV_64F), newK,
            /* balance= */ 1.0, imageSize, /* fov_scale= */ 1.0
        )
        Calib3d.fisheye_initUndistortRectifyMap(
            K, D, Mat.eye(3, 3, CvType.CV_64F), newK,
            imageSize, CvType.CV_16SC2, map1, map2
        )
        newK.release()
    }

    fun undistort(src: Mat, dst: Mat) {
        Imgproc.remap(src, dst, map1, map2, Imgproc.INTER_LINEAR)
    }
}
