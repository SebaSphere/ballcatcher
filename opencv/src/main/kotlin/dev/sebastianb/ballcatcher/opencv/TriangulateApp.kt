package dev.sebastianb.ballcatcher.opencv

import org.opencv.core.*
import org.opencv.imgcodecs.Imgcodecs
import org.opencv.imgproc.Imgproc
import java.io.File
import kotlin.math.sqrt
import kotlin.math.tan

fun main(args: Array<String>) {
    nu.pattern.OpenCV.loadLocally()

    val screenshotDir = if (args.isNotEmpty()) {
        args[0]
    } else {
        val screenshotsRoot = File("../cam/screenshots")
        val latest = screenshotsRoot.listFiles()
            ?.filter { it.isDirectory }
            ?.maxByOrNull { it.name }
            ?: error("No screenshot directories found in ${screenshotsRoot.absolutePath}")
        latest.path
    }

    val cam1Path = "$screenshotDir/cam1.png"
    val cam2Path = "$screenshotDir/cam2.png"
    val camerasPath = "$screenshotDir/cameras.txt"
    val spherePath = "$screenshotDir/sphere.txt"

    // Load images
    val img1 = Imgcodecs.imread(cam1Path)
    val img2 = Imgcodecs.imread(cam2Path)
    require(!img1.empty()) { "Failed to load $cam1Path" }
    require(!img2.empty()) { "Failed to load $cam2Path" }

    println("Loaded images: ${img1.cols()}x${img1.rows()}")

    // Parse camera data
    val cameraData = parseCameras(File(camerasPath).readText())
    println("Left  camera: (%.4f, %.4f, %.4f)".format(cameraData.leftPos[0], cameraData.leftPos[1], cameraData.leftPos[2]))
    println("Right camera: (%.4f, %.4f, %.4f)".format(cameraData.rightPos[0], cameraData.rightPos[1], cameraData.rightPos[2]))

    // Detect the ball in each image
    val center1 = detectBall(img1, "left")
    val center2 = detectBall(img2, "right")

    if (center1 == null || center2 == null) {
        println("ERROR: Could not detect the ball in one or both images.")
        return
    }

    println("Ball pixel in left  image: (${center1.x}, ${center1.y})")
    println("Ball pixel in right image: (${center2.x}, ${center2.y})")

    // Triangulate
    val point = triangulate(
        cameraData, center1, center2,
        img1.cols().toDouble(), img1.rows().toDouble()
    )

    println()
    println("=== Triangulated 3D position ===")
    println("X: %.4f".format(point[0]))
    println("Y: %.4f".format(point[1]))
    println("Z: %.4f".format(point[2]))

    // Compare with actual sphere position if available
    val sphereFile = File(spherePath)
    if (sphereFile.exists()) {
        val actual = parseSphere(sphereFile.readText())
        val error = sqrt(
            (point[0] - actual[0]).let { it * it } +
            (point[1] - actual[1]).let { it * it } +
            (point[2] - actual[2]).let { it * it }
        )
        println()
        println("=== Actual sphere position ===")
        println("X: %.4f".format(actual[0]))
        println("Y: %.4f".format(actual[1]))
        println("Z: %.4f".format(actual[2]))
        println("Margin Distance: %.4f".format(error))
    }
}

data class CameraData(
    val leftPos: DoubleArray,
    val rightPos: DoubleArray,
    val forward: DoubleArray,
    val up: DoubleArray
)

fun triangulate(
    cameraData: CameraData,
    center1: Point, center2: Point,
    width: Double, height: Double
): DoubleArray {
    // Camera intrinsics matching CubeScene: FOV 67 vertical
    val vfov = Math.toRadians(67.0)

    val forward = normalize(cameraData.forward)
    val right = normalize(cross(forward, cameraData.up))
    val up = normalize(cross(right, forward))

    // Build rays from pixel coordinates
    val ray1 = pixelToWorldRay(center1.x, center1.y, width, height, vfov, forward, right, up)
    val ray2 = pixelToWorldRay(center2.x, center2.y, width, height, vfov, forward, right, up)

    // Triangulate: find closest point between two rays
    return closestPointBetweenRays(cameraData.leftPos, ray1, cameraData.rightPos, ray2)
}

/** Detect the green ball by HSV thresholding and return its centroid. */
fun detectBall(img: Mat, label: String): Point? {
    val hsv = Mat()
    Imgproc.cvtColor(img, hsv, Imgproc.COLOR_BGR2HSV)

    // Green in HSV: hue ~35-85 (OpenCV hue is 0-180), high saturation, high value
    val lower = Scalar(35.0, 80.0, 80.0)
    val upper = Scalar(85.0, 255.0, 255.0)
    val mask = Mat()
    Core.inRange(hsv, lower, upper, mask)

    // Find contours
    val contours = mutableListOf<MatOfPoint>()
    val hierarchy = Mat()
    Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)

    if (contours.isEmpty()) {
        println("No green object found in $label image")
        return null
    }

    // Pick the largest contour
    val largest = contours.maxBy { Imgproc.contourArea(it) }
    val moments = Imgproc.moments(largest)

    if (moments.m00 == 0.0) return null

    val cx = moments.m10 / moments.m00
    val cy = moments.m01 / moments.m00

    println("$label: detected ball area=${moments.m00.toInt()}, centroid=(%.1f, %.1f)".format(cx, cy))

    return Point(cx, cy)
}

/** Convert a pixel coordinate to a world-space ray direction. */
fun pixelToWorldRay(
    u: Double, v: Double,
    width: Double, height: Double,
    vfov: Double,
    forward: DoubleArray, right: DoubleArray, up: DoubleArray
): DoubleArray {
    val aspect = width / height
    val tanHalfVfov = tan(vfov / 2.0)
    val tanHalfHfov = aspect * tanHalfVfov

    // Normalized device coordinates (centered, y-up)
    val nx = (u - width / 2.0) / (width / 2.0)
    val ny = -(v - height / 2.0) / (height / 2.0)

    // Camera-space direction
    val dx = nx * tanHalfHfov
    val dy = ny * tanHalfVfov
    // forward is -z in camera space, so ray = dx*right + dy*up + 1*forward
    val ray = doubleArrayOf(
        dx * right[0] + dy * up[0] + forward[0],
        dx * right[1] + dy * up[1] + forward[1],
        dx * right[2] + dy * up[2] + forward[2]
    )
    return normalize(ray)
}

/**
 * Find the closest point between two rays (midpoint of the closest approach).
 * Ray1: P1 + t * D1,  Ray2: P2 + s * D2
 */
fun closestPointBetweenRays(
    p1: DoubleArray, d1: DoubleArray,
    p2: DoubleArray, d2: DoubleArray
): DoubleArray {
    val w0 = doubleArrayOf(p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2])
    val a = dot(d1, d1)
    val b = dot(d1, d2)
    val c = dot(d2, d2)
    val d = dot(d1, w0)
    val e = dot(d2, w0)

    val denom = a * c - b * b
    if (denom < 1e-10) {
        println("WARNING: rays are nearly parallel")
        return p1
    }

    val t = (b * e - c * d) / denom
    val s = (a * e - b * d) / denom

    // Closest points on each ray
    val closest1 = doubleArrayOf(p1[0] + t * d1[0], p1[1] + t * d1[1], p1[2] + t * d1[2])
    val closest2 = doubleArrayOf(p2[0] + s * d2[0], p2[1] + s * d2[1], p2[2] + s * d2[2])

    val dist = sqrt(
        (closest1[0] - closest2[0]).let { it * it } +
        (closest1[1] - closest2[1]).let { it * it } +
        (closest1[2] - closest2[2]).let { it * it }
    )
    println("Ray closest-approach distance: %.6f".format(dist))

    // Return midpoint
    return doubleArrayOf(
        (closest1[0] + closest2[0]) / 2.0,
        (closest1[1] + closest2[1]) / 2.0,
        (closest1[2] + closest2[2]) / 2.0
    )
}

fun parseCameras(text: String): CameraData {
    val map = text.trim().lines().associate { line ->
        val parts = line.split(" ")
        parts[0] to parts.drop(1).map { it.toDouble() }.toDoubleArray()
    }
    return CameraData(
        leftPos = map["left_cam"] ?: error("Missing left_cam"),
        rightPos = map["right_cam"] ?: error("Missing right_cam"),
        forward = map["direction"] ?: error("Missing direction"),
        up = map["up"] ?: error("Missing up")
    )
}

fun parseSphere(text: String): DoubleArray {
    return text.trim().removePrefix("sphere ").split(" ").map { it.toDouble() }.toDoubleArray()
}

fun normalize(v: DoubleArray): DoubleArray {
    val len = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    return doubleArrayOf(v[0] / len, v[1] / len, v[2] / len)
}

fun cross(a: DoubleArray, b: DoubleArray): DoubleArray = doubleArrayOf(
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0]
)

fun dot(a: DoubleArray, b: DoubleArray): Double =
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
