package dev.sebastianb.ballcatcher.opencv

import org.opencv.core.Point
import kotlin.math.sqrt
import kotlin.test.Test
import kotlin.test.assertTrue

class TriangulateTest {

    // Data from cam/screenshots/2026-04-11_23-12-53-092
    // cameras.txt: left_cam -1.6344 7.2008 0.2718 / right_cam -2.6331 7.2008 0.3244
    // sphere.txt: sphere 0.9086 0.0000 7.2744
    // Camera was looking from ~(-2.1, 7.2, 0.3) — direction and up need to be derived
    // from the original lookAt(0,0,0) at position (5,5,5), then moved by FreeCamController.
    // Since the old cameras.txt didn't save direction/up, we test with known geometry instead.

    @Test
    fun `parseCameras parses all fields`() {
        val text = """
            left_cam -1.6344 7.2008 0.2718
            right_cam -2.6331 7.2008 0.3244
            direction -0.5774 -0.5774 -0.5774
            up -0.4082 0.8165 -0.4082
        """.trimIndent()
        val data = parseCameras(text)
        assertArrayClose(doubleArrayOf(-1.6344, 7.2008, 0.2718), data.leftPos, 1e-4)
        assertArrayClose(doubleArrayOf(-2.6331, 7.2008, 0.3244), data.rightPos, 1e-4)
        assertArrayClose(doubleArrayOf(-0.5774, -0.5774, -0.5774), data.forward, 1e-4)
        assertArrayClose(doubleArrayOf(-0.4082, 0.8165, -0.4082), data.up, 1e-4)
    }

    @Test
    fun `parseSphere parses position`() {
        val result = parseSphere("sphere 0.9086 0.0000 7.2744")
        assertArrayClose(doubleArrayOf(0.9086, 0.0, 7.2744), result, 1e-4)
    }

    @Test
    fun `triangulate with object at origin and cameras at (5,5,5)`() {
        // Cameras at default position looking at origin
        // direction = normalize(-1,-1,-1), up from libGDX lookAt
        val forward = normalize(doubleArrayOf(-1.0, -1.0, -1.0))
        val rightVec = normalize(cross(forward, doubleArrayOf(0.0, 1.0, 0.0)))
        val up = cross(rightVec, forward)

        // eyeOffset = 0.5 along right vector
        val leftPos = doubleArrayOf(
            5.0 - 0.5 * rightVec[0], 5.0 - 0.5 * rightVec[1], 5.0 - 0.5 * rightVec[2]
        )
        val rightPos = doubleArrayOf(
            5.0 + 0.5 * rightVec[0], 5.0 + 0.5 * rightVec[1], 5.0 + 0.5 * rightVec[2]
        )

        val cameraData = CameraData(leftPos, rightPos, forward, up)

        // Object at origin. Cameras are offset along the right vector, so
        // the origin projects slightly off-center horizontally in each eye.
        // Compute the expected pixel positions.
        val vfov = Math.toRadians(67.0)
        val tanHalfVfov = kotlin.math.tan(vfov / 2.0)
        val tanHalfHfov = (640.0 / 720.0) * tanHalfVfov
        val dist = sqrt(leftPos[0] * leftPos[0] + leftPos[1] * leftPos[1] + leftPos[2] * leftPos[2])

        // Project origin into each camera: the offset along the right vector
        // means the object shifts by ±eyeOffset in the right direction
        val leftShift = 0.5 / dist   // object is shifted right relative to left cam
        val rightShift = -0.5 / dist  // object is shifted left relative to right cam

        val leftU = 320.0 + (leftShift / tanHalfHfov) * 320.0
        val rightU = 320.0 + (rightShift / tanHalfHfov) * 320.0

        val result = triangulate(cameraData, Point(leftU, 360.0), Point(rightU, 360.0), 640.0, 720.0)

        val error = sqrt(result[0] * result[0] + result[1] * result[1] + result[2] * result[2])
        assertTrue(error < 0.5, "Expected near origin, got (%.2f, %.2f, %.2f), error=%.2f".format(
            result[0], result[1], result[2], error
        ))
    }

    @Test
    fun `triangulate with object offset from center`() {
        // Camera looking down -Z axis from (0,0,10)
        val forward = doubleArrayOf(0.0, 0.0, -1.0)
        val up = doubleArrayOf(0.0, 1.0, 0.0)
        val rightVec = normalize(cross(forward, up))

        val leftPos = doubleArrayOf(-0.5, 0.0, 10.0)
        val rightPos = doubleArrayOf(0.5, 0.0, 10.0)

        val cameraData = CameraData(leftPos, rightPos, forward, up)

        // Object at (0, 0, 0): distance = 10 along Z
        // For left camera at (-0.5, 0, 10): object is at relative (0.5, 0, -10)
        // pixel_x = 320 + (0.5/10) / tan(hfov/2) * 320
        val vfov = Math.toRadians(67.0)
        val tanHalfVfov = kotlin.math.tan(vfov / 2.0)
        val aspect = 640.0 / 720.0
        val tanHalfHfov = aspect * tanHalfVfov

        // For left cam: dx = 0.5/10 = 0.05, pixel_u = 320 + 0.05/tanHalfHfov * 320
        val leftU = 320.0 + (0.5 / 10.0) / tanHalfHfov * 320.0
        val leftV = 360.0 // centered vertically

        // For right cam: dx = -0.5/10 = -0.05
        val rightU = 320.0 + (-0.5 / 10.0) / tanHalfHfov * 320.0
        val rightV = 360.0

        val result = triangulate(cameraData, Point(leftU, leftV), Point(rightU, rightV), 640.0, 720.0)

        val error = sqrt(result[0] * result[0] + result[1] * result[1] + result[2] * result[2])
        assertTrue(error < 0.1, "Expected near origin, got (%.4f, %.4f, %.4f), error=%.4f".format(
            result[0], result[1], result[2], error
        ))
    }

    @Test
    fun `triangulate with object at known offset`() {
        // Camera looking down -Z from (0, 5, 20)
        val forward = doubleArrayOf(0.0, 0.0, -1.0)
        val up = doubleArrayOf(0.0, 1.0, 0.0)
        val rightVec = normalize(cross(forward, up))

        val leftPos = doubleArrayOf(-0.5, 5.0, 20.0)
        val rightPos = doubleArrayOf(0.5, 5.0, 20.0)

        val cameraData = CameraData(leftPos, rightPos, forward, up)

        // Target at (3, 2, 5) -> relative to camera center (0, 5, 20): (3, -3, -15)
        val targetX = 3.0
        val targetY = 2.0
        val targetZ = 5.0
        val dist = 15.0 // along Z

        val vfov = Math.toRadians(67.0)
        val tanHalfVfov = kotlin.math.tan(vfov / 2.0)
        val aspect = 640.0 / 720.0
        val tanHalfHfov = aspect * tanHalfVfov

        // Left cam at (-0.5, 5, 20): relative to target = (3.5, -3, -15)
        val leftU = 320.0 + (3.5 / dist) / tanHalfHfov * 320.0
        val leftV = 360.0 - (-3.0 / dist) / tanHalfVfov * 360.0

        // Right cam at (0.5, 5, 20): relative to target = (2.5, -3, -15)
        val rightU = 320.0 + (2.5 / dist) / tanHalfHfov * 320.0
        val rightV = 360.0 - (-3.0 / dist) / tanHalfVfov * 360.0

        val result = triangulate(cameraData, Point(leftU, leftV), Point(rightU, rightV), 640.0, 720.0)

        val error = sqrt(
            (result[0] - targetX).let { it * it } +
            (result[1] - targetY).let { it * it } +
            (result[2] - targetZ).let { it * it }
        )
        assertTrue(error < 0.1, "Expected (%.1f, %.1f, %.1f), got (%.4f, %.4f, %.4f), error=%.4f".format(
            targetX, targetY, targetZ, result[0], result[1], result[2], error
        ))
    }

    @Test
    fun `closestPointBetweenRays intersecting rays`() {
        // Two rays that intersect at (1, 1, 1)
        val p1 = doubleArrayOf(0.0, 0.0, 0.0)
        val d1 = normalize(doubleArrayOf(1.0, 1.0, 1.0))
        val p2 = doubleArrayOf(2.0, 0.0, 0.0)
        val d2 = normalize(doubleArrayOf(-1.0, 1.0, 1.0))

        val result = closestPointBetweenRays(p1, d1, p2, d2)

        assertArrayClose(doubleArrayOf(1.0, 1.0, 1.0), result, 1e-6)
    }

    @Test
    fun `normalize produces unit vector`() {
        val v = normalize(doubleArrayOf(3.0, 4.0, 0.0))
        val len = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
        assertTrue(kotlin.math.abs(len - 1.0) < 1e-10)
        assertArrayClose(doubleArrayOf(0.6, 0.8, 0.0), v, 1e-10)
    }

    @Test
    fun `cross product is correct`() {
        val result = cross(doubleArrayOf(1.0, 0.0, 0.0), doubleArrayOf(0.0, 1.0, 0.0))
        assertArrayClose(doubleArrayOf(0.0, 0.0, 1.0), result, 1e-10)
    }

    private fun assertArrayClose(expected: DoubleArray, actual: DoubleArray, tolerance: Double) {
        assertTrue(expected.size == actual.size, "Array size mismatch")
        for (i in expected.indices) {
            assertTrue(
                kotlin.math.abs(expected[i] - actual[i]) < tolerance,
                "Index $i: expected ${expected[i]}, got ${actual[i]}"
            )
        }
    }
}
