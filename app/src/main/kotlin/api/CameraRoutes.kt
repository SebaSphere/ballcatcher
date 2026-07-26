package dev.sebastianb.ballcatcher.app.api

import dev.sebastianb.ballcatcher.app.api.dto.CommandResponse
import dev.sebastianb.ballcatcher.app.camera.FisheyeCalibrationResult
import dev.sebastianb.ballcatcher.app.camera.FisheyeCalibrator
import dev.sebastianb.ballcatcher.app.camera.FisheyeUndistorter
import dev.sebastianb.ballcatcher.app.camera.BallTracker
import dev.sebastianb.ballcatcher.app.camera.FaceTracker
import dev.sebastianb.ballcatcher.app.camera.RawPhotoCapture
import dev.sebastianb.ballcatcher.app.camera.ScanCycle
import dev.sebastianb.ballcatcher.app.camera.StereoCalibrationCapture
import dev.sebastianb.ballcatcher.app.ship.YawJointController
import io.ktor.http.*
import io.ktor.server.request.*
import io.ktor.server.response.*
import io.ktor.server.routing.*
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import kotlinx.serialization.Serializable
import java.time.LocalDateTime
import java.time.format.DateTimeFormatter

@Serializable
data class CalibrationCaptureRequest(val count: Int, val delayMs: Long = 0)

@Serializable
data class RawCaptureResponse(val timestamp: String, val rightPath: String, val leftPath: String)

@Serializable
data class FisheyeCalibrateRequest(
    val imageDir: String = ".",
    val boardWidth: Int = 7,
    val boardHeight: Int = 7,
)

private const val FISHEYE_CALIBRATION_PATH = "fisheye_calibration.yml"

@Serializable
data class TrackBallRequest(val flipPanDirection: Boolean = false)

@Serializable
data class TrackFaceRequest(val flipPanDirection: Boolean = false)

@Serializable
data class ScanCycleRequest(
    val count: Int,
    val cycles: Int = 1,
    val sweepSpeed: Int = 200,
    val capturePauseMs: Long = 500,
)

fun Route.cameraRoutes(controller: YawJointController) {
    val cameraScope = CoroutineScope(Dispatchers.Default)
    var ballTracker: BallTracker? = null
    var faceTracker: FaceTracker? = null

    route("/camera") {
        post("/fisheye-calibrate") {
            val request = call.receive<FisheyeCalibrateRequest>()
            val boardSize = org.opencv.core.Size(request.boardWidth.toDouble(), request.boardHeight.toDouble())
            val savePath = java.io.File(FISHEYE_CALIBRATION_PATH).absolutePath
            try {
                withContext(Dispatchers.IO) {
                    FisheyeCalibrator(boardSize)
                        .calibrate(request.imageDir)
                        .save(FISHEYE_CALIBRATION_PATH)
                }
                call.respond(CommandResponse(true, "Fisheye calibration complete — saved to $savePath"))
            } catch (e: Exception) {
                println("CameraRoutes: fisheye calibration failed — ${e.message}")
                e.printStackTrace()
                call.respond(HttpStatusCode.InternalServerError, CommandResponse(false, "Fisheye calibration failed: ${e.message}"))
            }
        }

        post("/calibration-capture") {
            val request = call.receive<CalibrationCaptureRequest>()
            val capture = StereoCalibrationCapture()
            val saved = withContext(Dispatchers.IO) {
                capture.capture(request.count, request.delayMs)
            }
            call.respond(CommandResponse(true, "Captured $saved calibration image pairs"))
        }

        post("/raw-capture") {
            val result = withContext(Dispatchers.IO) {
                RawPhotoCapture().capture()
            }
            call.respond(RawCaptureResponse(result.timestamp, result.rightPath, result.leftPath))
        }

        route("/track-ball") {
            post("/start") {
                if (ballTracker?.isTracking == true) {
                    call.respond(HttpStatusCode.Conflict, CommandResponse(false, "Ball tracking is already active"))
                    return@post
                }
                val request = call.receive<TrackBallRequest>()
                ballTracker = BallTracker(controller, flipPanDirection = request.flipPanDirection)
                ballTracker!!.start(cameraScope)
                call.respond(CommandResponse(true, "Ball tracking started (flipPanDirection=${request.flipPanDirection})"))
            }

            post("/stop") {
                val tracker = ballTracker
                if (tracker == null || !tracker.isTracking) {
                    call.respond(HttpStatusCode.Conflict, CommandResponse(false, "Ball tracking is not active"))
                    return@post
                }
                tracker.stop()
                ballTracker = null
                call.respond(CommandResponse(true, "Ball tracking stopped"))
            }
        }

        route("/track-face") {
            post("/start") {
                if (faceTracker?.isTracking == true) {
                    call.respond(HttpStatusCode.Conflict, CommandResponse(false, "Face tracking is already active"))
                    return@post
                }
                val request = call.receive<TrackFaceRequest>()
                faceTracker = FaceTracker(controller, flipPanDirection = request.flipPanDirection)
                faceTracker!!.start(cameraScope)
                call.respond(CommandResponse(true, "Face tracking started (flipPanDirection=${request.flipPanDirection})"))
            }

            post("/stop") {
                val tracker = faceTracker
                if (tracker == null || !tracker.isTracking) {
                    call.respond(HttpStatusCode.Conflict, CommandResponse(false, "Face tracking is not active"))
                    return@post
                }
                tracker.stop()
                faceTracker = null
                call.respond(CommandResponse(true, "Face tracking stopped"))
            }
        }

        post("/scan-cycle") {
            val leftAngle = controller.calibratedLeftAngle
            val rightAngle = controller.calibratedRightAngle

            if (leftAngle == null || rightAngle == null) {
                call.respond(
                    HttpStatusCode.Conflict,
                    CommandResponse(false, "Motor has not been calibrated yet — run /motor/calibrate-home first")
                )
                return@post
            }

            val request = call.receive<ScanCycleRequest>()
            val sessionTimestamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyy-MM-dd_HH-mm-ss-SSS"))
            val calibration = FisheyeCalibrationResult.load(FISHEYE_CALIBRATION_PATH)
            val undistorterR = calibration?.let { FisheyeUndistorter(it.kR, it.dR, it.imageSize) }
            val undistorterL = calibration?.let { FisheyeUndistorter(it.kL, it.dL, it.imageSize) }
            if (calibration != null) println("CameraRoutes: fisheye calibration loaded — undistortion active")
            val scanCycle = ScanCycle(leftAngle, rightAngle, controller, undistorterR = undistorterR, undistorterL = undistorterL)

            cameraScope.launch {
                for (cycle in 1..request.cycles) {
                    scanCycle.cycle(
                        sessionTimestamp = sessionTimestamp,
                        cycleNumber = cycle,
                        count = request.count,
                        sweepSpeed = request.sweepSpeed,
                        capturePauseMs = request.capturePauseMs,
                        firstRun = cycle == 1,
                    )
                }
            }

            call.respond(CommandResponse(true, "Scan session started — $sessionTimestamp, ${request.cycles} cycle(s), ${request.count} points each"))
        }
    }
}
