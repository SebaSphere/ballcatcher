package dev.sebastianb.ballcatcher.app.api

import dev.sebastianb.ballcatcher.app.api.dto.CommandResponse
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
data class ScanCycleRequest(
    val count: Int,
    val cycles: Int = 1,
    val sweepSpeed: Int = 200,
    val capturePauseMs: Long = 500,
)

fun Route.cameraRoutes(controller: YawJointController) {
    val cameraScope = CoroutineScope(Dispatchers.Default)

    route("/camera") {
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
            val scanCycle = ScanCycle(leftAngle, rightAngle, controller)

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
