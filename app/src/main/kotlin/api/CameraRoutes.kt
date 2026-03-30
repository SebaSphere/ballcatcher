package dev.sebastianb.ballcatcher.app.api

import dev.sebastianb.ballcatcher.app.api.dto.CommandResponse
import dev.sebastianb.ballcatcher.app.camera.StereoCalibrationCapture
import io.ktor.server.request.*
import io.ktor.server.response.*
import io.ktor.server.routing.*
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import kotlinx.serialization.Serializable

@Serializable
data class CalibrationCaptureRequest(val count: Int, val delayMs: Long = 0)

fun Route.cameraRoutes() {
    route("/camera") {
        post("/calibration-capture") {
            val request = call.receive<CalibrationCaptureRequest>()
            val capture = StereoCalibrationCapture()
            val saved = withContext(Dispatchers.IO) {
                capture.capture(request.count, request.delayMs)
            }
            call.respond(CommandResponse(true, "Captured $saved calibration image pairs"))
        }
    }
}
