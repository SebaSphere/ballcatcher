package dev.sebastianb.ballcatcher.app.api

import dev.sebastianb.ballcatcher.app.api.dto.*
import dev.sebastianb.ballcatcher.app.ship.YawJointController
import io.ktor.server.request.*
import io.ktor.server.response.*
import io.ktor.server.routing.*
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch

fun Route.motorRoutes(controller: YawJointController) {
    val motorScope = CoroutineScope(Dispatchers.Default)

    route("/motor") {

        get("/feedback") {
            val dto = MotorFeedbackDto.from(controller.motorFeedback)
            call.respond(dto)
        }

        get("/state") {
            val dto = MotorStateDto.from(controller.motorState)
            call.respond(dto)
        }

        post("/move-to-angle") {
            val request = call.receive<MoveToAngleRequest>()
            motorScope.launch { controller.motorControl.moveToAngle(request.degrees) }
            call.respond(CommandResponse(true, "Moving to angle ${request.degrees}°"))
        }

        post("/move-to-position") {
            val request = call.receive<MoveToPositionRequest>()
            motorScope.launch { controller.moveToPosition(request.fraction) }
            call.respond(CommandResponse(true, "Moving to position ${request.fraction}"))
        }

        post("/move-clockwise") {
            val request = call.receive<MoveClockwiseRequest>()
            motorScope.launch { controller.motorControl.moveClockwise(request.degrees) }
            call.respond(CommandResponse(true, "Moving ${request.degrees}° clockwise"))
        }

        post("/move-counter-clockwise") {
            val request = call.receive<MoveCounterClockwiseRequest>()
            motorScope.launch { controller.motorControl.moveCounterClockwise(request.degrees) }
            call.respond(CommandResponse(true, "Moving ${request.degrees}° counter-clockwise"))
        }

        post("/calibrate-home") {
            motorScope.launch { controller.motorControl.calibrateHome() }
            call.respond(CommandResponse(true, "Calibrating home"))
        }

        post("/stop") {
            controller.motorControl.stop()
            call.respond(CommandResponse(true, "Motor stopped"))
        }

        post("/enable") {
            val request = call.receive<SetEnabledRequest>()
            controller.motorControl.setEnabled(request.enabled)
            call.respond(CommandResponse(true, "Motor enabled: ${request.enabled}"))
        }
    }
}
