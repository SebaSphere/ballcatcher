package dev.sebastianb.ballcatcher.app.api

import dev.sebastianb.ballcatcher.app.api.dto.HeadPositionDto
import dev.sebastianb.ballcatcher.app.ship.HeadRotationalPosition
import dev.sebastianb.ballcatcher.app.ship.YawJointController
import io.ktor.server.response.*
import io.ktor.server.routing.*

fun Route.shipRoutes(controller: YawJointController) {

    route("/ship") {

        get("/position") {
            val yaw = controller.motorFeedback.currentAngle.toFloat()
            val position = HeadRotationalPosition(yaw = yaw, pitch = 0f)
            call.respond(HeadPositionDto.from(position))
        }
    }
}
