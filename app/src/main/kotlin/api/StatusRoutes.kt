package dev.sebastianb.ballcatcher.app.api

import dev.sebastianb.ballcatcher.app.api.dto.SystemStatusDto
import dev.sebastianb.ballcatcher.app.ship.YawJointController
import io.ktor.server.response.*
import io.ktor.server.routing.*

fun Route.statusRoutes(controller: YawJointController) {

    get("/status") {
        call.respond(SystemStatusDto.from(controller))
    }
}
