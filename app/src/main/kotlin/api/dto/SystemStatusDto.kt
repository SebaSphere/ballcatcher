package dev.sebastianb.ballcatcher.app.api.dto

import dev.sebastianb.ballcatcher.app.ship.YawJointController
import kotlinx.serialization.Serializable

@Serializable
data class SystemStatusDto(
    val motorState: MotorStateDto,
    val motorFeedback: MotorFeedbackDto,
    val leftSwitchSteps: Long,
    val rightSwitchSteps: Long,
) {
    companion object {
        fun from(controller: YawJointController) = SystemStatusDto(
            motorState = MotorStateDto.from(controller.motorState),
            motorFeedback = MotorFeedbackDto.from(controller.motorFeedback),
            leftSwitchSteps = controller.leftSwitchSteps,
            rightSwitchSteps = controller.rightSwitchSteps,
        )
    }
}
