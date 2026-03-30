package dev.sebastianb.ballcatcher.app.api.dto

import dev.sebastianb.ballcatcher.app.motor.IMotorFeedback
import kotlinx.serialization.Serializable

@Serializable
data class MotorFeedbackDto(
    val minAngle: Double,
    val maxAngle: Double,
    val currentAngle: Double,
    val currentRawSteps: Long,
    val angularVelocity: Double,
    val isAtLeftSwitch: Boolean,
    val isAtRightSwitch: Boolean,
    val isAtLimitSwitch: Boolean,
    val isOn: Boolean,
) {
    companion object {
        fun from(feedback: IMotorFeedback) = MotorFeedbackDto(
            minAngle = feedback.minAngle,
            maxAngle = feedback.maxAngle,
            currentAngle = feedback.currentAngle,
            currentRawSteps = feedback.currentRawSteps,
            angularVelocity = feedback.angularVelocity,
            isAtLeftSwitch = feedback.isAtLeftSwitch,
            isAtRightSwitch = feedback.isAtRightSwitch,
            isAtLimitSwitch = feedback.isAtLimitSwitch,
            isOn = feedback.isOn,
        )
    }
}
