package dev.sebastianb.ballcatcher.app.api.dto

import dev.sebastianb.ballcatcher.app.motor.MotorState
import kotlinx.serialization.Serializable

@Serializable
data class MotorStateDto(
    val state: String,
) {
    companion object {
        fun from(motorState: MotorState) = MotorStateDto(
            state = when (motorState) {
                is MotorState.Uninitialized -> "uninitialized"
                is MotorState.Idle -> "idle"
                is MotorState.Homing -> "homing"
                is MotorState.Moving -> "moving"
            }
        )
    }
}
