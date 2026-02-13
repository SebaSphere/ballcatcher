package dev.sebastianb.ballcatcher.app.motor

sealed class MotorState {
    object Uninitialized : MotorState()
    object Idle : MotorState()
    object Homing : MotorState()

    data class Moving(val motorFeedback: IMotorFeedback) : MotorState()
}