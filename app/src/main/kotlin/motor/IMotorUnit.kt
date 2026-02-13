package dev.sebastianb.ballcatcher.app.motor

interface IMotorUnit {
    val motorControl: IMotorControl
    val motorFeedback: IMotorFeedback
    val motorState: MotorState

    // makes executive decisions about the motor, high level logic on stopping/starting
    fun update()
}