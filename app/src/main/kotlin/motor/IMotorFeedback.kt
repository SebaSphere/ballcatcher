package dev.sebastianb.ballcatcher.app.motor

interface IMotorFeedback {
    val minAngle: Double
        get() = 0.0

    // the bounds of the robot so it doesn't hit the frame. Yaw should be 360 degrees, pitch could be 45?
    val maxAngle: Double
    val currentAngle: Double
    val currentRawSteps: Long
    // degrees per second
    val angularVelocity: Double
    // hardware sensors
    val isAtLeftSwitch: Boolean
        get() = false
    val isAtRightSwitch: Boolean
        get() = false
    val isAtLimitSwitch: Boolean
        get() = isAtLeftSwitch || isAtRightSwitch
    val isOn: Boolean
}